use std::cell::Cell;
use std::ffi::{OsStr, OsString};
use std::io::{self, ErrorKind, Write};
use std::sync::Arc;
use std::time::{Duration, Instant};

use rusb::{DeviceHandle, Direction, GlobalContext, TransferType};

pub const PUCK_USB_VID: u16 = 0x303a;
pub const PUCK_USB_PID: u16 = 0x1002;

const BENCHMARK_BYTES: usize = 64 * 1024;
const BENCHMARK_DURATION: Duration = Duration::from_secs(10);

fn benchmark_requested(args: impl IntoIterator<Item = OsString>) -> bool {
    args.into_iter()
        .skip(1)
        .any(|arg| arg == OsStr::new("--bulk-benchmark"))
}

fn bytes_per_second(bytes: u64, elapsed: Duration) -> u64 {
    let nanos = elapsed.as_nanos();
    if nanos == 0 {
        0
    } else {
        ((u128::from(bytes) * 1_000_000_000) / nanos).min(u128::from(u64::MAX)) as u64
    }
}

fn run_benchmark(mut writer: BulkLink) -> io::Result<()> {
    let buffer = vec![0u8; BENCHMARK_BYTES];
    let start = Instant::now();
    let mut next_report = start + Duration::from_secs(1);
    let mut report_start = start;
    let mut report_sent = 0u64;
    let mut sent = 0u64;
    let mut offset = 0usize;

    while start.elapsed() < BENCHMARK_DURATION {
        if offset == buffer.len() {
            offset = 0;
        }
        let written = match writer.write(&buffer[offset..]) {
            Ok(0) => Err(io::Error::new(
                ErrorKind::WriteZero,
                "bulk benchmark zero-byte write",
            )),
            Ok(n) => Ok(n),
            Err(e) => Err(e),
        };
        let n = match written {
            Ok(n) => n,
            Err(e) => {
                eprintln!(
                    "bulk benchmark sent={sent} errors=1 rate={}",
                    bytes_per_second(sent, start.elapsed())
                );
                return Err(e);
            }
        };
        sent += n as u64;
        offset += n;

        let now = Instant::now();
        if now >= next_report {
            println!(
                "bulk benchmark sent={sent} errors=0 rate={}",
                bytes_per_second(sent - report_sent, now.duration_since(report_start))
            );
            report_start = now;
            report_sent = sent;
            next_report = now + Duration::from_secs(1);
        }
    }

    println!(
        "bulk benchmark total sent={sent} errors=0 rate={}",
        bytes_per_second(sent, start.elapsed())
    );
    Ok(())
}

pub fn run_benchmark_if_requested() -> Option<io::Result<()>> {
    benchmark_requested(std::env::args_os()).then(|| run_benchmark(BulkLink::open()))
}

const READ_TIMEOUT: Duration = Duration::from_millis(100);
const WRITE_TIMEOUT: Duration = Duration::from_millis(250);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct EndpointCandidate {
    address: u8,
    direction: Direction,
    transfer_type: TransferType,
}

#[derive(Debug)]
struct InterfaceCandidate {
    interface: u8,
    setting: u8,
    class_code: u8,
    sub_class_code: u8,
    protocol_code: u8,
    endpoints: Vec<EndpointCandidate>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct EndpointPair {
    interface: u8,
    bulk_in: u8,
    bulk_out: u8,
}

fn select_endpoints(candidates: &[InterfaceCandidate]) -> Result<EndpointPair, &'static str> {
    let mut selected = None;
    for candidate in candidates {
        if candidate.setting != 0
            || candidate.class_code != 0xff
            || candidate.sub_class_code != 0
            || candidate.protocol_code != 0
        {
            continue;
        }

        let mut bulk_in = None;
        let mut bulk_out = None;
        let mut valid = true;
        for endpoint in &candidate.endpoints {
            if endpoint.transfer_type != TransferType::Bulk {
                continue;
            }
            let slot = match endpoint.direction {
                Direction::In => &mut bulk_in,
                Direction::Out => &mut bulk_out,
            };
            if slot.is_some() {
                valid = false;
                break;
            }
            *slot = Some(endpoint.address);
        }
        if !valid || bulk_in.is_none() || bulk_out.is_none() {
            continue;
        }
        if selected.is_some() {
            return Err("ambiguous vendor bulk interface");
        }
        selected = Some(EndpointPair {
            interface: candidate.interface,
            bulk_in: bulk_in.unwrap(),
            bulk_out: bulk_out.unwrap(),
        });
    }
    selected.ok_or("no vendor bulk interface with exactly one bulk IN and OUT endpoint")
}

#[derive(Clone)]
pub struct BulkLink {
    handle: Arc<DeviceHandle<GlobalContext>>,
    interface: u8,
    bulk_in: u8,
    bulk_out: u8,
    write_after_partial: Cell<bool>,
}

impl BulkLink {
    pub fn open() -> Self {
        let devices = rusb::devices().unwrap_or_else(|e| {
            panic!("enumerate USB devices: {e}; run `make usb-permissions` if access is denied")
        });
        let mut matches = Vec::new();
        for device in devices.iter() {
            let descriptor = device
                .device_descriptor()
                .unwrap_or_else(|e| panic!("read USB device descriptor: {e}"));
            if descriptor.vendor_id() == PUCK_USB_VID && descriptor.product_id() == PUCK_USB_PID {
                matches.push(device);
            }
        }
        match matches.len() {
            0 => panic!(
                "puck USB {PUCK_USB_VID:04x}:{PUCK_USB_PID:04x} not found; flash the OTG firmware and reconnect native USB"
            ),
            1 => {}
            n => panic!("expected one puck USB {PUCK_USB_VID:04x}:{PUCK_USB_PID:04x}, found {n}"),
        }

        let device = matches.pop().unwrap();
        let handle = device.open().unwrap_or_else(|e| {
            if e == rusb::Error::Access {
                panic!(
                    "open puck USB {PUCK_USB_VID:04x}:{PUCK_USB_PID:04x}: permission denied; run `make usb-permissions`, then unplug and replug native USB ({e})"
                );
            }
            panic!("open puck USB {PUCK_USB_VID:04x}:{PUCK_USB_PID:04x}: {e}")
        });
        handle
            .set_auto_detach_kernel_driver(true)
            .unwrap_or_else(|e| panic!("enable kernel-driver auto-detach: {e}"));

        let config = device
            .active_config_descriptor()
            .unwrap_or_else(|e| panic!("read active USB configuration: {e}"));
        let mut candidates = Vec::new();
        for interface in config.interfaces() {
            for setting in interface.descriptors() {
                candidates.push(InterfaceCandidate {
                    interface: setting.interface_number(),
                    setting: setting.setting_number(),
                    class_code: setting.class_code(),
                    sub_class_code: setting.sub_class_code(),
                    protocol_code: setting.protocol_code(),
                    endpoints: setting
                        .endpoint_descriptors()
                        .map(|endpoint| EndpointCandidate {
                            address: endpoint.address(),
                            direction: endpoint.direction(),
                            transfer_type: endpoint.transfer_type(),
                        })
                        .collect(),
                });
            }
        }
        let endpoints = select_endpoints(&candidates).unwrap_or_else(|e| {
            panic!("select {PUCK_USB_VID:04x}:{PUCK_USB_PID:04x} endpoints: {e}")
        });
        handle
            .claim_interface(endpoints.interface)
            .unwrap_or_else(|e| panic!("claim USB interface {}: {e}", endpoints.interface));

        println!(
            "puck bulk {:04x}:{:04x} interface {} in=0x{:02x} out=0x{:02x}",
            PUCK_USB_VID, PUCK_USB_PID, endpoints.interface, endpoints.bulk_in, endpoints.bulk_out
        );
        Self {
            handle: Arc::new(handle),
            interface: endpoints.interface,
            bulk_in: endpoints.bulk_in,
            bulk_out: endpoints.bulk_out,
            write_after_partial: Cell::new(false),
        }
    }

    fn transfer_error(&self, operation: &str, endpoint: u8, error: rusb::Error) -> io::Error {
        let message = format!(
            "USB interface {} {operation} endpoint 0x{endpoint:02x}: {error}",
            self.interface
        );
        match error {
            rusb::Error::Timeout => io::Error::new(ErrorKind::TimedOut, message),
            rusb::Error::Interrupted => io::Error::new(ErrorKind::Interrupted, message),
            _ => io::Error::other(message),
        }
    }

    pub fn read_bulk(&self, buffer: &mut [u8]) -> io::Result<usize> {
        self.handle
            .read_bulk(self.bulk_in, buffer, READ_TIMEOUT)
            .map_err(|e| self.transfer_error("bulk IN", self.bulk_in, e))
    }

    pub fn write_bulk(&self, buffer: &[u8]) -> io::Result<usize> {
        if buffer.is_empty() {
            return Ok(0);
        }
        let written = self
            .handle
            .write_bulk(self.bulk_out, buffer, WRITE_TIMEOUT)
            .map_err(|e| self.transfer_error("bulk OUT", self.bulk_out, e))?;
        if written == 0 {
            Err(io::Error::new(
                ErrorKind::WriteZero,
                format!(
                    "USB interface {} bulk OUT endpoint 0x{:02x}: zero-byte write",
                    self.interface, self.bulk_out
                ),
            ))
        } else {
            Ok(written)
        }
    }
}

impl Write for BulkLink {
    fn write(&mut self, buffer: &[u8]) -> io::Result<usize> {
        let result = self.write_bulk(buffer);
        match result {
            Ok(n) => {
                self.write_after_partial.set(n < buffer.len());
                Ok(n)
            }
            Err(e) => {
                let after_partial = self.write_after_partial.replace(false);
                if after_partial && matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::Interrupted)
                {
                    Err(io::Error::other(format!(
                        "{e}; previous bulk OUT transfer was partial"
                    )))
                } else {
                    Err(e)
                }
            }
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn candidate(interface: u8, bulk_in: u8, bulk_out: u8) -> InterfaceCandidate {
        InterfaceCandidate {
            interface,
            setting: 0,
            class_code: 0xff,
            sub_class_code: 0,
            protocol_code: 0,
            endpoints: vec![
                EndpointCandidate {
                    address: bulk_in,
                    direction: Direction::In,
                    transfer_type: TransferType::Bulk,
                },
                EndpointCandidate {
                    address: bulk_out,
                    direction: Direction::Out,
                    transfer_type: TransferType::Bulk,
                },
            ],
        }
    }

    #[test]
    fn benchmark_flag_is_exact_and_ignores_the_program_name() {
        assert!(benchmark_requested(
            [OsString::from("engine"), OsString::from("--bulk-benchmark")].into_iter()
        ));
        assert!(!benchmark_requested(
            [
                OsString::from("engine"),
                OsString::from("--bulk-benchmark=10"),
                OsString::from("bulk-benchmark"),
            ]
            .into_iter()
        ));
    }

    #[test]
    fn endpoint_selection_uses_descriptor_addresses_and_rejects_ambiguity() {
        assert_eq!(
            select_endpoints(&[candidate(4, 0x84, 0x04)]),
            Ok(EndpointPair {
                interface: 4,
                bulk_in: 0x84,
                bulk_out: 0x04,
            })
        );
        assert!(select_endpoints(&[candidate(4, 0x84, 0x04), candidate(5, 0x85, 0x05)]).is_err());
    }
}
