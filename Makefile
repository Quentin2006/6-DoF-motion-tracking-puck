.PHONY: all clean lsp compile upload monitor engine device test
FQBN ?= esp32:esp32:esp32s3:CDCOnBoot=cdc
# Firmware port. The puck's own USB port (USB-Serial-JTAG) gets a stable
# /dev/serial/by-id node whatever order things were plugged in — the ttyACM
# index does not. Fall back to the historical literal, which is where a
# bridge-only (1a86:55d3) setup enumerates. Override with PORT=<node>.
PORT ?= $(firstword $(wildcard /dev/serial/by-id/usb-Espressif_USB_JTAG_*) /dev/ttyACM0)

all: compile upload engine
clean:
	arduino-cli compile --clean --fqbn $(FQBN) device
compile:
	arduino-cli compile --fqbn $(FQBN) device
upload:
	arduino-cli upload --fqbn $(FQBN) -p $(PORT) device
monitor:
	arduino-cli monitor -p $(PORT) device
device: compile upload monitor
lsp:
	arduino-cli compile --clean --fqbn $(FQBN) --only-compilation-database --build-path ./build ./device/
	jq '[.[] | .file |= (sub("build/sketch/"; "device/") | sub("\\.ino\\.cpp$$"; ".ino")) | . as $$e | .arguments |= ([.[0]] + (if ($$e.file | endswith(".ino")) then ["-x", "c++"] else [] end) + (.[1:] | map(select(. != "-MMD" and . != "-o" and (test("\\.o$$") | not) and (test("build/file_opts$$") | not)) | gsub("build/sketch/"; "device/") | sub("\\.ino\\.cpp$$"; ".ino"))))]' build/compile_commands.json > compile_commands.json

engine:
	cd engine && cargo run

# Cross-contract + engine tests. The C++ host test asserts the firmware packet
# producer (device/packet.h) matches tests/packet_fixture.txt; cargo test asserts
# the engine parser matches the same bytes. Requires a host g++.
# tests/video_test.cpp is built with ASAN so "the assembler never writes past
# its buffer" is a real assertion (redzone), not a code-reading exercise.
test:
	mkdir -p build
	g++ -std=c++17 -Idevice tests/packet_test.cpp -o build/packet_test
	./build/packet_test tests/packet_fixture.txt
	g++ -std=c++17 -Idevice tests/frame_test.cpp -o build/frame_test
	./build/frame_test
	g++ -std=c++17 -fsanitize=address -Idevice tests/video_test.cpp -o build/video_test
	./build/video_test
	cd engine && cargo test
