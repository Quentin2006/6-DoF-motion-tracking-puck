.PHONY: all clean lsp compile upload monitor usb-permissions engine device test
FQBN ?= esp32:esp32:esp32s3:USBMode=default,CDCOnBoot=default,UploadMode=default
# CH343 UART bridge for flashing, logs, and calibration. The native USB-OTG
# connector is owned by the engine's vendor-bulk link and never appears as a
# tty device. Override with PORT=<node>.
PORT ?= $(firstword $(wildcard /dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00) /dev/ttyACM0)

all: compile upload engine
clean:
	arduino-cli compile --clean --fqbn $(FQBN) device
compile:
	arduino-cli compile --fqbn $(FQBN) device
upload:
	arduino-cli upload --fqbn $(FQBN) -p $(PORT) device
monitor:
	arduino-cli monitor -p $(PORT) device
usb-permissions:
	sudo install -m 0644 engine/udev/99-puck-usb.rules /etc/udev/rules.d/99-puck-usb.rules
	sudo udevadm control --reload-rules
	sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=303a --attr-match=idProduct=1002
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
