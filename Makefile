.PHONY: all compile upload monitor engine device
all: compile upload engine
clean:
	arduino-cli compile --clean device
compile:
	arduino-cli compile device
upload:
	arduino-cli upload device
monitor:
	arduino-cli monitor device
device: compile upload monitor
lsp:
	arduino-cli compile --clean --fqbn esp32:esp32:esp32s3 --only-compilation-database --build-path ./build ./device/
	rm compile_commands.json
	cp build/compile_commands.json .

engine:
	cd engine && cargo run
