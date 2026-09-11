.PHONY: all compile upload monitor engine device
all: compile upload engine
compile:
	arduino-cli compile device
upload:
	arduino-cli upload device
monitor:
	arduino-cli monitor device
device: compile upload monitor
engine:
	cd engine && cargo run
