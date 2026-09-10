.PHONY: all compile-db engine device
all:
	arduino-cli compile device
	arduino-cli upload device
	make engine

monitor:
	arduino-cli monitor device

engine:
	cd engine && cargo run

device:
	arduino-cli compile device
	arduino-cli upload device
	arduino-cli monitor device
