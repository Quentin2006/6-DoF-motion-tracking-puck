.PHONY: all compile-db engine device
all:
	arduino-cli compile device
	arduino-cli upload device
	make engine

monitor:
	arduino-cli monitor device

compile-db:
	arduino-cli compile --only-compilation-database --build-path build device

engine:
	cd engine && make clean all run && cd ..

device:
	arduino-cli compile device
	arduino-cli upload device
	arduino-cli monitor device
