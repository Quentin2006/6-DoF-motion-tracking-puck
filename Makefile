all:
	arduino-cli compile device
	arduino-cli upload device
	arduino-cli monitor device
