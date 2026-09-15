.PHONY: all compile upload monitor engine device
FQBN ?= esp32:esp32:esp32s3

all: compile upload engine
clean:
	arduino-cli compile --clean device
compile:
	arduino-cli compile --fqbn $(FQBN) device
upload:
	arduino-cli upload --fqbn $(FQBN) -p /dev/ttyACM0 device
monitor:
	arduino-cli monitor device
device: compile upload monitor
lsp:
	arduino-cli compile --clean --fqbn $(FQBN) --only-compilation-database --build-path ./build ./device/
	jq '[.[] | .file |= (sub("build/sketch/"; "device/") | sub("\\.ino\\.cpp$$"; ".ino")) | . as $$e | .arguments |= ([.[0]] + (if ($$e.file | endswith(".ino")) then ["-x", "c++"] else [] end) + (.[1:] | map(select(. != "-MMD" and . != "-o" and (test("\\.o$$") | not) and (test("build/file_opts$$") | not)) | gsub("build/sketch/"; "device/") | sub("\\.ino\\.cpp$$"; ".ino"))))]' build/compile_commands.json > compile_commands.json

engine:
	cd engine && cargo run
