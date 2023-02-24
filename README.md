# tinygorfm9x
A tinygo library to run an rfm9x module. Based off of [code by @cyraxx](https://github.com/cyraxx/node-rfm9x/) and [this adafruit article](https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test).

See the [example](/example/) for usage.

## Tasks

### Upload
Upload the example code to the device.
```
tinygo flash -target challenger-rp2040 example/main.go
```

### Edit
Open vscode with tinygo autocomplete
```
tinygo-edit --target challenger-rp2040 --editor code
```
