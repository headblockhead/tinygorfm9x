package main

import (
	"machine"

	"github.com/headblockhead/tinygoRFM9X"
)

func main() {
	device := tinygoRFM9X.RFM9x{
		SpiDevice: *machine.SPI1,
	}
	err := device.Init(tinygoRFM9X.Options{
		FrequencyMhz:    868,
		BandwidthHz:     500000,
		CodingRate:      5,
		SpreadingFactor: 7,
		ResetPin:        machine.GPIO13,
		Dio0Pin:         machine.GPIO14,
		Dio1Pin:         machine.GPIO15,
		Dio2Pin:         machine.GPIO18,
	})
	if err != nil {
		print("Error initializing device: %s", err)
	}
}
