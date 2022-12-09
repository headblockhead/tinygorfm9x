package main

import (
	"machine"

	"github.com/headblockhead/tinygoRFM9X"
)

func main() {
	device := tinygoRFM9X.RFM9x{}
	device.SpiDevice = *machine.SPI1
	err := device.Init(tinygoRFM9X.Options{
		FrequencyMhz:    868,
		BandwidthHz:     500000,
		CodingRate:      5,
		SpreadingFactor: 7,
	})
	if err != nil {
		print("Error initializing device: %s", err)
	}
}
