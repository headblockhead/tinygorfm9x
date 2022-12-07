package main

import (
	"machine"
	"time"

	"github.com/headblockhead/tinygoRFM9X"
)

func main() {
	time.Sleep(2 * time.Second)
	print("Starting up...\r\n")
	device := tinygoRFM9X.RFM9x{}
	print("Starting up2...\r\n")
	device.SpiDevice = *machine.SPI1
	device.Dio0Gpio = machine.GPIO14
	err := device.Init(tinygoRFM9X.Options{
		FrequencyMhz:    868,
		BandwidthHz:     500000,
		CodingRate:      5,
		SpreadingFactor: 7,
	})
	print("Starting up3...\r\n")
	if err != nil {
		print("Error initializing device: %s", err)
	}
	print("Device initialized")
	err = device.Send([]byte("PING"))
	if err != nil {
		print("Error sending packet: %s", err)
	}
	print("Packet sent")
	device.StartRecieve()
	for {
		pac := <-device.RecievedPackets
		print("%s", pac.Payload)
		if string(pac.Payload) == "PING" {
			err := device.Send([]byte("PONG"))
			if err != nil {
				print("Error sending packet: %s", err)
			}
			device.StartRecieve()
		}
	}
}
