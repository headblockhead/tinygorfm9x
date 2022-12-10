package main

import (
	"machine"
	"time"

	"github.com/headblockhead/tinygoRFM9X"
)

func main() {
	time.Sleep(1 * time.Second)
	device := tinygoRFM9X.RFM9x{
		SpiDevice: *machine.SPI1,
	}
	print("Initializing device\r\n")
	err := device.Init(tinygoRFM9X.Options{
		FrequencyMhz: 868,
		ResetPin:     machine.LORA_RESET,
		CSPin:        machine.LORA_CS,
		Dio0Pin:      machine.LORA_DIO0,
		Dio1Pin:      machine.LORA_DIO1,
		Dio2Pin:      machine.LORA_DIO2,
		TxTimeoutMs:  10000,
	})
	if err != nil {
		print("Error initializing device: ", err, "\r\n")
	}
	print("Setting up device\r\n")
	device.OnReceivedPacket = func(packet tinygoRFM9X.Packet) {
		print("Received packet: %s", packet.Payload)
		if string(packet.Payload[0]) == "PING" {
			err = device.Send([]byte("PONG"))
			if err != nil {
				print("Error sending packet: ", err, "\r\n")
			}
			err = device.StartRecieve()
			if err != nil {
				print("Error restarting recieve: ", err, "\r\n")
			}
		} else if string(packet.Payload[0]) == "PONG" {
			err = device.Send([]byte("PING"))
			if err != nil {
				print("Error sending packet: ", err, "\r\n")
			}
			err = device.StartRecieve()
			if err != nil {
				print("Error restarting recieve: ", err, "\r\n")
			}
		}
	}
	print("Sending packet\r\n")
	err = device.Send([]byte("PING"))
	if err != nil {
		print("Error sending packet: ", err, "\r\n")
	}
	print("Starting recieve\r\n")
	err = device.StartRecieve()
	if err != nil {
		print("Error starting recieve: ", err, "\r\n")
	}
}
