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
		FrequencyMhz:      868,
		ResetPin:          machine.LORA_RESET,
		CSPin:             machine.LORA_CS,
		Dio0Pin:           machine.LORA_DIO0,
		Dio1Pin:           machine.LORA_DIO1,
		Dio2Pin:           machine.LORA_DIO2,
		EnableCrcChecking: true,
	})
	if err != nil {
		print("Error initializing device: ", err, "\r\n")
	}
	print("Setting up device\r\n")
	device.OnReceivedPacket = func(packet tinygoRFM9X.Packet) {
		print("Received packet: ", string(packet.Payload), "\r\n")
		print("RSSI: ", packet.RSSIDb, "\r\n")
		print("SNR: ", packet.SNRDb, "\r\n")
		print("Packet bytes: \r\n")
		for _, b := range packet.Payload {
			print(b, " ")
		}
		print("\r\n")
	}
	print("Sending packet\r\n")
	err = device.Send([]byte("Hello World!"))
	if err != nil {
		print("Error sending packet: ", err, "\r\n")
	}
	print("Starting recieve\r\n")
	err = device.StartRecieve()
	if err != nil {
		print("Error starting recieve: ", err, "\r\n")
	}
}
