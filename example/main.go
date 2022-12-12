package main

import (
	"machine"
	"time"

	"github.com/headblockhead/tinygorfm9x"
)

func main() {
	time.Sleep(1 * time.Second)
	device := tinygorfm9x.RFM9x{
		SPIDevice: *machine.SPI1,
	}
	print("Initializing device\r\n")
	err := device.Init(tinygorfm9x.Options{
		FrequencyMHz:      868,
		ResetPin:          machine.LORA_RESET,
		CSPin:             machine.LORA_CS,
		DIO0Pin:           machine.LORA_DIO0,
		DIO1Pin:           machine.LORA_DIO1,
		DIO2Pin:           machine.LORA_DIO2,
		EnableCRCChecking: true,
	})
	if err != nil {
		print("Error initializing device: ", err, "\r\n")
	}
	print("Setting up device\r\n")
	device.OnReceivedPacket = func(packet tinygorfm9x.Packet) {
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
	err = device.StartReceive()
	if err != nil {
		print("Error starting recieve: ", err, "\r\n")
	}
}
