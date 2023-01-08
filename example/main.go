package main

import (
	"encoding/hex"
	"fmt"
	"image"
	"image/color"
	"machine"
	"time"

	"github.com/headblockhead/tinygorfm9x"
	"golang.org/x/image/font"
	"golang.org/x/image/font/basicfont"
	"golang.org/x/image/math/fixed"
	"tinygo.org/x/drivers/ssd1306"
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
		SpreadingFactor:   12,
		EnableCRCChecking: false,
	})
	if err != nil {
		print("Error initializing device: ", err, "\r\n")
	}
	machine.I2C0.Configure(machine.I2CConfig{
		Frequency: machine.TWI_FREQ_400KHZ,
		SDA:       machine.GPIO0,
		SCL:       machine.GPIO1,
	})
	display := ssd1306.NewI2C(machine.I2C0)
	display.Configure(ssd1306.Config{
		Address: 0x3C,
		Width:   128,
		Height:  64,
	})
	display.ClearDisplay()
	print("Setting up device\r\n")
	device.OnReceivedPacket = func(packet tinygorfm9x.Packet) {
		img := image.NewRGBA(image.Rect(0, 0, 128, 64))
		print("Received packet: ", string(packet.Payload), "\r\n")
		print("RSSI: ", packet.RSSIDb, "\r\n")
		print("SNR: ", packet.SNRDb, "\r\n")
		print("Packet bytes: \r\n")
		for _, b := range packet.Payload {
			print(b, " ")
		}
		print("\r\n")
		drawText(img, 0, 13, "RSSI: "+fmt.Sprint(packet.RSSIDb))
		drawText(img, 0, 26, "SNR: "+fmt.Sprint(packet.SNRDb))
		drawText(img, 0, 39, "Bytes: ")
		drawText(img, 0, 52, hex.EncodeToString(packet.Payload))
		drawText(img, 0, 65, string(packet.Payload))
		displayImage(&display, img)
	}
	print("Sending packet\r\n")

	for {
		err = device.Send([]byte("Hello World!"))
		if err != nil {
			print("Error sending packet: ", err, "\r\n")
		}
		print("Starting recieve\r\n")
		err = device.StartReceive()
		if err != nil {
			print("Error starting recieve: ", err, "\r\n")
		}
		time.Sleep(3 * time.Second)
		display.ClearDisplay()
	}
}

// drawText will write text in a 7x13 pixel font at a location.
func drawText(img *image.RGBA, x, y int, text string) {
	col := color.RGBA{255, 255, 255, 255}
	point := fixed.Point26_6{X: fixed.I(x), Y: fixed.I(y)}

	d := &font.Drawer{
		Dst:  img,
		Src:  image.NewUniform(col),
		Face: basicfont.Face7x13,
		Dot:  point,
	}
	d.DrawString(text)
}

// displayImage takes in an image and writes it to the screen.
func displayImage(display *ssd1306.Device, img image.Image) (err error) {
	// Put the image into the buffer.
	for y := 0; y < img.Bounds().Dy(); y++ {
		for x := 0; x < img.Bounds().Dx(); x++ {
			r, g, b, a := img.At(x, y).RGBA()
			display.SetPixel(int16(x), int16(y), color.RGBA{uint8(r), uint8(g), uint8(b), uint8(a)})
		}
	}
	// Show the buffer.
	err = display.Display()
	if err != nil {
		return err
	}
	return nil
}
