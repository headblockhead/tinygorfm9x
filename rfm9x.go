package tinygorfm9x

import (
	"bytes"
	"errors"
	"machine"
	"math"
	"time"
)

// RFM9x is a driver for the RFM9x LoRa radio module.
type RFM9x struct {
	// Options is an instance of the Options type
	Options Options
	// SPIDevice is the SPI device used to communicate with the RFM9X
	SPIDevice machine.SPI
	// OnReceivedPacket is a function set by the user that is executed when a packet is received
	OnReceivedPacket func(Packet)
}

// Packet is a packet received from the radio.
type Packet struct {
	// Payload is the bytes received by the radio
	Payload []byte
	// RSSIDb is the Received Signal Strength Indication of the Packet
	RSSIDb byte
	// SNRDb is the signal-to-noise ratio of the Packet
	SNRDb byte
}

// Options are the options for the radio.
type Options struct {
	FrequencyMHz      int
	PreambleLength    uint16
	BandwidthHz       int
	CodingRate        byte
	SpreadingFactor   byte
	EnableCRCChecking bool
	TxPowerDb         int
	EnableAGC         bool
	ResetPin          machine.Pin
	CSPin             machine.Pin
	DIO0Pin           machine.Pin
	DIO1Pin           machine.Pin // Currently not used
	DIO2Pin           machine.Pin // Currently not used
	SPISpeedHz        uint32
	TxTimeoutMs       int
}

// defaultOptions are the default options for the radio.
// FrequencyMHz is set to 915 by default, as that is the frequency used in the US.
// The different Pin options are also set to NoPin, as they are set by the user in the Init() function.
var defaultOptions = Options{
	FrequencyMHz:      915,
	PreambleLength:    8,
	BandwidthHz:       500000,
	CodingRate:        5,
	SpreadingFactor:   7,
	EnableCRCChecking: false,
	TxPowerDb:         23,
	EnableAGC:         false,
	ResetPin:          machine.NoPin,
	CSPin:             machine.NoPin,
	DIO0Pin:           machine.NoPin,
	DIO1Pin:           machine.NoPin,
	DIO2Pin:           machine.NoPin,
	SPISpeedHz:        100000,
	TxTimeoutMs:       2000,
}

const (
	REGISTERS_FIFO              = 0x00
	REGISTERS_OP_MODE           = 0x01
	REGISTERS_FRF_MSB           = 0x06
	REGISTERS_FRF_MID           = 0x07
	REGISTERS_FRF_LSB           = 0x08
	REGISTERS_PA_CONFIG         = 0x09
	REGISTERS_FIFO_ADDR_PTR     = 0x0D
	REGISTERS_FIFO_TX_BASE_ADDR = 0x0E
	REGISTERS_FIFO_RX_BASE_ADDR = 0x0F
	REGISTERS_IRQ_FLAGS         = 0x12
	REGISTERS_RX_NB_BYTES       = 0x13
	REGISTERS_PKT_SNR_VALUE     = 0x19
	REGISTERS_PKT_RSSI_VALUE    = 0x1A
	REGISTERS_MODEM_CONFIG_1    = 0x1D
	REGISTERS_MODEM_CONFIG_2    = 0x1E
	REGISTERS_PREAMBLE_MSB      = 0x20
	REGISTERS_PREAMBLE_LSB      = 0x21
	REGISTERS_PAYLOAD_LENGTH    = 0x22
	REGISTERS_MODEM_CONFIG_3    = 0x26
	REGISTERS_DIO_MAPPING_1     = 0x40
	REGISTERS_DIO_MAPPING_2     = 0x41
	REGISTERS_VERSION           = 0x42
	REGISTERS_PA_DAC            = 0x4D
	OP_MODES_SLEEP              = 0b000
	OP_MODES_STANDBY            = 0b001
	OP_MODES_TRANSMIT           = 0b011
	OP_MODES_RXCONT             = 0b101
	OP_MODES_RXSINGLE           = 0b110
	OP_MODES_CAD                = 0b111
	DIO0_MAPPINGS_RX_DONE       = 0b00
	DIO0_MAPPINGS_TX_DONE       = 0b01
	DIO0_MAPPINGS_CAD_DONE      = 0b10
	RF95_FXOSC                  = 32000000
	RF95_FSTEP                  = RF95_FXOSC / 524288
)

var (
	BANDWIDTHS        = []int{7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000}
	BW_REG_2F_OFFSETS = []byte{0x48, 0x44, 0x44, 0x44, 0x44, 0x44, 0x40, 0x40, 0x40}
	BITMASKS          = []byte{0b00000001, 0b00000011, 0b00000111, 0b00001111, 0b00011111, 0b00111111, 0b01111111}
)

// SetDIO0Interrupt sets the function to be run when the DIO0 pin is changed.
func (rfm *RFM9x) SetDIO0Interrupt(interrupt func(machine.Pin)) (err error) {
	return rfm.Options.DIO0Pin.SetInterrupt(machine.PinRising, interrupt)
}

// ClearDIO0Interrupt deletes the function set to be run when the DIO0 pin is changed.
func (rfm *RFM9x) ClearDIO0Interrupt() (err error) {
	return rfm.Options.DIO0Pin.SetInterrupt(machine.PinRising, nil)
}

// Init sets up the RFM9x with the options provided, and uses the defaultOptions for fields not specified by the options provided.
func (rfm *RFM9x) Init(opts Options) (err error) {
	rfm.Options = defaultOptions
	rfm.Options.ResetPin = opts.ResetPin
	rfm.Options.CSPin = opts.CSPin
	rfm.Options.DIO0Pin = opts.DIO0Pin
	rfm.Options.DIO1Pin = opts.DIO1Pin
	rfm.Options.DIO2Pin = opts.DIO2Pin
	if opts.BandwidthHz != 0 {
		rfm.Options.BandwidthHz = opts.BandwidthHz
	}
	if opts.CodingRate != 0 {
		rfm.Options.CodingRate = opts.CodingRate
	}
	if opts.SpreadingFactor != 0 {
		rfm.Options.SpreadingFactor = opts.SpreadingFactor
	}
	if opts.EnableCRCChecking != false {
		rfm.Options.EnableCRCChecking = opts.EnableCRCChecking
	}
	if opts.TxPowerDb != 0 {
		rfm.Options.TxPowerDb = opts.TxPowerDb
	}
	if opts.EnableAGC != false {
		rfm.Options.EnableAGC = opts.EnableAGC
	}
	if opts.SPISpeedHz != 0 {
		rfm.Options.SPISpeedHz = opts.SPISpeedHz
	}
	if opts.TxTimeoutMs != 0 {
		rfm.Options.TxTimeoutMs = opts.TxTimeoutMs
	}
	err = machine.SPI1.Configure(machine.SPIConfig{
		Frequency: rfm.Options.SPISpeedHz,
	})
	if err != nil {
		return err
	}
	rfm.Options.ResetPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	rfm.Options.ResetPin.High()
	rfm.Options.CSPin.Configure(machine.PinConfig{Mode: machine.PinOutput})
	rfm.Options.DIO0Pin.Configure(machine.PinConfig{Mode: machine.PinInput})
	err = rfm.Reset()
	if err != nil {
		return err
	}
	version, err := rfm.GetVersion()
	if err != nil {
		return err
	}
	if version == 0 {
		return errors.New("RFM9x module not detected")
	} else if version != 0x12 {
		return errors.New("RFM9x version not supported")
	}
	// Switch to sleep mode and set LoRa mode (can only be done in sleep mode)
	err = rfm.SetOperatingMode(OP_MODES_SLEEP)
	if err != nil {
		return err
	}
	err = rfm.SetLoRaMode(true)
	if err != nil {
		return err
	}
	// Perform a sanity check
	currentOperatingMode, err := rfm.GetOperatingMode()
	if err != nil {
		return err
	}
	currentLoRaMode, err := rfm.GetLoRaMode()
	if err != nil {
		return err
	}
	if currentOperatingMode != OP_MODES_SLEEP {
		return errors.New("communication error: op Readback of module configuration failed")
	}
	if !(currentLoRaMode) {
		return errors.New("communication error: lor Readback of module configuration failed")
	}
	// Clear low frequency mode if frequency is high
	if rfm.Options.FrequencyMHz > 525 {
		err = rfm.SetLowFrequencyMode(false)
		if err != nil {
			return err
		}
	}
	// Setup entire 256 byte FIFO
	err = rfm.SetFIFOBaseAddress(0, 0)
	if err != nil {
		return err
	}
	// Switch back to standby mode and set parameters
	err = rfm.SetOperatingMode(OP_MODES_STANDBY)
	if err != nil {
		return err
	}
	err = rfm.SetPreambleLength(rfm.Options.PreambleLength)
	if err != nil {
		return err
	}
	err = rfm.SetFrequencyAndBandwidth(rfm.Options.FrequencyMHz, rfm.Options.BandwidthHz)
	if err != nil {
		return err
	}
	err = rfm.SetSpreadingFactor(rfm.Options.SpreadingFactor)
	if err != nil {
		return err
	}
	err = rfm.SetCodingRate(rfm.Options.CodingRate)
	if err != nil {
		return err
	}
	err = rfm.SetRxCRC(rfm.Options.EnableCRCChecking)
	if err != nil {
		return err
	}
	err = rfm.SetAGC(rfm.Options.EnableAGC)
	if err != nil {
		return err
	}
	err = rfm.SetTxPower(rfm.Options.TxPowerDb)
	if err != nil {
		return err
	}
	return nil
}

// Reset resets the RFM9x by toggling the ResetPin.
func (rfm *RFM9x) Reset() (err error) {
	rfm.Options.ResetPin.Low()
	time.Sleep(100 * time.Microsecond)
	rfm.Options.ResetPin.High()
	time.Sleep(10 * time.Millisecond)
	return nil
}

// StartReceive tells the RFM9X to start waiting for a packet. When it receives a packet, it will run OnReceivedPacket().
func (rfm *RFM9x) StartReceive() (err error) {
	err = rfm.StopReceive()
	if err != nil {
		return err
	}
	err = rfm.WriteByteToAddress(REGISTERS_FIFO_ADDR_PTR, 0)
	if err != nil {
		return err
	}
	err = rfm.WriteBits(REGISTERS_DIO_MAPPING_1, 2, 6, DIO0_MAPPINGS_RX_DONE)
	if err != nil {
		return err
	}
	err = rfm.SetDIO0Interrupt(func(dio0 machine.Pin) {
		if dio0.Get() == true {
			flags, err := rfm.ReadBits(REGISTERS_IRQ_FLAGS, 3, 4)
			if err != nil {
				return
			}
			err = rfm.WriteByteToAddress(REGISTERS_IRQ_FLAGS, 0xFF)
			if err != nil {
				return
			}
			if flags != 0b0101 {
				return
			}
			numBytes, err := rfm.ReadByteFromAddress(REGISTERS_RX_NB_BYTES)
			if err != nil {
				return
			}
			err = rfm.WriteByteToAddress(REGISTERS_FIFO_ADDR_PTR, 0)
			if err != nil {
				return
			}
			rxbuf, err := rfm.ReadBuffer(REGISTERS_FIFO, numBytes)
			if err != nil {
				return
			}
			snr, err := rfm.ReadByteFromAddress(REGISTERS_PKT_SNR_VALUE)
			if err != nil {
				return
			}
			rssi, err := rfm.ReadByteFromAddress(REGISTERS_PKT_RSSI_VALUE)
			if err != nil {
				return
			}
			rfm.OnReceivedPacket(Packet{Payload: rxbuf, SNRDb: snr, RSSIDb: rssi})
		}
	})
	if err != nil {
		return err
	}
	return rfm.SetOperatingMode(OP_MODES_RXCONT)
}

// StopReceive puts	the device into standby mode and stops waiting for packets.
func (rfm *RFM9x) StopReceive() (err error) {
	err = rfm.SetOperatingMode(OP_MODES_STANDBY)
	if err != nil {
		return err
	}
	return rfm.ClearDIO0Interrupt()
}

// Send a payload using the radio.
func (rfm *RFM9x) Send(payload []byte) (err error) {
	if len(payload) < 1 {
		return errors.New("empty payload supplied")
	}
	if len(payload) > 255 {
		return errors.New("payload too long")
	}
	err = rfm.StopReceive()
	if err != nil {
		return err
	}
	err = rfm.WriteByteToAddress(REGISTERS_FIFO_ADDR_PTR, 0)
	if err != nil {
		return err
	}
	err = rfm.WriteBuffer(REGISTERS_FIFO, payload)
	if err != nil {
		return err
	}
	err = rfm.WriteByteToAddress(REGISTERS_PAYLOAD_LENGTH, byte(len(payload)))
	if err != nil {
		return err
	}
	err = rfm.WriteBits(REGISTERS_DIO_MAPPING_1, 2, 6, DIO0_MAPPINGS_TX_DONE)
	if err != nil {
		return err
	}
	timeout := make(chan error, 1)
	success := make(chan bool, 1)
	defer close(timeout)
	defer close(success)
	go func() {
		time.Sleep(time.Duration(rfm.Options.TxTimeoutMs) * time.Millisecond)
		timeout <- errors.New("timeout")
	}()
	err = rfm.SetDIO0Interrupt(func(p machine.Pin) {
		if p.Get() == true {
			err = rfm.WriteByteToAddress(REGISTERS_IRQ_FLAGS, 0xFF)
			if err != nil {
				return
			}
			err = rfm.ClearDIO0Interrupt()
			if err != nil {
				return
			}
		}
		success <- true
	})
	if err != nil {
		return err
	}
	err = rfm.SetOperatingMode(OP_MODES_TRANSMIT)
	select {
	case <-timeout:
		return errors.New("timeout")
	case <-success:
		break
	}
	return err
}

// GetVersion returns the version of the device. It should be 0x12.
func (rfm *RFM9x) GetVersion() (version byte, err error) {
	return rfm.ReadByteFromAddress(REGISTERS_VERSION)
}

// GetOperatingMode returns the current operating mode of the device.
func (rfm *RFM9x) GetOperatingMode() (mode byte, err error) {
	return rfm.ReadBits(REGISTERS_OP_MODE, 3, 0)
}

// SetOperatingMode sets the current operating mode of the device.
func (rfm *RFM9x) SetOperatingMode(mode byte) (err error) {
	return rfm.WriteBits(REGISTERS_OP_MODE, 3, 0, mode)
}

// GetLoRaMode returns the current LoRa mode of the device.
func (rfm *RFM9x) GetLoRaMode() (isLoRa bool, err error) {
	value, err := rfm.ReadBits(REGISTERS_OP_MODE, 1, 7)
	return value == 1, err
}

// SetLoRaMode sets the current LoRa mode of the device.
func (rfm *RFM9x) SetLoRaMode(isLoRa bool) (err error) {
	return rfm.WriteBits(REGISTERS_OP_MODE, 1, 7, boolToByte(isLoRa))
}

// SetLowFrequencyMode sets the low frequency mode of the device.
func (rfm *RFM9x) SetLowFrequencyMode(isLowFrequency bool) (err error) {
	return rfm.WriteBits(REGISTERS_OP_MODE, 1, 3, boolToByte(isLowFrequency))
}

func boolToByte(b bool) byte {
	if b {
		return 1
	}
	return 0
}

// SetFIFOBaseAddress sets the TX and RX base address of the device.
func (rfm *RFM9x) SetFIFOBaseAddress(txBaseAddress byte, rxBaseAddress byte) (err error) {
	err = rfm.WriteByteToAddress(REGISTERS_FIFO_TX_BASE_ADDR, txBaseAddress)
	if err != nil {
		return err
	}
	return rfm.WriteByteToAddress(REGISTERS_FIFO_RX_BASE_ADDR, rxBaseAddress)
}

// SetFrequencyAndBandwidth sets the frequency (in MHz) and the bandwidth (in Hz) of the device.
func (rfm *RFM9x) SetFrequencyAndBandwidth(frequencyMHz int, bandwidthHz int) (err error) {
	// Offset frequency value to prevent spurious reception
	// (Semtech SX1276 errata note 2.3)
	frequencyHz := float64(frequencyMHz * 1000000)
	if bandwidthHz < 62500 {
		frequencyHz += float64(bandwidthHz)
	}
	frf := int(math.Round(frequencyHz/RF95_FSTEP)) & 0xFFFFFF
	err = rfm.WriteByteToAddress(REGISTERS_FRF_MSB, byte(frf>>16))
	if err != nil {
		return err
	}
	err = rfm.WriteByteToAddress(REGISTERS_FRF_MID, byte((frf>>8)&0xFF))
	if err != nil {
		return err
	}
	err = rfm.WriteByteToAddress(REGISTERS_FRF_LSB, byte(frf&0xFF))
	if err != nil {
		return err
	}
	// Find the lowest bandwidth setting that is greater than or equal to the desired bandwidth.
	// bandwidthID will be set to the length of the array if none is found.
	var bandwidthID int
	for bandwidthID = 0; bandwidthID < len(BANDWIDTHS); bandwidthID++ {
		if bandwidthHz <= BANDWIDTHS[bandwidthID] {
			break
		}
	}
	err = rfm.WriteBits(REGISTERS_MODEM_CONFIG_1, 4, 4, byte(bandwidthID))
	if err != nil {
		return err
	}
	// Receiver Spurious Reception of LoRa Signal
	// (Semtech SX1276 errata note 2.3)
	if bandwidthID < len(BANDWIDTHS) {
		err = rfm.WriteBits(0x31, 1, 7, 0)
		if err != nil {
			return err
		}
		err = rfm.WriteByteToAddress(0x2F, BW_REG_2F_OFFSETS[bandwidthID])
		if err != nil {
			return err
		}
		err = rfm.WriteByteToAddress(0x30, 0)
		if err != nil {
			return err
		}
	} else {
		err = rfm.WriteBits(0x31, 1, 7, 1)
		if err != nil {
			return err
		}
	}

	// Sensitivity Optimization with 500 kHz Bandwidth
	// (Semtech SX1276 errata note 2.1)
	if bandwidthID == len(BANDWIDTHS) {
		if frequencyMHz >= 862 {
			err = rfm.WriteByteToAddress(0x36, 0x02)
			if err != nil {
				return err
			}
			err = rfm.WriteByteToAddress(0x3A, 0x64)
			if err != nil {
				return err
			}
		} else if frequencyMHz <= 525 {
			err = rfm.WriteByteToAddress(0x36, 0x02)
			if err != nil {
				return err
			}
			err = rfm.WriteByteToAddress(0x3A, 0x7F)
			if err != nil {
				return err
			}
		}
	} else {
		err = rfm.WriteByteToAddress(0x36, 0x03)
		if err != nil {
			return err
		}
	}
	return nil
}

// SetPreambleLength sets the preamble length of the device.
func (rfm *RFM9x) SetPreambleLength(preambleLength uint16) (err error) {
	err = rfm.WriteByteToAddress(REGISTERS_PREAMBLE_MSB, byte((preambleLength>>8)&0xFF))
	if err != nil {
		return err
	}
	return rfm.WriteByteToAddress(REGISTERS_PREAMBLE_LSB, byte(preambleLength&0xFF))
}

// SetSpreadingFactor sets the spreading factor of the device.
func (rfm *RFM9x) SetSpreadingFactor(sf byte) (err error) {
	if sf < 6 || sf > 12 {
		return errors.New("invalid spreading factor")
	}
	err = rfm.WriteBits(REGISTERS_MODEM_CONFIG_2, 4, 4, sf)
	if err != nil {
		return err
	}
	if sf == 6 {
		err = rfm.WriteBits(0x31, 3, 0, 0b101)
		if err != nil {
			return err
		}
		err = rfm.WriteByteToAddress(0x37, 0x0C)
		if err != nil {
			return err
		}
	}
	return nil
}

// SetCodingRate sets the coding rate of the device.
func (rfm *RFM9x) SetCodingRate(codingRate byte) (err error) {
	if codingRate < 5 || codingRate > 8 {
		return errors.New("invalid coding rate")
	}
	return rfm.WriteBits(REGISTERS_MODEM_CONFIG_1, 3, 1, codingRate-4)
}

// SetRxCRC enables or disables the CRC checking functionality of the device.
func (rfm *RFM9x) SetRxCRC(enableCRC bool) (err error) {
	return rfm.WriteBits(REGISTERS_MODEM_CONFIG_2, 1, 2, boolToByte(enableCRC))
}

// SetAGC enables or disables the Automatic Gain Control of the device.
func (rfm *RFM9x) SetAGC(enableAGC bool) (err error) {
	return rfm.WriteBits(REGISTERS_MODEM_CONFIG_3, 1, 2, boolToByte(enableAGC))
}

// SetTxPower sets the TX power of the device in db.
func (rfm *RFM9x) SetTxPower(txPowerDb int) (err error) {
	// Currently only high power mode (PA_BOOST) is supported
	if txPowerDb < 5 || txPowerDb > 23 {
		return errors.New("invalid TX power")
	}
	if txPowerDb > 20 {
		err = rfm.WriteByteToAddress(REGISTERS_PA_DAC, 0x87)
		if err != nil {
			return err
		}
		txPowerDb -= 3
	} else {
		err = rfm.WriteByteToAddress(REGISTERS_PA_DAC, 0x84)
		if err != nil {
			return err
		}
	}
	err = rfm.WriteBits(REGISTERS_PA_CONFIG, 1, 7, 1)
	if err != nil {
		return err
	}
	return rfm.WriteBits(REGISTERS_PA_CONFIG, 4, 0, byte(txPowerDb-5))
}

// ReadByteFromAddress reads a single byte from the device at the address provided.
func (rfm *RFM9x) ReadByteFromAddress(address byte) (value byte, err error) {
	rxbuf, err := rfm.ReadBuffer(address, 1)
	if err != nil {
		return 0, err
	}
	return rxbuf[0], nil
}

// ReadBuffer uses SPI to read multiple bytes from the device using the address and length provided.
func (rfm *RFM9x) ReadBuffer(address byte, length byte) (value []byte, err error) {
	txbuf := make([]byte, length+1)
	txbuf[0] = address & 0x7F
	rxbuf := make([]byte, len(txbuf))
	rfm.Options.CSPin.Low()
	err = rfm.SPIDevice.Tx(txbuf, rxbuf)
	if err != nil {
		return nil, err
	}
	rfm.Options.CSPin.High()
	rxbuf = rxbuf[1:]
	return rxbuf, nil
}

// WriteByteToAddress writes a single byte to the device at the address provided.
func (rfm *RFM9x) WriteByteToAddress(address byte, val byte) (err error) {
	return rfm.WriteBuffer(address, []byte{val & 0xFF})
}

// WriteBuffer uses SPI to write multiple bytes to the device using the address and buffer provided.
func (rfm *RFM9x) WriteBuffer(address byte, buffer []byte) (err error) {
	txbuf := bytes.Join([][]byte{
		{byte((address & 0x7F) | 0x80)},
		buffer,
	}, nil)
	rfm.Options.CSPin.Low()
	err = rfm.SPIDevice.Tx(txbuf, nil)
	if err != nil {
		return err
	}
	rfm.Options.CSPin.High()
	return nil
}

// ReadBits reads a bit from a byte address using an offset.
func (rfm *RFM9x) ReadBits(address, bits, offset byte) (value byte, err error) {
	mask := BITMASKS[bits-1] << offset
	registerValue, err := rfm.ReadByteFromAddress(address)
	return (registerValue & mask) >> offset, err
}

// WriteBits writes a bit in a byte using an offset.
func (rfm *RFM9x) WriteBits(address, bits, offset, val byte) (err error) {
	mask := BITMASKS[bits-1]
	val &= mask
	oldRegisterValue, err := rfm.ReadByteFromAddress(address)
	if err != nil {
		return err
	}
	registerValue := oldRegisterValue
	registerValue &= ^(mask << offset)
	registerValue |= val << offset
	return rfm.WriteByteToAddress(address, registerValue)
}
