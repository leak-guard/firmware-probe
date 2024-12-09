# Project setup
#### in the main directory run these commands:  
`git submodule init`  
`git submodule update --init --recursive`  
`git submodule foreach --recursive git fetch`

# Used library
https://github.com/wdomski/SX1278/

# Useful links
https://blog.domski.pl/stm32-hal-driver-for-lora-sx1278-wireless-module/
https://www.youtube.com/watch?v=jMGnSQyCUqw

# STM32 Setup
## 1. System Clock
### `HSI 2 MHz`

## 2. SPI
### Mode: Full-Duplex Master
- Hardware NSS Signal: Disabled
- Motorola
- 8 Bits
- MSB First
- Prescaler: 2
- Baud Rate: 1000.0 KBits/s
- Clock Polarity (CPOL): Low
- Clock Phase (CPHA): 1 Edge
- CRC Calculation: Disabled
- NSS Signal Type: Software

### SCK:
- GPIO mode: Alternate Function Push Pull
- GPIO Pull-up/Pull-down: No pull-up and no pull-down
- Maximum output speed: High

### MISO:
- GPIO mode: Alternate Function Push Pull
- GPIO Pull-up/Pull-down: Pull-up
- Maximum output speed: High

### MOSI:
- GPIO mode: Alternate Function Push Pull
- GPIO Pull-up/Pull-down: No pull-up and no pull-down
- Maximum output speed: High

## 3. GPIO

### NSS
- GPIO_Output
- GPIO output level: High
- GPIO mode: Output Push Pull
- GPIO Pull-up/Pull-down: Pull-up
- Maximum output speed: Low
- User Label: NSS

### DIO0
- GPIO_EXTI
- GPIO mode: External Interrupt Mode with Rising edge trigger detection (enable EXTI Interrupt in NVIC!)
- GPIO Pull-up/Pull-down: No pull-up and no pull-down
- User Label: DIO0

### RESET
- GPIO_Output
- GPIO output level: High
- GPIO mode: Output Push Pull
- GPIO Pull-up/Pull-down: No pull-up and no pull-down
- Maximum output speed: Low
- Fast Mode: Disable
- User Label: RESET

## Exemplary connection (STM32G431KB NUCLEO)

The module has to be connected with the MCU board 
with following pins:

| MCU pin | SX1278 pin | Description                                      |
|---------|------------| -------------------------------------------------|
| PB4     | NSS        | SPI chip--select                                 |
| PA5     | SCK        | SPI SCK                                          |
| PA6     | MISO       | SPI MISO                                         |
| PA7     | MOSI       | SPI MOSI                                         |
| PB5     | DIO0       | LoRa interrupt indicator pin (for receiver mode) |
| PB6     | REST       | LoRa reset                                       |
| ------- | ---------- | -------------------------------------------------|
| VDD     | VCC        | +3V3                                             |
| VSS     | GND        | GND                                              |

## 4. Default LoRa settings

### Frequency
`default`: `433 Mhz`
supported values: `433E6`, `866E6`, `915E6`

### Spreading Factor
`default`: `7`
Supported values are between `6` and `12`. If a spreading factor of `6` is set, implicit header mode must be used to transmit and receive packets.

### Bandwidth
`default`: `125 KHz`
Supported values: `7.8E3`, `10.4E3`, `15.6E3`, `20.8E3`, `31.25E3`, `41.7E3`, `62.5E3`, `125E3`, `250E3`, and `500E3`.

### Coding Rate
`default`: `4/5`
Supported values are between `5` and `8`, these correspond to coding rates of `4/5` and `4/8`. The coding rate numerator is fixed at `4`.

### Preamble Length
`default`: `8`
Supported values are between `6` and `65535`.

### Sync Word
* default LoRa sync word: `0x12`
* sync word reserved for LoRaWAN networks: `0x34`

### CRC
By default it's `disabled`.

### Invert IQ Signals
Enable or disable Invert the LoRa I and Q signals, by default a invertIQ is `not used`.

### LNA Gain
Set LNA Gain for better RX sensitivity, by default AGC (Automatic Gain Control) is used and LNA gain is not used.
Supported values are between `0` and `6`. If gain is 0, AGC will be enabled and LNA gain will not be used. Else if gain is from 1 to 6, AGC will be disabled and LNA gain will be used.

## TEST Button simulates ALARM
Sends: IDs, battery voltage, `ALARM` message type and CRC

## ALARM Pin
Sends: IDs, battery voltage, `ALARM` message type and CRC

## PING Button
Sends: IDs, battery voltage, `PING` message type and CRC

## Every 12 hours
Sends: IDs, battery voltage, `BATTERY` message type and CRC