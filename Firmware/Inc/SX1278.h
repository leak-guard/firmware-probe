#ifndef __SX1278_H__
#define __SX1278_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <gpio.h>
#include <stdbool.h>
#include <stdint.h>

#define SX1278_MAX_PACKET 256
#define SX1278_DEFAULT_TIMEOUT 3000

// RFM98 Internal registers Address
/********************LoRa mode***************************/
#define LR_RegFifo 0x00
// Common settings
#define LR_RegOpMode 0x01
#define LR_RegFrMsb 0x06
#define LR_RegFrMid 0x07
#define LR_RegFrLsb 0x08
// Tx settings
#define LR_RegPaConfig 0x09
#define LR_RegPaRamp 0x0A
#define LR_RegOcp 0x0B
// Rx settings
#define LR_RegLna 0x0C
// LoRa registers
#define LR_RegFifoAddrPtr 0x0D
#define LR_RegFifoTxBaseAddr 0x0E
#define LR_RegFifoRxBaseAddr 0x0F
#define LR_RegFifoRxCurrentaddr 0x10
#define LR_RegIrqFlagsMask 0x11
#define LR_RegIrqFlags 0x12
#define LR_RegRxNbBytes 0x13
#define LR_RegRxHeaderCntValueMsb 0x14
#define LR_RegRxHeaderCntValueLsb 0x15
#define LR_RegRxPacketCntValueMsb 0x16
#define LR_RegRxPacketCntValueLsb 0x17
#define LR_RegModemStat 0x18
#define LR_RegPktSnrValue 0x19
#define LR_RegPktRssiValue 0x1A
#define LR_RegRssiValue 0x1B
#define LR_RegHopChannel 0x1C
#define LR_RegModemConfig1 0x1D
#define LR_RegModemConfig2 0x1E
#define LR_RegSymbTimeoutLsb 0x1F
#define LR_RegPreambleMsb 0x20
#define LR_RegPreambleLsb 0x21
#define LR_RegPayloadLength 0x22
#define LR_RegMaxPayloadLength 0x23
#define LR_RegHopPeriod 0x24
#define LR_RegFifoRxByteAddr 0x25
#define LR_RegModemConfig3 0x26
// I/O settings
#define REG_LR_DIOMAPPING1 0x40
#define REG_LR_DIOMAPPING2 0x41
// Version
#define REG_LR_VERSION 0x42
// Additional settings
#define REG_LR_PLLHOP 0x44
#define REG_LR_TCXO 0x4B
#define REG_LR_PADAC 0x4D
#define REG_LR_FORMERTEMP 0x5B
#define REG_LR_AGCREF 0x61
#define REG_LR_AGCTHRESH1 0x62
#define REG_LR_AGCTHRESH2 0x63
#define REG_LR_AGCTHRESH3 0x64

/********************FSK/ook mode***************************/
#define RegFIFO 0x00
#define RegOpMode 0x01
#define RegBitRateMsb 0x02
#define RegBitRateLsb 0x03
#define RegFdevMsb 0x04
#define RegFdevLsb 0x05
#define RegFreqMsb 0x06
#define RegFreqMid 0x07
#define RegFreqLsb 0x08
#define RegPaConfig 0x09
#define RegPaRamp 0x0a
#define RegOcp 0x0b
#define RegLna 0x0c
#define RegRxConfig 0x0d
#define RegRssiConfig 0x0e
#define RegRssiCollision 0x0f
#define RegRssiThresh 0x10
#define RegRssiValue 0x11
#define RegRxBw 0x12
#define RegAfcBw 0x13
#define RegOokPeak 0x14
#define RegOokFix 0x15
#define RegOokAvg 0x16
#define RegAfcFei 0x1a
#define RegAfcMsb 0x1b
#define RegAfcLsb 0x1c
#define RegFeiMsb 0x1d
#define RegFeiLsb 0x1e
#define RegPreambleDetect 0x1f
#define RegRxTimeout1 0x20
#define RegRxTimeout2 0x21
#define RegRxTimeout3 0x22
#define RegRxDelay 0x23
#define RegOsc 0x24
#define RegPreambleMsb 0x25
#define RegPreambleLsb 0x26
#define RegSyncConfig 0x27
#define RegSyncValue1 0x28
#define RegSyncValue2 0x29
#define RegSyncValue3 0x2a
#define RegSyncValue4 0x2b
#define RegSyncValue5 0x2c
#define RegSyncValue6 0x2d
#define RegSyncValue7 0x2e
#define RegSyncValue8 0x2f
#define RegPacketConfig1 0x30
#define RegPacketConfig2 0x31
#define RegPayloadLength 0x32
#define RegNodeAdrs 0x33
#define RegBroadcastAdrs 0x34
#define RegFifoThresh 0x35
#define RegSeqConfig1 0x36
#define RegSeqConfig2 0x37
#define RegTimerResol 0x38
#define RegTimer1Coef 0x39
#define RegSyncWord 0x39
#define RegTimer2Coef 0x3a
#define RegImageCal 0x3b
#define RegTemp 0x3c
#define RegLowBat 0x3d
#define RegIrqFlags1 0x3e
#define RegIrqFlags2 0x3f
#define RegDioMapping1 0x40
#define RegDioMapping2 0x41
#define RegVersion 0x42
#define RegPllHop 0x44
#define RegPaDac 0x4d
#define RegBitRateFrac 0x5d

/**********************************************************
 **Parameter table define
 **********************************************************/

#define SX1278_FREQ_433MHz 0
#define SX1278_FREQ_866MHz 1
#define SX1278_FREQ_915MHz 2

static const uint64_t SX1278_Frequency[3] = { 433E6, 866E6, 915E6};

#define SX1278_POWER_20DBM 0
#define SX1278_POWER_17DBM 1
#define SX1278_POWER_14DBM 2
#define SX1278_POWER_11DBM 3

static const uint8_t SX1278_Power[4] =
{
    0xFF, // 20dbm
    0xFC, // 17dbm
    0xF9, // 14dbm
    0xF6, // 11dbm
};

#define SX1278_LORA_SF_6 0
#define SX1278_LORA_SF_7 1
#define SX1278_LORA_SF_8 2
#define SX1278_LORA_SF_9 3
#define SX1278_LORA_SF_10 4
#define SX1278_LORA_SF_11 5
#define SX1278_LORA_SF_12 6

static const uint8_t SX1278_SpreadFactor[7] = {6, 7, 8, 9, 10, 11, 12};

#define SX1278_LORA_BW_7_8KHZ 0
#define SX1278_LORA_BW_10_4KHZ 1
#define SX1278_LORA_BW_15_6KHZ 2
#define SX1278_LORA_BW_20_8KHZ 3
#define SX1278_LORA_BW_31_2KHZ 4
#define SX1278_LORA_BW_41_7KHZ 5
#define SX1278_LORA_BW_62_5KHZ 6
#define SX1278_LORA_BW_125KHZ 7
#define SX1278_LORA_BW_250KHZ 8
#define SX1278_LORA_BW_500KHZ 9

static const uint8_t SX1278_LoRaBandwidth[10] =
{
    0,    //   7.8KHz,
    1,    //  10.4KHz,
    2,    //  15.6KHz,
    3,    //  20.8KHz,
    4,    //  31.2KHz,
    5,    //  41.7KHz,
    6,    //  62.5KHz,
    7,    // 125.0KHz,
    8,    // 250.0KHz,
    9     // 500.0KHz
};

// Coding rate
#define SX1278_LORA_CR_4_5 0
#define SX1278_LORA_CR_4_6 1
#define SX1278_LORA_CR_4_7 2
#define SX1278_LORA_CR_4_8 3

static const uint8_t SX1278_CodingRate[4] = {0x01, 0x02, 0x03, 0x04};

// CRC Enable
#define SX1278_LORA_CRC_EN 0
#define SX1278_LORA_CRC_DIS 1

static const uint8_t SX1278_CRC_Sum[2] = {0x01, 0x00};

typedef enum
{
  SLEEP,
  STANDBY,
  TX,
  RX
} SX1278_Status_t;

// SX127X_REG_SYNC_WORD
#define SX127X_SYNC_WORD 0x12               // default LoRa sync word
#define SX127X_SYNC_WORD_LORAWAN 0x34       // sync word reserved for LoRaWAN networks

typedef struct
{
  uint32_t pin;
  GPIO_TypeDef *port;

} SX1278_gpio_t;

typedef struct
{
  SX1278_gpio_t reset;
  SX1278_gpio_t dio0;
  SX1278_gpio_t nss;
  SPI_HandleTypeDef *spi;

} SX1278_pins_t;

typedef struct
{
  SX1278_pins_t *hw;

  uint64_t frequency;
  uint8_t power;
  uint8_t LoRa_SF;
  uint8_t LoRa_BW;
  uint8_t LoRa_CR;
  uint8_t LoRa_CRC_sum;
  uint8_t packetLength;
  uint8_t sync_word;

  SX1278_Status_t status;

  uint8_t rxBuffer[SX1278_MAX_PACKET];
  uint8_t readBytes;

} SX1278_t;


void SX1278_hw_init(SX1278_pins_t *hw);

void SX1278_SetNSS(SX1278_pins_t *hw, int value);

void SX1278_Reset(SX1278_pins_t *hw);

void SX1278_SPICommand(SX1278_pins_t *hw, uint8_t cmd);

uint8_t SX1278_SPIReadByte(SX1278_pins_t *hw);

void SX1278_DelayMs(uint32_t msec);

int SX1278_GetDIO0(SX1278_pins_t *hw);

uint8_t SX1278_SPIRead(SX1278_t *module, uint8_t addr);

void SX1278_SPIWrite(SX1278_t *module, uint8_t addr, uint8_t cmd);

void SX1278_SPIBurstRead(SX1278_t *module, uint8_t addr, uint8_t *rxBuf, uint8_t length);

void SX1278_SPIBurstWrite(SX1278_t *module, uint8_t addr, uint8_t *txBuf, uint8_t length);

void SX1278_config(SX1278_t *module);

void SX1278_clearLoRaIrq(SX1278_t *module);

int SX1278_LoRaEntryRx(SX1278_t *module, uint8_t length, uint32_t timeout);

uint8_t SX1278_LoRaRxPacket(SX1278_t *module);

int SX1278_LoRaEntryTx(SX1278_t *module, uint8_t length, uint32_t timeout);

int SX1278_LoRaTxPacket(SX1278_t *module, uint8_t *txBuf, uint8_t length, uint32_t timeout);

void SX1278_init(SX1278_t *module, uint64_t frequency, uint8_t power,
                 uint8_t LoRa_SF, uint8_t LoRa_BW, uint8_t LoRa_CR,
                 uint8_t LoRa_CRC_sum, uint8_t packetLength, uint8_t sync_word);

int SX1278_transmit(SX1278_t *module, uint8_t *txBuf, uint8_t length, uint32_t timeout);

int SX1278_receive(SX1278_t *module, uint8_t length, uint32_t timeout);

uint8_t SX1278_available(SX1278_t *module);

uint8_t SX1278_read(SX1278_t *module, uint8_t *rxBuf, uint8_t length);

uint8_t SX1278_RSSI_LoRa(SX1278_t *module);

uint8_t SX1278_RSSI(SX1278_t *module);

void SX1278_standby(SX1278_t *module);

void SX1278_sleep(SX1278_t *module);

#ifdef __cplusplus
}
#endif

#endif
