#pragma once

#include <stm32l0xx_ll_gpio.h>
#include <stm32l0xx_hal.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "SX1278.h"


enum MsgType {
  PING = 0, // PAIR: IDs & battery voltage
  BATTERY = 1, // every 12 hrs
  ALARM = 2 // also TEST
};

typedef struct __attribute__((packed)) Message {
  uint8_t MsgType;
  uint8_t dipId;
  uint16_t batMvol;
  uint32_t uid1;
  uint32_t uid2;
  uint32_t uid3;
  uint32_t crc;
} Message;

void SystemClock_Config(void);
void PullUpEn(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins);
void PinsToAnalog(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins);
void GetDeviceUID(void);
void ReadDIPSwitch(void);
void MeasureBatteryVoltage(void);
void EnterStopMode(void);
void LoraInit(void);
void LoRaSendPacket(void);
void LoRaSleep(void);
void LoRaWakeUp(void);
void LedOn(void);
void LedOff(void);

void DeviceControl_Init(void);
void DeviceControl_Process(void);
