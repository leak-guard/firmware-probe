#pragma once

#include <stm32l0xx_ll_gpio.h>
#include <stm32l0xx_hal.h>
#include <stdio.h>
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"
#include "SX1278.h"


enum MsgType {
  PING = 0,
  BATTERY = 1,
  ALARM = 2
};

typedef struct Message {
  uint8_t MsgType;
  uint32_t uid1;
  uint32_t uid2;
  uint32_t uid3;
  uint8_t dip_id;
  uint16_t bat_mvol;
  uint32_t crc;
} Message;

void SystemClock_Config(void);
void PullUpEn(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins);
void PinsToAnalog(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins);
void GetDeviceUID(void);
void ReadDIPSwitch(void);
void MeasureBatteryVoltage(void);
void EnterStopMode(void);
void SendLoRaMessage(struct Message* message);
void LoRaSleep(void);
void LoRaWakeUp(void);

void DeviceControl_Init(void);
void DeviceControl_Process(void);
