#pragma once

#include <stm32l0xx_ll_gpio.h>
#include <stm32l0xx_hal.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "crc.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"
#include "SX1278.h"


typedef enum {
  APP_STATE_IDLE,
  APP_STATE_MEASURE,
  APP_STATE_SEND,
  APP_STATE_SLEEP,
} AppState_t;


enum MsgType {
  PING = 0,
  BATTERY = 1,
  ALARM = 2
};

typedef struct __attribute__((packed)) Message {
  uint8_t MsgType;
  uint32_t uid1;
  uint32_t uid2;
  uint32_t uid3;
  uint8_t dipId;
  uint16_t batMvol;
  uint32_t crc;
} Message;

void SystemClock_Config(void);
void PullUpEn(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins);
void PinsToAnalog(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins);
void GetDeviceUID(void);
void ReadDIPSwitch(void);
void MeasureBatteryVoltage(void);
void EnterStopMode(void);
void SendLoRaMessage(void);
void LoRaSleep(void);
void LoRaWakeUp(void);

void DeviceControl_Init(void);
void DeviceControl_Process(void);
