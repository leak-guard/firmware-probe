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


void SystemClock_Config(void);
void PullUpEn(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins);
void PinsToAnalog(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins);
void GetDeviceUID(void);
void ReadDIPSwitch(void);
void MeasureBatteryVoltage(void);
void EnterStopMode(void);
void SendLoRaMessage(const char* msg);
void LoRaSleep(void);
void LoRaWakeUp(void);

void DeviceControl_Init(void);
void DeviceControl_Process(void);
