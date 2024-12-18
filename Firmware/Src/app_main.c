#include "app_main.h"

#define ADC_FS              4095U       // ADC in 12bit resolution
#define VREF_MEAS_MV        3000U       // VDDA voltage during VREFINT factory calibration in mV.
#define VREFINT_CAL         (*((uint16_t *)0x1FF80078)) // DS12323 Table 17.

volatile uint8_t StopModeFlag = 1;
volatile uint8_t WakeUpFlag = 0;

static uint16_t adcIn17Raw = 0;
static uint16_t VDDAmV = 0;
static uint16_t VREFINTmV = 0;

SX1278_pins_t SX1278_hw;
SX1278_t SX1278;

Message msg __attribute__((aligned(4)));

volatile uint8_t blink_count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  if (htim == &htim2)
  {
    static uint8_t blinks_done = 0;
    if (WakeUpFlag && blink_count > 0)
    {
      HAL_GPIO_TogglePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin);
      blinks_done++;

      if (blinks_done >= (blink_count * 2))
      {
        blinks_done = 0;
        blink_count = 0;
        LedOff();
      }
    }
  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  StopModeFlag = 0;
  WakeUpFlag = 1;

  msg.MsgType = 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  StopModeFlag = 0;
  WakeUpFlag = 1;

  if (GPIO_Pin == BTN_PING_Pin)
  {
    msg.MsgType = 0;
  }
  else if (GPIO_Pin == WATER_ALARM_IN_Pin || GPIO_Pin == BTN_TEST_Pin)
  {
    msg.MsgType = 2;
  }
}

void EnterStopMode()
{
  HAL_SuspendTick();

  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  SystemClock_Config();

  HAL_ResumeTick();
}

void LoraInit()
{
  SX1278_hw.dio0.port = LORA_DIO0_GPIO_Port;
  SX1278_hw.dio0.pin = LORA_DIO0_Pin;
  SX1278_hw.nss.port = LORA_NSS_GPIO_Port;
  SX1278_hw.nss.pin = LORA_NSS_Pin;
  SX1278_hw.reset.port = LORA_RESET_GPIO_Port;
  SX1278_hw.reset.pin = LORA_RESET_Pin;
  SX1278_hw.spi = &hspi1;

  SX1278.hw = &SX1278_hw;

  SX1278_init(&SX1278, SX1278_FREQ_433MHz, SX1278_POWER_11DBM, SX1278_LORA_SF_8,
              SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_DIS, 8, SX127X_SYNC_WORD);

  SX1278_LoRaEntryTx(&SX1278, 16, 2000);
}

void LoRaSleep()
{
  SX1278_sleep(&SX1278);
  HAL_Delay(100);
}

void LoRaWakeUp()
{
  SX1278_standby(&SX1278);
  HAL_Delay(200);
}

void LoRaSendPacket(uint8_t amount)
{
  for (uint8_t i = 0; i < amount; i++)
  {
    if (SX1278_LoRaEntryTx(&SX1278, sizeof(msg), 2000))
    {
      SX1278_LoRaTxPacket(&SX1278, (uint8_t*)&msg, sizeof(msg), 2000);
      HAL_Delay(100);
    }
  }
}

void LoRaMessageHandler()
{
  ReadDIPSwitch();
  MeasureBatteryVoltage();

  msg.crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)&msg,
    (sizeof(Message) - sizeof(uint32_t)) / sizeof(uint32_t));

  blink_count = 3;
  LedOn();

  LoRaSendPacket(5);
}

void PullUpEn(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint16_t combinedPins = 0;

  // Process pins until we hit a 0 (sentinel value)
  while(*GPIO_Pins != 0) {
    combinedPins |= *GPIO_Pins;
    GPIO_Pins++;
  }

  GPIO_InitStruct.Pin = combinedPins;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void PinsToAnalog(GPIO_TypeDef* GPIOx, const uint16_t* GPIO_Pins)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint16_t combinedPins = 0;

  // Process pins until we hit a 0 (sentinel value)
  while(*GPIO_Pins != 0) {
    combinedPins |= *GPIO_Pins;
    GPIO_Pins++;
  }

  GPIO_InitStruct.Pin = combinedPins;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void LedOn()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = LED_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_TIM_Base_Start_IT(&htim2);
}

void LedOff()
{
  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_GPIO_WritePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin, GPIO_PIN_SET);

  uint16_t pins[] = { LED_ALARM_Pin, 0 };

  PinsToAnalog(LED_ALARM_GPIO_Port, pins);
}

void GetDeviceUID()
{
  msg.uid1 = HAL_GetUIDw0();
  msg.uid2 = HAL_GetUIDw1();
  msg.uid3 = HAL_GetUIDw2();
}

void ReadDIPSwitch()
{
  uint16_t pins[] = {ADDR0_Pin, ADDR1_Pin, ADDR2_Pin, ADDR3_Pin, ADDR4_Pin, ADDR5_Pin, ADDR6_Pin, ADDR7_Pin, 0};  // Note the 0 at the end
  PullUpEn(GPIOA, pins);

  uint8_t dip_switch = LL_GPIO_ReadInputPort(ADDR0_GPIO_Port) & 0xFF;

  msg.dipId = dip_switch;

  PinsToAnalog(GPIOA, pins);
}

void MeasureBatteryVoltage()
{
  HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc);

  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  adcIn17Raw = HAL_ADC_GetValue(&hadc);
  VDDAmV = (VREF_MEAS_MV * VREFINT_CAL) / adcIn17Raw;
  VREFINTmV = (VDDAmV * adcIn17Raw) / ADC_FS;

  msg.batMvol = VDDAmV;
}

void DeviceControl_Init(void)
{
  GetDeviceUID();

  LoraInit();

  msg.MsgType = 1;
  LoRaMessageHandler();

  LoRaSleep();
}

void DeviceControl_Process(void)
{
  if (StopModeFlag)
  {
    LedOff();
    LoRaSleep();
    EnterStopMode();
  }

  if (WakeUpFlag)
  {
    LoRaWakeUp();
    LedOn();
    LoRaMessageHandler();
    LoRaSleep();

    WakeUpFlag = 0;
    StopModeFlag = 1;
  }
}
