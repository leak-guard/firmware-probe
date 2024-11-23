/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "lptim.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SX1278.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t StopModeFlag = 1;
volatile uint8_t WakeUpFlag = 0;
volatile uint8_t AlarmActiveFlag = 0;

uint32_t uid[3];
char* device_id;

SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

int ret;

char buffer[512];

int message;
int message_length;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void EnterStopMode(void);
void BlinkLED(void);
void SendLoRaMessage(const char* msg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_LPTIM1_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin, GPIO_PIN_RESET);

  //initialize LoRa module
  SX1278_hw.dio0.port = LORA_DIO0_GPIO_Port;
  SX1278_hw.dio0.pin = LORA_DIO0_Pin;
  SX1278_hw.nss.port = LORA_NSS_GPIO_Port;
  SX1278_hw.nss.pin = LORA_NSS_Pin;
  SX1278_hw.reset.port = LORA_RESET_GPIO_Port;
  SX1278_hw.reset.pin = LORA_RESET_Pin;
  SX1278_hw.spi = &hspi1;

  SX1278.hw = &SX1278_hw;

  SX1278_init(&SX1278, SX1278_FREQ_433MHz, SX1278_POWER_20DBM, SX1278_LORA_SF_7,
              SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_DIS, 8, SX127X_SYNC_WORD);

  SX1278_LoRaEntryTx(&SX1278, 16, 2000);

  SX1278_sleep(&SX1278);
  HAL_Delay(100); // Wait for the module to sleep

  HAL_GPIO_WritePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (StopModeFlag)
    {
      EnterStopMode();
    }

    if (WakeUpFlag)
    {
      if (AlarmActiveFlag)
      {
        SendLoRaMessage("02");
        BlinkLED();
      }
      else
      {
        BlinkLED();
      }

      WakeUpFlag = 0;
      StopModeFlag = 1;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                                     |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  StopModeFlag = 0;
  BlinkLED();
  StopModeFlag = 1;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN_PING_Pin)
  {
    StopModeFlag = 0;
    WakeUpFlag = 1;
    AlarmActiveFlag = 0;
  }
  else if (GPIO_Pin == WATER_ALARM_IN_Pin)
  {
    StopModeFlag = 0;
    WakeUpFlag = 1;
    AlarmActiveFlag = 1;
  }
}

void GetDeviceUID()
{
  uid[0] = HAL_GetUIDw0();
  uid[1] = HAL_GetUIDw1();
  uid[2] = HAL_GetUIDw2();

  sprintf(device_id, "%08lX%08lX%08lX", uid[0], uid[1], uid[2]);
}

void EnterStopMode()
{
  HAL_SuspendTick();

  // HAL_PWR_EnableSleepOnExit();

  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  // HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  SystemClock_Config();
  HAL_ResumeTick();
}

void SendLoRaMessage(const char* msg)
{
  SX1278_standby(&SX1278);

  HAL_Delay(200);

  message_length = sprintf(buffer, "%s", msg);

  if (SX1278_LoRaEntryTx(&SX1278, message_length, 2000))
  {
    SX1278_LoRaTxPacket(&SX1278, (uint8_t*)buffer, message_length, 2000);

    HAL_Delay(100);
  }

  SX1278_sleep(&SX1278);
}

void BlinkLED()
{
  HAL_GPIO_WritePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin, GPIO_PIN_SET);

  for (uint8_t i = 0; i < 10; i++)
  {
    HAL_GPIO_WritePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin, GPIO_PIN_RESET);  // Turn ON
    HAL_Delay(500);
    HAL_GPIO_WritePin(LED_ALARM_GPIO_Port, LED_ALARM_Pin, GPIO_PIN_SET);    // Turn OFF
    HAL_Delay(500);

    if (AlarmActiveFlag && (i % 5 == 0))
    {
      SendLoRaMessage("02");
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
