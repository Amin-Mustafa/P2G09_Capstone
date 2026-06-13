/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "system.h"
#include "ultrasonic.h"
#include "thp_sensor.h"
#include "piezo.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

//-------Definitions-------//
typedef enum {
	STATE_WAKE_INIT,
	STATE_POLL_SENSORS,
	STATE_FUSE_SENSORS,
	STATE_TRANSMIT_WIFI,
	STATE_ENTER_SLEEP
} SystemState_t;

typedef enum {
    STATUS_GREEN = 0,  // Clear skies
    STATUS_YELLOW = 1, // Rain detected or rising water
    STATUS_RED = 2     // Flash flood warning
} RiskLevel_t;

typedef struct {
    float water_level_cm;
    ThpData_t thp;           // <-- Nested BME280 struct
    uint32_t rain_intensity;
    RiskLevel_t risk_level;
} SensorData_t;

SystemState_t current_state = STATE_WAKE_INIT;
SensorData_t current_data = {0};
uint32_t sleep_duration_s = 300; // Default 5 minutes

//-------Prototypes-------//
void SystemClock_Config(void);
void SensorFusion(void);
bool ESP32_TransmitPayload(void);
void EnterDeepSleep(uint32_t seconds);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  DWT_Init();

  Ultrasonic_Init();
  Piezo_Init();
  BME280_Init();

  while (1)
  {
	  switch(current_state) {
	  case STATE_WAKE_INIT:
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		  current_state = STATE_POLL_SENSORS;
		  break;
	  case STATE_POLL_SENSORS:
		  BME280_WakeAndMeasure();
		  current_data.water_level_cm = Ultrasonic_PollSensor(50);
		  current_data.thp = BME280_GetData();
		  current_data.rain_intensity = Piezo_GetRainIntensity(500);

		  current_state = STATE_FUSE_SENSORS;
		  break;
	  case STATE_FUSE_SENSORS:
		  SensorFusion();
		  current_state = STATE_TRANSMIT_WIFI;
		  break;
	  case STATE_TRANSMIT_WIFI:
		  ESP32_TransmitPayload();
		  current_state = STATE_ENTER_SLEEP;
		  break;
	  case STATE_ENTER_SLEEP:
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		  EnterDeepSleep(sleep_duration_s);

		  // Halt until woken

		  current_state = STATE_WAKE_INIT;
		  break;
	  }
  }
}

void SensorFusion(void) {
	bool is_raining = (current_data.rain_intensity > 500);
	bool water_rising = (current_data.water_level_cm > 0 &&
							current_data.water_level_cm < 100.0f); // e.g., < 1 meter from sensor
	bool overflow = (current_data.water_level_cm > 0 &&
						current_data.water_level_cm < 30.0f);   // e.g., < 30 cm from sensor

	if (overflow) {
		current_data.risk_level = STATUS_RED;
		sleep_duration_s = 5;       // Stream data every 5 seconds
	}
	else if (water_rising || is_raining) {
		current_data.risk_level = STATUS_YELLOW;
		sleep_duration_s = 60;     // Log every minute
	}
	else {
		current_data.risk_level = STATUS_GREEN;
		sleep_duration_s = 300;   // Log every 5 minutes
	}
}

bool ESP32_TransmitPayload(void) {
	uint8_t rx_buffer[1] = {0};
	char json_payload[128];

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

	if (HAL_UART_Receive(&huart1, rx_buffer, 1, 500) == HAL_OK) {
		if (rx_buffer[0] == 'R') {
			sprintf(json_payload, "{\"lvl\":%.1f,\"tmp\":%.1f,\"hum\":%.1f,\"prs\":%.1f,\"rn\":%lu,\"st\":%d}\n",
			                    current_data.water_level_cm,
			                    current_data.thp.temperature_C,      // Nested access
			                    current_data.thp.humidity_percent,   // Nested access
			                    current_data.thp.pressure_hPa,       // Nested access
			                    current_data.rain_intensity,
			                    current_data.risk_level);

			HAL_UART_Transmit(&huart1, (uint8_t*)json_payload, strlen(json_payload), 100);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			return true;
		}
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	return false;
}

void EnterDeepSleep(uint32_t seconds) {
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, seconds, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

	HAL_SuspendTick();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	//MCU wakes up here

	// Resume SysTick
	SystemClock_Config();
	HAL_ResumeTick();
}

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
#ifdef USE_FULL_ASSERT
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
