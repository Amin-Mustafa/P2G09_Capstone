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

// Custom
#include "ultrasonic.h"
#include "thp_sensor.h"
#include "piezo.h"
#include "sensor_fusion.h"
#include "telemetry_queue.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//------- Definitions & Globals -------//

#define ESP_WAKEUP_TIME	12000
#define ESP_ACK_TIME	6000

typedef enum {
	STATE_WAKE_INIT,
	STATE_POLL_AND_FUSE,
	STATE_TRANSMIT_WIFI,
	STATE_ENTER_SLEEP
} SystemState_t;

volatile SystemState_t current_state = STATE_WAKE_INIT;

// Structs to hold the data as it moves through the pipeline
RawSensorData_t current_raw_data = {0};
FusedData_t current_evaluation = {0};

//------- Prototypes -------//
void SystemClock_Config(void);
bool ESP32_TransmitPayload(void);
void EnterDeepSleep(uint32_t seconds);
void Enable_RTC_Wakeup_Interrupt(void);
uint32_t Get_Local_RTC_Seconds(void);

int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_RTC_Init();
	Enable_RTC_Wakeup_Interrupt();
	DWT_Init();

	Ultrasonic_Init();
	Piezo_Init();     // Establishes the dry baseline noise floor
	BME280_Init();
	SensorFusion_Init();

	Queue_Init();

	srand(Get_Local_RTC_Seconds());

	while(1) {
		switch(current_state) {
		case STATE_WAKE_INIT:
			// Turn on debug LED to indicate MCU is awake
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			current_state = STATE_POLL_AND_FUSE;
			break;
		case STATE_POLL_AND_FUSE:
			current_raw_data.water_distance_cm	= Ultrasonic_GetDistance();
			current_raw_data.rain_energy		= Piezo_GetRainIntensity(1000);
			current_raw_data.atm 				= BME280_GetData();

			current_evaluation = SensorFusion_Process(current_raw_data);

			TelemetryRecord_t new_record;
			new_record.local_rtc_stamp = Get_Local_RTC_Seconds();
			new_record.water_cm = current_evaluation.smoothed_water_cm;
			new_record.rise_rate = current_evaluation.water_rise_rate_cm_min;
			new_record.temperature_C = current_raw_data.atm.temperature_C;
			new_record.humidity_percent = current_raw_data.atm.humidity_percent;
			new_record.pressure_hpa = current_raw_data.atm.pressure_hPa;
			new_record.pressure_trend = current_evaluation.pressure_trend_hpa;
			new_record.rain_energy = current_raw_data.rain_energy;
			new_record.risk_level = current_evaluation.risk_level;

			Queue_Push(new_record);

			current_state = STATE_TRANSMIT_WIFI;
			break;
		case STATE_TRANSMIT_WIFI:
			ESP32_TransmitPayload();
			current_state = STATE_ENTER_SLEEP;
			break;
		case STATE_ENTER_SLEEP:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			EnterDeepSleep(current_evaluation.next_sleep_s);

			// MCU wakes here
			current_state = STATE_WAKE_INIT;
			break;
		}
	}
}

bool ESP32_TransmitPayload(void) {
	uint8_t rx_buffer[1] = {0};
	char json_payload[256];

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

    // Wait up to 500ms for ESP32 to wake up and reply 'R'
	if (HAL_UART_Receive(&huart1, rx_buffer, 1, ESP_WAKEUP_TIME) == HAL_OK && rx_buffer[0] == 'R') {

        uint32_t current_time = Get_Local_RTC_Seconds();

        while (Queue_HasPendingData()) {
            TelemetryRecord_t pending;
            Queue_Peek(&pending);

            // Calculate how old this data is (in seconds)
            // If it just happened, age will be ~0. If Wi-Fi was down for an hour, age will be 3600.
            uint32_t age_seconds = current_time - pending.local_rtc_stamp;

            sprintf(json_payload,
				"{\"id\":\"node_01\",\"age\":%lu,\"lvl\":%.1f,\"rate\":%.2f,\"tmp\":%.1f,\"hum\":%.1f,\"prs\":%.1f,\"ptrend\":%.2f,\"rn\":%lu,\"st\":%d}\n",
				age_seconds,
				pending.water_cm,
				pending.rise_rate,
				pending.temperature_C,
				pending.humidity_percent,
				pending.pressure_hpa,
				pending.pressure_trend,
				pending.rain_energy,
				pending.risk_level);

			HAL_UART_Transmit(&huart1, (uint8_t*)json_payload, strlen(json_payload), 200);

            // Wait up to 2 seconds for ESP32 to confirm it published to MQTT ('K' for aCK)
            if (HAL_UART_Receive(&huart1, rx_buffer, 1, ESP_ACK_TIME) == HAL_OK && rx_buffer[0] == 'K') {
                Queue_Pop(); // Safely delete from SRAM only after server receives it
            } else {
                break; // Wi-Fi/MQTT failed mid-transmission. Leave rest in queue and sleep.
            }
        }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        return true;
	}

    // Wi-Fi is down. ESP32 did not respond. Data remains safely in Queue.
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	return false;
}

uint32_t Get_Local_RTC_Seconds(void) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // HAL requires reading BOTH Time and Date to unlock the shadow registers
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Convert into a flat linear integer of elapsed seconds.
    uint32_t total_seconds = (sDate.Date * 86400) +
                             (sTime.Hours * 3600) +
                             (sTime.Minutes * 60) +
                             sTime.Seconds;

    return total_seconds;
}

void Enable_RTC_Wakeup_Interrupt(void) {
    // STM32F411 maps the RTC Wakeup to EXTI Line 22
    EXTI->IMR |= (1 << 22);  // Unmask the interrupt
    EXTI->RTSR |= (1 << 22); // Trigger on rising edge

    // Enable the interrupt in the Nested Vector Interrupt Controller (NVIC)
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
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

void RTC_WKUP_IRQHandler(void)
{
    // Clears the hardware flags so it doesn't trigger infinitely
    HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
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
