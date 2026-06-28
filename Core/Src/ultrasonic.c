#include "ultrasonic.h"
#include "system.h"
#include "stm32f4xx_hal.h"

#define TRIG_PORT	GPIOB
#define TRIG_PIN	GPIO_PIN_4

#define ECHO_PORT	GPIOB
#define ECHO_PIN	GPIO_PIN_3

#define BURST_SAMPLES		5
#define PING_TIMEOUT_MS		50
#define ACOUSTIC_DELAY_MS	80
#define MIN_RANGE_CM		25.0f
#define MAX_RANGE_CM		600.0f
#define SPEED_OF_SOUND_CMUS	0.0343f

extern TIM_HandleTypeDef htim2;

static volatile bool echo_ready = false;
static volatile uint32_t echo_time_us = 0;

static inline void swap(float *a, float *b) {
    float temp = *a;
    *a = *b;
    *b = temp;
}

static void sort_array(float arr[], uint8_t n) {
    for (uint8_t i = 1; i < n; i++) {
        float key = arr[i];
        int8_t j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }
}

static float ping_ultrasonic(void) {
	echo_ready = false;

	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	DWT_Delay_us(10);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

	uint32_t timeout_start = HAL_GetTick();
	while (!echo_ready) {
		if ((HAL_GetTick() - timeout_start) > PING_TIMEOUT_MS) {
			return -1.0f; // Hardware timeout or object out of range
		}
	}

	return (echo_time_us * SPEED_OF_SOUND_CMUS) / 2.0f;
}

void Ultrasonic_Init(void) {
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
	DWT_Init();
}

float Ultrasonic_GetDistance(void) {
    float readings[BURST_SAMPLES];
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < BURST_SAMPLES; i++) {
        float dist = ping_ultrasonic();

        if (dist > MIN_RANGE_CM && dist < MAX_RANGE_CM) {
            readings[valid_count] = dist;
            valid_count++;
        }
        HAL_Delay(ACOUSTIC_DELAY_MS);
    }

    if (valid_count == 0) {
        return -1.0f; // All pings timed out; sensor might be unplugged or broken.
    }

    // Median filter
    sort_array(readings, valid_count);
    return readings[valid_count / 2];
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ECHO_PIN) { // PB3 is our Echo pin

        // Rising Edge
        if (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET) {
            __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset stopwatch
            HAL_TIM_Base_Start(&htim2);       // Start counting microseconds
        }
        // Falling Edge
        else {
            echo_time_us = __HAL_TIM_GET_COUNTER(&htim2); // Grab the time
            HAL_TIM_Base_Stop(&htim2);                    // Stop stopwatch
            echo_ready = true;                            // Flag FSM
        }
    }
}

void EXTI3_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}
