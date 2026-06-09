#include "ultrasonic.h"
#include "system.h"
#include "stm32f4xx_hal.h"

#define TRIG_PORT	GPIOB
#define TRIG_PIN	GPIO_PIN_4

#define ECHO_PORT	GPIOB
#define ECHO_PIN	GPIO_PIN_3

extern TIM_HandleTypeDef htim2;

static volatile bool echo_ready = false;
static volatile uint32_t echo_time_us = 0;
static float final_distance_cm = 0.0f;

void Ultrasonic_Init(void) {
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

	DWT_Init();
}

void Ultrasonic_Trigger(void) {
    echo_ready = false;

    // Fire the trigger pulse
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

    // Perfect 10us hardware-timed delay
    DWT_Delay_us(10);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

bool Ultrasonic_IsReady(void) {
	return echo_ready;
}

float Ultrasonic_GetDistance(void) {
    // Speed of sound is ~340 m/s or 0.034 cm/us.
    // Divide by 2 because the sound goes out and bounces back.
    final_distance_cm = (echo_time_us * 0.0343f) / 2.0f;
    return final_distance_cm;
}

float Ultrasonic_PollSensor(uint32_t timeout_ms) {
	Ultrasonic_Trigger();

	// Record the time we fired it for timeout protection
	uint32_t timeout_start = HAL_GetTick();

	while (!Ultrasonic_IsReady()) {
		if ((HAL_GetTick() - timeout_start) > timeout_ms) {
			return final_distance_cm;
		}
	}

	return Ultrasonic_GetDistance();
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

void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}
