#include "piezo.h"
#include "adc.h"
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
static uint32_t baseline_noise = 0;

void Piezo_Init(void) {
	// Take 100 quick readings while it is dry to establish the noise profile
	uint32_t accumulator = 0;

	for(uint8_t i = 0; i < 100; ++i) {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 5);
		accumulator += HAL_ADC_GetValue(&hadc1);
	}

	baseline_noise = accumulator / 100;
}

uint32_t Piezo_GetRainIntensity(uint32_t listen_window_ms) {
	uint32_t max_peak = 0;
	uint32_t start_time = HAL_GetTick();

	while ((HAL_GetTick() - start_time) < listen_window_ms) {

		HAL_ADC_Start(&hadc1);

		// Timeout of 5ms is plenty for a 1us conversion
		if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
			uint32_t current_val = HAL_ADC_GetValue(&hadc1);

			// Capture the highest peak seen in this window
			if (current_val > max_peak) {
				max_peak = current_val;
			}
		}
	}

	// Subtract the ambient baseline noise
	if (max_peak > baseline_noise) {
		return (max_peak - baseline_noise);
	} else {
		return 0; // No rain detected above the noise floor
	}
}
