#include "piezo.h"
#include "adc.h"
#include "stm32f4xx_hal.h"

#define SAMPLE_INTERVAL_MS	10

// Only accept voltage spikes 4 points ABOVE the idle state
#define PIEZO_NOISE_BUFFER	4

#define PIEZO_GAIN          15

#define PIEZO_MAX_ENERGY        1000

extern ADC_HandleTypeDef hadc1;
static uint32_t baseline_noise = 0;

void Piezo_Init(void) {
    uint32_t accumulator = 0;
    const uint8_t CALIBRATION_SAMPLES = 50;

    // Take 50 samples spaced 10ms apart to establish the resting voltage
    for(uint8_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
            accumulator += HAL_ADC_GetValue(&hadc1);
        }
        HAL_Delay(SAMPLE_INTERVAL_MS);
    }

    baseline_noise = accumulator / CALIBRATION_SAMPLES;

    // Add a tiny buffer (e.g., 10-20 ADC ticks) to act as a deadband
    // against natural thermal drift of the Schottky diode over time.
    baseline_noise += 15;
}

uint32_t Piezo_GetRainIntensity(uint32_t listen_window_ms) {
    uint32_t total_energy = 0;
    uint32_t start_time = HAL_GetTick();

    // Listen as fast as possible
    while ((HAL_GetTick() - start_time) < listen_window_ms) {
        HAL_ADC_Start(&hadc1);

        if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
            uint32_t current_val = HAL_ADC_GetValue(&hadc1);

            if (current_val > baseline_noise + PIEZO_NOISE_BUFFER) {
                // Subtract the baseline
                total_energy += (current_val - baseline_noise);
            }
        }
    }

    // Amplify the real, isolated impacts
    uint32_t final_intensity = total_energy * PIEZO_GAIN;

    // Safety clamp to protect the Node-RED charts from overflowing
    if (final_intensity > 5000) {
        final_intensity = 5000;
    }

    return final_intensity;
}
