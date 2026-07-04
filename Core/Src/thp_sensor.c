#include "thp_sensor.h"
#include "bme280.h"
#include "system.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;

#define BME280_I2C_ADDR 0x76

static struct bme280_dev dev;

BME280_INTF_RET_TYPE bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)BME280_I2C_ADDR << 1, reg_addr, 1, reg_data, len, 100) != HAL_OK) {
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}

BME280_INTF_RET_TYPE bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    if (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)BME280_I2C_ADDR << 1, reg_addr, 1, (uint8_t*)reg_data, len, 100) != HAL_OK) {
        return BME280_E_COMM_FAIL;
    }
    return BME280_OK;
}


void bme280_delay_us(uint32_t period, void *intf_ptr) {
    DWT_Delay_us(period);
}

bool BME280_Init(void) {
    // Link STM32 I2C functions to the Bosch driver
    dev.intf_ptr = NULL;
    dev.read = bme280_i2c_read;
    dev.write = bme280_i2c_write;
    dev.delay_us = bme280_delay_us;
    dev.intf = BME280_I2C_INTF;

    // Start the sensor
    if (bme280_init(&dev) != BME280_OK) {
        return false;
    }

    // Set up oversampling settings
    struct bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_1X;
    settings.osr_t = BME280_OVERSAMPLING_1X;
    settings.filter = BME280_FILTER_COEFF_OFF; // No filter for forced mode

    if (bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &dev) != BME280_OK) {
        return false;
    }

    return true;
}

ThpData_t BME280_GetData(void) {
    ThpData_t formatted_data = {0};
    struct bme280_data raw_data;

    int8_t rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &dev);

    if (rslt != BME280_OK) {
        formatted_data.is_valid = false;
        return formatted_data;
    }

    // Wait 10ms for 1x oversampling to complete (Max time is 9.3ms)
    HAL_Delay(10);

    // Retrieve the calculated data
    rslt = bme280_get_sensor_data(BME280_ALL, &raw_data, &dev);

    // Format for the Fusion Engine
    if (rslt == BME280_OK) {
        formatted_data.temperature_C = (float)raw_data.temperature;
        formatted_data.pressure_hPa = (float)raw_data.pressure / 100.0f; // Pa to hPa
        formatted_data.humidity_percent = (float)raw_data.humidity;
        formatted_data.is_valid = true;
    } else {
        formatted_data.is_valid = false;
    }

    return formatted_data;
}


ThpData_t BME280_GetSimData(void) {
    ThpData_t sim_data;

    // Generate slight random noise for organic sensor jitter
    float temp_noise = ((float)rand() / (float)RAND_MAX) * 0.3f - 0.15f;
    float hum_noise  = ((float)rand() / (float)RAND_MAX) * 1.5f - 0.75f;
    float pres_noise = ((float)rand() / (float)RAND_MAX) * 0.4f - 0.2f;

    // Apply noise to baselines
    sim_data.temperature_C    = 26.8 + temp_noise;      // Mid-morning heat
    sim_data.humidity_percent = 85.5f + hum_noise;       // Lingering morning moisture
    sim_data.pressure_hPa     = 1010.2f + pres_noise;    // Diurnal pressure peak

    return sim_data;
}
