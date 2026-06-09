#include "thp_sensor.h"
#include "bme280.h"
#include "system.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define BME280_I2C_ADDR 0x76

static struct bme280_dev dev;
static struct bme280_data raw_data;
static bool last_measurement_success = false;

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
    // 1. Link your STM32 I2C functions to the Bosch driver
    dev.intf_ptr = NULL;
    dev.read = bme280_i2c_read;
    dev.write = bme280_i2c_write;
    dev.delay_us = bme280_delay_us;
    dev.intf = BME280_I2C_INTF;

    // 2. Start the sensor
    if (bme280_init(&dev) != BME280_OK) {
        return false;
    }

    // 3. Set up oversampling settings
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

void BME280_WakeAndMeasure(void) {
    // Set mode to FORCED to trigger one measurement
    int8_t rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &dev);
    if (rslt != BME280_OK) {
        last_measurement_success = false;
        return;
    }

    dev.delay_us(20000, dev.intf_ptr);

    rslt = bme280_get_sensor_data(BME280_ALL, &raw_data, &dev);

    last_measurement_success = (rslt == BME280_OK);
}

ThpData_t BME280_GetData(void) {
    ThpData_t formatted_data;

    formatted_data.is_valid = last_measurement_success;

    if (last_measurement_success) {
        // Convert Bosch integer outputs to float
        formatted_data.temperature_C = (float)raw_data.temperature;

        // Pressure: 101325 Pa becomes 1013.25 hPa
        formatted_data.pressure_hPa = (float)raw_data.pressure / 100.0f;

        formatted_data.humidity_percent = (float)raw_data.humidity;
    } else {
        // Clear data if reading failed
        formatted_data.temperature_C = 0.0f;
        formatted_data.pressure_hPa = 0.0f;
        formatted_data.humidity_percent = 0.0f;
    }

    return formatted_data;
}

