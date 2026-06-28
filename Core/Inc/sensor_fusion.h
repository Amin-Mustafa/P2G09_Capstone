#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <stdint.h>
#include <stdbool.h>
#include "thp_sensor.h"	// For ThpData_t

#define SLEEP_CRITICAL	1
#define SLEEP_WARNING	2
#define SLEEP_NORMAL	3

typedef enum {
    STATUS_GREEN = 0,
    STATUS_YELLOW = 1,
    STATUS_RED = 2
} RiskLevel_t;

typedef struct {
    float water_distance_cm; // Median-filtered from ultrasonic
    uint32_t rain_energy;    // Integrated envelope from piezo
    ThpData_t atm;           // Clean BME280 reading
} RawSensorData_t;

// Processed data packet
typedef struct {
    float smoothed_water_cm;
    float water_rise_rate_cm_min;
    float pressure_trend_hpa;
    RiskLevel_t risk_level;
    uint32_t next_sleep_s;
} FusedData_t;

void SensorFusion_Init(void);

FusedData_t SensorFusion_Process(RawSensorData_t raw);

#endif
