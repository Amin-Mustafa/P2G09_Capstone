#ifndef THP_SENSOR_H
#define THP_SENSOR_H

#include <stdbool.h>

typedef struct {
    float temperature_C;
    float humidity_percent;
    float pressure_hPa;
    bool  is_valid;
} ThpData_t;

bool BME280_Init(void);
ThpData_t BME280_GetData(void);
ThpData_t BME280_GetSimData(void);

#endif
