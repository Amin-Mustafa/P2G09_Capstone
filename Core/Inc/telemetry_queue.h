#ifndef TELEMETRY_QUEUE_H
#define TELEMETRY_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include "sensor_fusion.h"

#define QUEUE_MAX_SIZE  500  // Stores ~41 hours of data at 5-min intervals

// The compressed record we store in memory
typedef struct {
    uint32_t local_rtc_stamp;
    float water_cm;
    float rise_rate;
    float temperature_C;
    float humidity_percent;
    float pressure_hpa;
    float pressure_trend;
    uint32_t rain_energy;
    RiskLevel_t risk_level;
} TelemetryRecord_t;

void Queue_Init(void);

// Push new data into the buffer
bool Queue_Push(TelemetryRecord_t record);

// Peek at the oldest unsent data without removing it
bool Queue_Peek(TelemetryRecord_t *record);

// Remove the oldest data (Call this ONLY after a successful Wi-Fi transmission)
void Queue_Pop(void);

// Check if there is data waiting to be sent
bool Queue_HasPendingData(void);

#endif // TELEMETRY_QUEUE_H
