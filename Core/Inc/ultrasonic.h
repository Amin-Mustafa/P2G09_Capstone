#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

void Ultrasonic_Init(void);
void Ultrasonic_Trigger(void);
bool Ultrasonic_IsReady(void);
float Ultrasonic_GetDistance(void);
float Ultrasonic_PollSensor(uint32_t timeout_ms);

#endif
