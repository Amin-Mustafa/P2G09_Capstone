#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

void Ultrasonic_Init(void);
float Ultrasonic_GetDistance(void);

#endif
