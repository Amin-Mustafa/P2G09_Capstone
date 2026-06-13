#ifndef PIEZO_H
#define PIEZO_H

#include <stdint.h>

void Piezo_Init(void);
uint32_t Piezo_GetRainIntensity(uint32_t listen_window_ms);

#endif
