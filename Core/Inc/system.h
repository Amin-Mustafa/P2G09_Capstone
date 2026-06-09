#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

void SystemClock_Config(void);
void DWT_Init(void);
void DWT_Delay_us(uint32_t us);

#endif
