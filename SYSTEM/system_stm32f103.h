#ifndef __SYSTEM_STM32F103_H
#define __SYSTEM_STM32F103_H

#include "stm32f103_min.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;

void SystemInit(void);
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
