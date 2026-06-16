#ifndef __SYSTEM_STM32F103_H
#define __SYSTEM_STM32F103_H

#include "stm32f103_min.h"

#ifdef __cplusplus
extern "C" {
#endif

// 当前系统核心时钟频率，本工程固定为 8 MHz。
extern uint32_t SystemCoreClock;

// 启动阶段系统初始化入口。
void SystemInit(void);
// 更新 SystemCoreClock 变量。
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
