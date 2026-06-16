#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "system_stm32f103.h"

#ifdef __cplusplus
extern "C" {
#endif

// 配置单个 GPIO 引脚模式。
void GPIO_ConfigPin(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t mode_cnf);
// 写 GPIO 电平。
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t level);
// 输出高电平。
void GPIO_SetPin(GPIO_TypeDef *GPIOx, uint8_t pin);
// 输出低电平。
void GPIO_ResetPin(GPIO_TypeDef *GPIOx, uint8_t pin);
// 读取 GPIO 输入电平。
uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif
