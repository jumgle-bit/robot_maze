#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "system_stm32f103.h"

#ifdef __cplusplus
extern "C" {
#endif

void GPIO_ConfigPin(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t mode_cnf);
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t level);
void GPIO_SetPin(GPIO_TypeDef *GPIOx, uint8_t pin);
void GPIO_ResetPin(GPIO_TypeDef *GPIOx, uint8_t pin);
uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif
