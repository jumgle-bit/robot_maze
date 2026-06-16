#include "bsp_gpio.h"

// 配置 STM32F103 单个 GPIO 引脚的 CRL/CRH 模式位。
void GPIO_ConfigPin(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t mode_cnf)
{
    volatile uint32_t *config_reg;
    uint8_t shift;

    if (pin < 8)
    {
        config_reg = &GPIOx->CRL;
        shift = pin * 4;
    }
    else
    {
        config_reg = &GPIOx->CRH;
        shift = (pin - 8) * 4;
    }

    *config_reg &= ~(0xFUL << shift);
    *config_reg |=  ((uint32_t)(mode_cnf & 0x0F) << shift);
}

// 按给定电平写 GPIO，引脚置 1 用 BSRR，置 0 用 BRR。
void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t level)
{
    if (level)
    {
        GPIOx->BSRR = (1UL << pin);
    }
    else
    {
        GPIOx->BRR = (1UL << pin);
    }
}

// 将指定 GPIO 引脚输出高电平。
void GPIO_SetPin(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    GPIOx->BSRR = (1UL << pin);
}

// 将指定 GPIO 引脚输出低电平。
void GPIO_ResetPin(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    GPIOx->BRR = (1UL << pin);
}

// 读取指定 GPIO 输入电平，返回 1 或 0。
uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    return (GPIOx->IDR & (1UL << pin)) ? 1U : 0U;
}
