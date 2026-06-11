#include "bsp_gpio.h"

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

void GPIO_SetPin(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    GPIOx->BSRR = (1UL << pin);
}

void GPIO_ResetPin(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    GPIOx->BRR = (1UL << pin);
}

uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint8_t pin)
{
    return (GPIOx->IDR & (1UL << pin)) ? 1U : 0U;
}
