#include "ir_sensor.h"
#include "config.h"
#include "delay.h"
#include "bsp_gpio.h"
#include "system_stm32f103.h"

/*
 * 左红外：PA12
 * 右红外：PA11   （已按要求由原 PB13 改为 PA11）
 * 输入默认上拉。当前配置为：检测到障碍物输出低电平。
 */
#define IR_LEFT_PORT     GPIOA
#define IR_LEFT_PIN      12

#define IR_RIGHT_PORT    GPIOA
#define IR_RIGHT_PIN     11

void IR_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIO_ConfigPin(IR_LEFT_PORT, IR_LEFT_PIN, GPIO_CNF_INPUT_PULL);
    GPIO_SetPin(IR_LEFT_PORT, IR_LEFT_PIN);     /* 输入上拉 */

    GPIO_ConfigPin(IR_RIGHT_PORT, IR_RIGHT_PIN, GPIO_CNF_INPUT_PULL);
    GPIO_SetPin(IR_RIGHT_PORT, IR_RIGHT_PIN);   /* 输入上拉 */
}

static uint8_t IR_ReadBlockedDebounced(GPIO_TypeDef *port, uint8_t pin)
{
    uint8_t i;
    uint8_t blocked_count = 0;

    for (i = 0; i < 3; i++)
    {
        uint8_t level = GPIO_ReadPin(port, pin);
        if (level == IR_BLOCKED_LEVEL)
        {
            blocked_count++;
        }
        Delay_ms(3);
    }

    return (blocked_count >= 2) ? 1U : 0U;
}

uint8_t IR_LeftBlocked(void)
{
    return IR_ReadBlockedDebounced(IR_LEFT_PORT, IR_LEFT_PIN);
}

uint8_t IR_RightBlocked(void)
{
    return IR_ReadBlockedDebounced(IR_RIGHT_PORT, IR_RIGHT_PIN);
}
