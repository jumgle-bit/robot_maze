#include "delay.h"
#include "system_stm32f103.h"

/*
 * TIM3 配置为 1 MHz 自由运行定时器：
 * CNT 每 1 us 加 1，16 bit 溢出周期约 65 ms。
 */
// 初始化 TIM3 为 1 MHz 计数器，用作微秒/毫秒延时基准。
void Delay_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = (uint16_t)(SystemCoreClock / 1000000UL - 1UL);
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
}

// 返回 TIM3 当前 16 位计数值，适合做短时间差计算。
uint16_t Delay_GetMicros16(void)
{
    return (uint16_t)TIM3->CNT;
}

// 忙等待指定微秒数，依赖 TIM3 的 1 MHz 计数。
void Delay_us(uint16_t us)
{
    uint16_t start = Delay_GetMicros16();
    while ((uint16_t)(Delay_GetMicros16() - start) < us)
    {
        /* wait */
    }
}

// 通过循环调用微秒延时实现毫秒级阻塞延时。
void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}
