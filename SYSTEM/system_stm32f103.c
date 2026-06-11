#include "system_stm32f103.h"

/*
 * 为了提高通用性，本工程默认使用 STM32F103 上电后的 HSI 8 MHz 时钟。
 * 若使用 STM32CubeMX 或需要 72 MHz，可在这里增加 HSE+PLL 配置，
 * 同时保持 SystemCoreClock 数值正确即可。
 */
uint32_t SystemCoreClock = 8000000UL;

void SystemInit(void)
{
    /* 使能 HSI，并保持系统时钟来源为 HSI */
    RCC->CR |= 0x00000001UL;     /* HSION */
    while ((RCC->CR & 0x00000002UL) == 0) { } /* HSIRDY */

    RCC->CFGR &= ~0x00000003UL;  /* SW = 00, HSI selected as system clock */

    /* 使能 AFIO，关闭 JTAG 保留 SWD，避免部分 JTAG 引脚占用。
       本工程没有使用 PB3/PB4/PA15，但保留该设置一般不影响 ST-LINK 下载。 */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->MAPR &= ~(0x7UL << 24);
    AFIO->MAPR |=  (0x2UL << 24); /* SWJ_CFG = 010: JTAG-DP Disabled and SW-DP Enabled */
}

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 8000000UL;
}
