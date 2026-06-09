#include "uart.h"
#include "config.h"
#include "system_stm32f103.h"
#include "bsp_gpio.h"

/*
 * USART1 串口：PA9 = TX，PA10 = RX。
 * 当前用于实时发送超声波、左右红外和左右电机 PWM 状态。
 */

#if UART_STATUS_ENABLE
static void UART1_SendUint(uint16_t value)
{
    char buf[6];
    int8_t i = 0;

    if (value == 0)
    {
        UART1_SendChar('0');
        return;
    }

    while ((value > 0) && (i < (int8_t)sizeof(buf)))
    {
        buf[i++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (i > 0)
    {
        UART1_SendChar(buf[--i]);
    }
}

static void UART1_SendSignedPwm(int16_t value)
{
    if (value > 0)
    {
        UART1_SendChar('+');
        UART1_SendUint((uint16_t)value);
    }
    else if (value < 0)
    {
        UART1_SendChar('-');
        UART1_SendUint((uint16_t)(-(int32_t)value));
    }
    else
    {
        UART1_SendChar('0');
    }
}
#endif

void UART1_Init(uint32_t baudrate)
{
    uint32_t brr;

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;

    /* PA9  USART1_TX：复用推挽输出；PA10 USART1_RX：浮空输入 */
    GPIO_ConfigPin(GPIOA, 9, GPIO_CNF_AF_PP_50MHZ);
    GPIO_ConfigPin(GPIOA, 10, GPIO_CNF_INPUT_FLOATING);

    /* 8 MHz HSI 下，115200 的 BRR 约为 0x45；这里按 SystemCoreClock 自动计算。 */
    if (baudrate == 0U)
    {
        baudrate = 115200U;
    }

    brr = (SystemCoreClock + (baudrate / 2U)) / baudrate;
    USART1->BRR = brr;

    /* 8位数据位，无校验，1位停止位，开启收发和 USART */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;
    USART1->CR2 = 0x00000000U;
    USART1->CR3 = 0x00000000U;
    USART1->CR1 |= USART_CR1_UE;
}

void UART1_SendChar(char ch)
{
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
        /* wait */
    }

    USART1->DR = (uint16_t)ch;
}

void UART1_SendString(const char *str)
{
    while ((str != 0) && (*str != '\0'))
    {
        UART1_SendChar(*str++);
    }
}

void UART1_SendDistanceCm(uint16_t cm)
{
#if UART_STATUS_ENABLE
    UART1_SendString("US=");
    UART1_SendUint(cm);
    UART1_SendString("cm\r\n");
#else
    (void)cm;
#endif
}

void UART1_SendRobotStatus(uint16_t cm,
                           uint8_t ultrasonic_valid,
                           const char *ultrasonic_error,
                           uint8_t left_blocked,
                           uint8_t right_blocked,
                           int16_t left_pwm,
                           int16_t right_pwm,
                           const char *action)
{
#if UART_STATUS_ENABLE
    UART1_SendString("US=");
    if (ultrasonic_valid)
    {
        UART1_SendUint(cm);
        UART1_SendString("cm,US_OK=1");
    }
    else
    {
        UART1_SendString("INVALID,US_OK=0");
    }

    UART1_SendString(",US_ERR=");
    UART1_SendString(ultrasonic_error);
    UART1_SendString(",IR_L=");
    UART1_SendString(left_blocked ? "BLOCK" : "CLEAR");
    UART1_SendString(",IR_R=");
    UART1_SendString(right_blocked ? "BLOCK" : "CLEAR");

    UART1_SendString(",PWM_L=");
    UART1_SendSignedPwm(left_pwm);
    UART1_SendString(",PWM_R=");
    UART1_SendSignedPwm(right_pwm);
    UART1_SendString(",ACT=");
    UART1_SendString(action);
    UART1_SendString("\r\n");
#else
    (void)cm;
    (void)ultrasonic_valid;
    (void)ultrasonic_error;
    (void)left_blocked;
    (void)right_blocked;
    (void)left_pwm;
    (void)right_pwm;
    (void)action;
#endif
}
