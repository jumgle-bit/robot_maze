#ifndef __UART_H
#define __UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 USART1，默认用于状态输出。
void UART1_Init(uint32_t baudrate);
// 发送一个字符。
void UART1_SendChar(char ch);
// 发送字符串。
void UART1_SendString(const char *str);
// 单独输出超声波距离。
void UART1_SendDistanceCm(uint16_t cm);
// 输出完整状态行：超声波、红外、电机 PWM。
void UART1_SendRobotStatus(uint16_t cm,
                           uint8_t ultrasonic_valid,
                           uint8_t left_blocked,
                           uint8_t right_blocked,
                           int16_t left_pwm,
                           int16_t right_pwm);

#ifdef __cplusplus
}
#endif

#endif
