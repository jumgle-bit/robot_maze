#ifndef __UART_H
#define __UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void UART1_Init(uint32_t baudrate);
void UART1_SendChar(char ch);
void UART1_SendString(const char *str);
void UART1_SendDistanceCm(uint16_t cm);
void UART1_SendRobotStatus(uint16_t cm,
                           uint8_t ultrasonic_valid,
                           const char *ultrasonic_error,
                           uint8_t left_blocked,
                           uint8_t right_blocked,
                           int16_t left_pwm,
                           int16_t right_pwm,
                           const char *action);

#ifdef __cplusplus
}
#endif

#endif
