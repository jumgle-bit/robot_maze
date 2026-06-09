#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Motor_Init(void);
void Motor_SetLeft(int16_t speed);
void Motor_SetRight(int16_t speed);
void Motor_Stop(void);
int16_t Motor_GetLeftPwm(void);
int16_t Motor_GetRightPwm(void);

void Motor_Forward(uint16_t speed);
void Motor_Backward(uint16_t speed);
void Motor_SpinLeft(uint16_t speed);
void Motor_SpinRight(uint16_t speed);
void Motor_LeftTurn(uint16_t inner_speed, uint16_t outer_speed);
void Motor_RightTurn(uint16_t inner_speed, uint16_t outer_speed);

#ifdef __cplusplus
}
#endif

#endif
