#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化电机方向引脚和 PWM 定时器。
void Motor_Init(void);
// 设置左电机带符号速度，正数前进、负数后退。
void Motor_SetLeft(int16_t speed);
// 设置右电机带符号速度，正数前进、负数后退。
void Motor_SetRight(int16_t speed);
// 停止左右电机。
void Motor_Stop(void);
// 读取左电机当前带符号 PWM。
int16_t Motor_GetLeftPwm(void);
// 读取右电机当前带符号 PWM。
int16_t Motor_GetRightPwm(void);

// 左右轮同速前进。
void Motor_Forward(uint16_t speed);
// 左右轮同速后退。
void Motor_Backward(uint16_t speed);
// 原地左旋。
void Motor_SpinLeft(uint16_t speed);
// 原地右旋。
void Motor_SpinRight(uint16_t speed);
// 差速左转，inner_speed 为内轮速度，outer_speed 为外轮速度。
void Motor_LeftTurn(uint16_t inner_speed, uint16_t outer_speed);
// 差速右转，inner_speed 为内轮速度，outer_speed 为外轮速度。
void Motor_RightTurn(uint16_t inner_speed, uint16_t outer_speed);

#ifdef __cplusplus
}
#endif

#endif
