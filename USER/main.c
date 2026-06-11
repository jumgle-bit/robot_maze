/**
  ******************************************************************************
  * @file    main.c
  * @brief   STM32F103C8T6 迷宫机器人主程序
  *
  * 硬件依据：
  * 1. 前向 HC-SR04：TRIG->PB14，ECHO->PB15（PB15 为 FT 数字输入，可选分压保护）
  * 2. 左红外：PA12，右红外：PA11
  * 3. L298N：PB6~PB9 控制 IN1~IN4，PA1/PA2 输出 PWM 使能
  *
  * 正式运行模式为迷宫寻迹；红外检测到障碍物为低电平；
  * 90°转弯和死路掉头采用低速脉冲式差速转弯；
  * USART1 每 0.3s 输出一次传感器和电机 PWM 状态；迷宫算法采用左手原则。
  ******************************************************************************
  */

#include "system_stm32f103.h"
#include "delay.h"
#include "motor.h"
#include "ultrasonic.h"
#include "ir_sensor.h"
#include "uart.h"
#include "maze.h"
#include "config.h"

int main(void)
{
    /* SystemInit() 已在启动文件中执行，这里初始化项目外设 */
    Delay_Init();
    Motor_Init();
    Ultrasonic_Init();
    IR_Init();
    UART1_Init(UART1_BAUDRATE);
    UART1_SendString("MazeRobot status: US, US_OK, US_ERR, IR_L, IR_R, PWM_L, PWM_R\r\n");
    Maze_Init();

    Delay_ms(500);

    while (1)
    {
        Maze_Task();
    }
}
