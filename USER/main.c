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
  * 默认运行模式为迷宫寻迹；红外检测到障碍物为低电平；
  * 转弯采用差速转弯，不采用左右轮正反转原地转弯；
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

#if (APP_RUN_MODE == APP_MODE_MOTOR_TEST)
static void App_TestMotor(void);
#elif (APP_RUN_MODE == APP_MODE_SENSOR_TEST)
static void App_TestSensorByMotion(void);
#endif

int main(void)
{
    /* SystemInit() 已在启动文件中执行，这里初始化项目外设 */
    Delay_Init();
    Motor_Init();
    Ultrasonic_Init();
    IR_Init();
    UART1_Init(UART1_BAUDRATE);
    UART1_SendString("MazeRobot status: US, US_OK, US_ERR, IR_L, IR_R, PWM_L, PWM_R, ACT\r\n");
    Maze_Init();

    Delay_ms(500);

#if (APP_RUN_MODE == APP_MODE_MOTOR_TEST)
    App_TestMotor();
#elif (APP_RUN_MODE == APP_MODE_SENSOR_TEST)
    App_TestSensorByMotion();
#else
    while (1)
    {
        Maze_Task();
    }
#endif
}

/**
  * @brief 电机单模块测试：前进、后退、差速左转、差速右转、停止。
  *        首次上车建议先把 APP_RUN_MODE 改为 APP_MODE_MOTOR_TEST。
  */
#if (APP_RUN_MODE == APP_MODE_MOTOR_TEST)
static void App_TestMotor(void)
{
    while (1)
    {
        (void)Maze_ReadSensorState();

        Maze_SetAction("TEST_FORWARD");
        Motor_Forward(MOTOR_TEST_FORWARD_PWM);
        Maze_SendStatusNow();
        Delay_ms(1500);

        Maze_SetAction("TEST_STOP");
        Motor_Stop();
        Maze_SendStatusNow();
        Delay_ms(500);

        Maze_SetAction("TEST_BACKWARD");
        Motor_Backward(MOTOR_TEST_FORWARD_PWM);
        Maze_SendStatusNow();
        Delay_ms(1000);

        Maze_SetAction("TEST_STOP");
        Motor_Stop();
        Maze_SendStatusNow();
        Delay_ms(500);

        Maze_SetAction("TEST_LEFT");
        Motor_LeftTurn(MOTOR_TEST_TURN_INNER_PWM, MOTOR_TEST_TURN_OUTER_PWM);
        Maze_SendStatusNow();
        Delay_ms(900);

        Maze_SetAction("TEST_STOP");
        Motor_Stop();
        Maze_SendStatusNow();
        Delay_ms(500);

        Maze_SetAction("TEST_RIGHT");
        Motor_RightTurn(MOTOR_TEST_TURN_INNER_PWM, MOTOR_TEST_TURN_OUTER_PWM);
        Maze_SendStatusNow();
        Delay_ms(900);

        Maze_SetAction("TEST_STOP");
        Motor_Stop();
        Maze_SendStatusNow();
        Delay_ms(1500);
    }
}
#endif

/**
  * @brief 简单传感器联动测试：
  *        左侧空旷则差速左转；
  *        否则按前方安全、右侧空旷、差速掉头顺序执行。
  */
#if (APP_RUN_MODE == APP_MODE_SENSOR_TEST)
static void App_TestSensorByMotion(void)
{
    while (1)
    {
        SensorState_t s = Maze_ReadSensorState();

        if (!s.front_valid)
        {
            Maze_SetAction("TEST_US_FAULT");
            Motor_Stop();
            Delay_ms(MAZE_SENSOR_FAULT_RETRY_MS);
        }
        else if (!s.left_blocked)
        {
            Maze_SetAction("TEST_LEFT");
            Motor_LeftTurn(MOTOR_TEST_TURN_INNER_PWM, MOTOR_TEST_TURN_OUTER_PWM);
            Delay_ms(450);
        }
        else if (s.front_cm > FRONT_SAFE_DISTANCE_CM)
        {
            Maze_SetAction("TEST_FORWARD");
            Motor_Forward(MOTOR_TEST_FORWARD_PWM);
            Delay_ms(250);
        }
        else if (!s.right_blocked)
        {
            Maze_SetAction("TEST_RIGHT");
            Motor_RightTurn(MOTOR_TEST_TURN_INNER_PWM, MOTOR_TEST_TURN_OUTER_PWM);
            Delay_ms(450);
        }
        else
        {
            Maze_SetAction("TEST_BACK");
            Motor_LeftTurn(MOTOR_TEST_TURN_INNER_PWM, MOTOR_TEST_TURN_OUTER_PWM);
            Delay_ms(900);
        }

        Maze_SendStatusNow();
        Motor_Stop();
        Delay_ms(150);
    }
}
#endif
