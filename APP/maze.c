#include "maze.h"
#include "config.h"
#include "delay.h"
#include "motor.h"
#include "ultrasonic.h"
#include "ir_sensor.h"
#include "uart.h"

/*
 * 串口输出节拍计数。
 * 说明：本工程没有使用 RTOS，因此在动作延时函数中分段计时，
 * 尽量保证 USART1 每 UART_STATUS_INTERVAL_MS 输出一次传感器和电机状态。
 */
#if UART_STATUS_ENABLE
static uint16_t g_uart_status_elapsed_ms = 0U;
static SensorState_t g_last_sensor_state = {0U, 0U, 0U, 0U};
static const char *g_action = "INIT";
#endif

void Maze_SetAction(const char *action)
{
#if UART_STATUS_ENABLE
    if (action != 0)
    {
        g_action = action;
    }
#else
    (void)action;
#endif
}

void Maze_SendStatusNow(void)
{
#if UART_STATUS_ENABLE
    UART1_SendRobotStatus(g_last_sensor_state.front_cm,
                          g_last_sensor_state.front_valid,
                          g_last_sensor_state.left_blocked,
                          g_last_sensor_state.right_blocked,
                          Motor_GetLeftPwm(),
                          Motor_GetRightPwm(),
                          g_action);
#endif
}

static void Maze_UARTStatusTask(uint16_t elapsed_ms)
{
#if UART_STATUS_ENABLE
    g_uart_status_elapsed_ms = (uint16_t)(g_uart_status_elapsed_ms + elapsed_ms);

    if (g_uart_status_elapsed_ms >= UART_STATUS_INTERVAL_MS)
    {
        Maze_SendStatusNow();
        g_uart_status_elapsed_ms = 0U;
    }
#else
    (void)elapsed_ms;
#endif
}

static void Maze_DelayWithUart(uint32_t ms)
{
    while (ms >= 10U)
    {
        Delay_ms(10U);
        Maze_UARTStatusTask(10U);
        ms -= 10U;
    }

    if (ms > 0U)
    {
        Delay_ms(ms);
        Maze_UARTStatusTask((uint16_t)ms);
    }
}

void Maze_Init(void)
{
    Maze_SetAction("STOP");
    Motor_Stop();
#if UART_STATUS_ENABLE
    g_uart_status_elapsed_ms = UART_STATUS_INTERVAL_MS;
#endif
}

/**
  * @brief 读取当前迷宫状态。
  * @note  输出形式：前方连续距离和有效状态 + 左右红外通断状态。
  */
SensorState_t Maze_ReadSensorState(void)
{
    SensorState_t s;

    s.front_cm = Ultrasonic_ReadFrontCm();
    s.front_valid = Ultrasonic_LastReadValid();
    s.left_blocked = IR_LeftBlocked();
    s.right_blocked = IR_RightBlocked();

#if UART_STATUS_ENABLE
    g_last_sensor_state = s;
    /* 开机首次决策立即输出，之后使用缓存状态，避免串口输出触发额外测距。 */
    if (g_uart_status_elapsed_ms >= UART_STATUS_INTERVAL_MS)
    {
        Maze_SendStatusNow();
        g_uart_status_elapsed_ms = 0U;
    }
#endif

    return s;
}

static void Maze_GoForwardByDistance(uint16_t ms)
{
    Maze_SetAction("FORWARD_STEP");
    Motor_Forward(MOTOR_FORWARD_PWM);
    Maze_DelayWithUart(ms);
    Maze_SetAction("STOP");
    Motor_Stop();
    Maze_DelayWithUart(80U);
}

static void Maze_PreTurnForwardIfClear(const SensorState_t *s)
{
    if (s->front_valid && s->front_cm > FRONT_LEFT_TURN_CLEAR_CM)
    {
        Maze_GoForwardByDistance(MAZE_PRE_TURN_FORWARD_MS);
    }
}

static void Maze_PostTurnForwardIfClear(void)
{
    SensorState_t s = Maze_ReadSensorState();

    if (s.front_valid && s.front_cm > FRONT_SAFE_DISTANCE_CM)
    {
        Maze_GoForwardByDistance(MAZE_POST_TURN_FORWARD_MS);
    }
}

static void Maze_Right90(const SensorState_t *s)
{
    Maze_SetAction("STOP_BEFORE_RIGHT");
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    /* 先前进一点，让车身中心进入路口，减少转弯撞角 */
    Maze_PreTurnForwardIfClear(s);

    /* 差速右转：左轮快、右轮慢，两个轮子都向前，不采用正反转原地转弯 */
    Maze_SetAction("RIGHT_TURN");
    Motor_RightTurn(MOTOR_TURN_INNER_PWM, MOTOR_TURN_OUTER_PWM);
    Maze_DelayWithUart(MAZE_TURN_90_MS);

    Maze_SetAction("STOP_AFTER_RIGHT");
    Motor_Stop();
    Maze_DelayWithUart(100U);

    /* 转向后向新通道内走一点，避免传感器还在路口边缘反复触发 */
    Maze_PostTurnForwardIfClear();
}

static void Maze_Left90(const SensorState_t *s)
{
    Maze_SetAction("STOP_BEFORE_LEFT");
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    Maze_PreTurnForwardIfClear(s);

    /* 差速左转：右轮快、左轮慢，两个轮子都向前，不采用正反转原地转弯 */
    Maze_SetAction("LEFT_TURN");
    Motor_LeftTurn(MOTOR_TURN_INNER_PWM, MOTOR_TURN_OUTER_PWM);
    Maze_DelayWithUart(MAZE_TURN_90_MS);

    Maze_SetAction("STOP_AFTER_LEFT");
    Motor_Stop();
    Maze_DelayWithUart(100U);

    Maze_PostTurnForwardIfClear();
}

static void Maze_TurnBack(void)
{
    Maze_SetAction("STOP_BEFORE_BACK");
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    /* 差速左转掉头：仍保持双轮向前，只靠左右轮速度差完成掉头 */
    Maze_SetAction("TURN_BACK");
    Motor_LeftTurn(MOTOR_TURN_BACK_INNER_PWM, MOTOR_TURN_BACK_OUTER_PWM);
    Maze_DelayWithUart(MAZE_TURN_BACK_MS);

    Maze_SetAction("STOP_AFTER_BACK");
    Motor_Stop();
    Maze_DelayWithUart(120U);

    Maze_PostTurnForwardIfClear();
}

/**
  * @brief 迷宫寻迹主任务。
  *
  * 本版修改点：
  * 1. 右红外管脚保持 PA11；
  * 2. 红外检测到障碍物为低电平；
  * 3. USART1 使用 PA9/PA10，每 0.3 s 输出一次传感器和电机 PWM 状态；
  * 4. 左转、右转、掉头均采用差速转弯，不采用左右轮正反转原地转弯；
  * 5. 算法由右手原则改为左手原则。
  *
  * 左手原则决策顺序：
  * 1. 左侧无障碍：优先左转；
  * 2. 左侧有障碍但前方安全：继续前进；
  * 3. 左侧和前方不可通行但右侧无障碍：右转；
  * 4. 左、前、右均不可通行：差速掉头。
  */
void Maze_Task(void)
{
    SensorState_t s = Maze_ReadSensorState();

    uint8_t front_safe = (s.front_valid && s.front_cm > FRONT_SAFE_DISTANCE_CM) ? 1U : 0U;

    if (!s.front_valid)
    {
        Maze_SetAction("US_FAULT_STOP");
        Motor_Stop();
        Maze_DelayWithUart(MAZE_SENSOR_FAULT_RETRY_MS);
        return;
    }

    /* 左手优先：左侧没有障碍时，优先进入左侧通道 */
    if (!s.left_blocked)
    {
        Maze_Left90(&s);
    }
    else if (front_safe)
    {
        if (s.front_cm < FRONT_SLOW_DISTANCE_CM)
        {
            Maze_SetAction("FORWARD_SLOW");
            Motor_Forward(MOTOR_SLOW_PWM);
        }
        else
        {
            Maze_SetAction("FORWARD");
            Motor_Forward(MOTOR_FORWARD_PWM);
        }

        Maze_DelayWithUart(MAZE_LOOP_DELAY_MS);
    }
    else if (!s.right_blocked)
    {
        Maze_Right90(&s);
    }
    else
    {
        Maze_TurnBack();
    }
}
