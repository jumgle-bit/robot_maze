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
static SensorState_t g_last_sensor_state = {0U, 0U, ULTRASONIC_STATUS_NO_ECHO_RISE, 0U, 0U};
static const char *g_action = "INIT";
#endif
static uint8_t g_turn_fault = 0U;

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
                          Ultrasonic_StatusText(g_last_sensor_state.front_status),
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
    g_turn_fault = 0U;
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
    s.front_status = Ultrasonic_GetLastStatus();
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

typedef enum
{
    MAZE_TURN_LEFT = 0,
    MAZE_TURN_RIGHT,
    MAZE_TURN_BACK
} MazeTurnDirection_t;

static uint16_t Maze_AbsDiff(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint8_t Maze_TurnReferenceChanged(const SensorState_t *initial,
                                         const SensorState_t *current)
{
    if (initial->left_blocked != current->left_blocked ||
        initial->right_blocked != current->right_blocked ||
        initial->front_valid != current->front_valid)
    {
        return 1U;
    }

    if (initial->front_valid && current->front_valid &&
        Maze_AbsDiff(initial->front_cm, current->front_cm) >= TURN_FEEDBACK_DISTANCE_CHANGE_CM)
    {
        return 1U;
    }

    return 0U;
}

static uint8_t Maze_TurnTargetReached(MazeTurnDirection_t direction,
                                      const SensorState_t *current)
{
    uint8_t front_safe =
        ((current->front_valid && current->front_cm > FRONT_SAFE_DISTANCE_CM) ||
         current->front_status == ULTRASONIC_STATUS_TOO_FAR) ? 1U : 0U;

    if (!front_safe)
    {
        return 0U;
    }

    if (direction == MAZE_TURN_LEFT)
    {
        return current->right_blocked;
    }

    if (direction == MAZE_TURN_RIGHT)
    {
        return current->left_blocked;
    }

    return (current->left_blocked || current->right_blocked) ? 1U : 0U;
}

static uint8_t Maze_RunFeedbackTurn(MazeTurnDirection_t direction,
                                    const SensorState_t *initial)
{
    uint8_t sample;
    uint8_t reference_changed = 0U;
    uint8_t stable_count = 0U;
    uint8_t invalid_count = 0U;

    for (sample = 0U; sample < TURN_FEEDBACK_MAX_SAMPLES; ++sample)
    {
        SensorState_t current;

        if (direction == MAZE_TURN_RIGHT)
        {
            Maze_SetAction("RIGHT_TURN_CREEP");
            Motor_SpinRight(MOTOR_FEEDBACK_TURN_PWM);
        }
        else
        {
            Maze_SetAction((direction == MAZE_TURN_LEFT)
                           ? "LEFT_TURN_CREEP"
                           : "BACK_TURN_CREEP");
            Motor_SpinLeft(MOTOR_FEEDBACK_TURN_PWM);
        }

        Maze_DelayWithUart(TURN_CREEP_PULSE_MS);
        Motor_Stop();
        Maze_DelayWithUart(TURN_CREEP_SETTLE_MS);
        current = Maze_ReadSensorState();

        if (!current.front_valid &&
            current.front_status != ULTRASONIC_STATUS_TOO_FAR)
        {
            ++invalid_count;
            if (invalid_count >= TURN_FEEDBACK_MAX_INVALID_SAMPLES)
            {
                break;
            }
        }
        else
        {
            invalid_count = 0U;
        }

        if (Maze_TurnReferenceChanged(initial, &current))
        {
            reference_changed = 1U;
        }

        if (reference_changed && Maze_TurnTargetReached(direction, &current))
        {
            stable_count = 1U;

            while (stable_count < TURN_FEEDBACK_STABLE_SAMPLES)
            {
                Maze_DelayWithUart(TURN_CREEP_SETTLE_MS);
                current = Maze_ReadSensorState();

                if (!Maze_TurnTargetReached(direction, &current))
                {
                    stable_count = 0U;
                    break;
                }

                ++stable_count;
            }

            if (stable_count >= TURN_FEEDBACK_STABLE_SAMPLES)
            {
                Maze_SetAction("TURN_ALIGNED");
                Motor_Stop();
                Maze_SendStatusNow();
                return 1U;
            }
        }

        Maze_SendStatusNow();
    }

    Maze_SetAction("TURN_FEEDBACK_FAIL");
    Motor_Stop();
    g_turn_fault = 1U;
    Maze_SendStatusNow();
    return 0U;
}

static void Maze_Right90(const SensorState_t *s)
{
    SensorState_t initial;

    Maze_SetAction("STOP_BEFORE_RIGHT");
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    Maze_PreTurnForwardIfClear(s);

    initial = Maze_ReadSensorState();
    if (Maze_RunFeedbackTurn(MAZE_TURN_RIGHT, &initial))
    {
        Maze_PostTurnForwardIfClear();
    }
}

static void Maze_Left90(const SensorState_t *s)
{
    SensorState_t initial;

    Maze_SetAction("STOP_BEFORE_LEFT");
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    Maze_PreTurnForwardIfClear(s);

    initial = Maze_ReadSensorState();
    if (Maze_RunFeedbackTurn(MAZE_TURN_LEFT, &initial))
    {
        Maze_PostTurnForwardIfClear();
    }
}

static void Maze_TurnBack(void)
{
    Maze_SetAction("STOP_BEFORE_BACK");
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    {
        SensorState_t initial = Maze_ReadSensorState();
        if (Maze_RunFeedbackTurn(MAZE_TURN_BACK, &initial))
        {
            Maze_PostTurnForwardIfClear();
        }
    }
}

/**
  * @brief 迷宫寻迹主任务。
  *
  * 本版修改点：
  * 1. 右红外管脚保持 PA11；
  * 2. 红外检测到障碍物为低电平；
  * 3. USART1 使用 PA9/PA10，每 0.3 s 输出一次传感器和电机 PWM 状态；
  * 4. 左转、右转、掉头采用传感器闭环原地转向，固定时间仅作为故障保护；
  * 5. 算法由右手原则改为左手原则。
  *
  * 左手原则决策顺序：
  * 1. 左侧无障碍：优先左转；
  * 2. 左侧有障碍但前方安全：继续前进；
  * 3. 左侧和前方不可通行但右侧无障碍：右转；
  * 4. 左、前、右均不可通行：闭环原地掉头。
  */
void Maze_Task(void)
{
    SensorState_t s = Maze_ReadSensorState();

    uint8_t front_safe = (s.front_valid && s.front_cm > FRONT_SAFE_DISTANCE_CM) ? 1U : 0U;

    if (g_turn_fault)
    {
        Maze_SetAction("TURN_FEEDBACK_FAIL");
        Motor_Stop();
        Maze_DelayWithUart(MAZE_SENSOR_FAULT_RETRY_MS);
        return;
    }

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
