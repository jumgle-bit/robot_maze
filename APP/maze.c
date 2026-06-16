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
#endif

#define MAZE_STATE_FORWARD    0U
#define MAZE_STATE_TURN_LEFT  1U
#define MAZE_STATE_TURN_RIGHT 2U
#define MAZE_STATE_TURN_BACK  3U

static uint8_t g_maze_state = MAZE_STATE_FORWARD;
static uint16_t g_state_elapsed_ms = 0U;
static uint8_t g_finish_confirm_count = 0U;

void Maze_SendStatusNow(void)
{
#if UART_STATUS_ENABLE
    UART1_SendRobotStatus(g_last_sensor_state.front_cm,
                          g_last_sensor_state.front_valid,
                          g_last_sensor_state.left_blocked,
                          g_last_sensor_state.right_blocked,
                          Motor_GetLeftPwm(),
                          Motor_GetRightPwm());
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
    Motor_Stop();
    g_maze_state = MAZE_STATE_FORWARD;
    g_state_elapsed_ms = 0U;
    g_finish_confirm_count = 0U;
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

static uint8_t Maze_IsFrontSafe(const SensorState_t *s, uint16_t safe_cm)
{
    return (s->front_valid && s->front_cm > safe_cm) ? 1U : 0U;
}

static void Maze_SetState(uint8_t state)
{
    g_maze_state = state;
    g_state_elapsed_ms = 0U;
    g_finish_confirm_count = 0U;
}

static uint8_t Maze_ConfirmFinish(uint8_t condition)
{
    if (condition)
    {
        if (g_finish_confirm_count < MAZE_IR_CONFIRM_COUNT)
        {
            g_finish_confirm_count++;
        }
    }
    else
    {
        g_finish_confirm_count = 0U;
    }

    return (g_finish_confirm_count >= MAZE_IR_CONFIRM_COUNT) ? 1U : 0U;
}

static void Maze_StopFor(uint16_t ms)
{
    Motor_Stop();
    Maze_DelayWithUart(ms);
}

static uint16_t Maze_LimitStep(uint16_t elapsed_ms, uint16_t max_ms, uint16_t step_ms)
{
    uint16_t remain_ms = (elapsed_ms < max_ms) ? (uint16_t)(max_ms - elapsed_ms) : 0U;

    if (remain_ms == 0U)
    {
        return 0U;
    }

    return (remain_ms > step_ms) ? step_ms : remain_ms;
}

static void Maze_ForwardOneLoop(const SensorState_t *s)
{
    if (s->front_cm < FRONT_SLOW_DISTANCE_CM)
    {
        Motor_Forward(MOTOR_SLOW_PWM);
    }
    else
    {
        Motor_Forward(MOTOR_FORWARD_PWM);
    }

    Maze_DelayWithUart(MAZE_LOOP_DELAY_MS);
}

static uint8_t Maze_GoForwardChecked(uint16_t ms, uint16_t pwm, uint16_t safe_cm)
{
    while (ms > 0U)
    {
        uint16_t step_ms;
        SensorState_t s = Maze_ReadSensorState();

        if (!Maze_IsFrontSafe(&s, safe_cm))
        {
            Maze_StopFor(80U);
            return 0U;
        }

        step_ms = (ms > MAZE_FORWARD_CHECK_STEP_MS) ? MAZE_FORWARD_CHECK_STEP_MS : ms;
        Motor_Forward(pwm);
        Maze_DelayWithUart(step_ms);
        Motor_Stop();

        if (ms > step_ms)
        {
            Maze_DelayWithUart(MAZE_FORWARD_CHECK_PAUSE_MS);
        }

        ms = (uint16_t)(ms - step_ms);
    }

    Maze_StopFor(80U);
    return 1U;
}

static void Maze_PostTurnForwardIfClear(void)
{
    SensorState_t s = Maze_ReadSensorState();

    if (Maze_IsFrontSafe(&s, FRONT_SAFE_DISTANCE_CM))
    {
        (void)Maze_GoForwardChecked(MAZE_POST_TURN_FORWARD_MS,
                                    MOTOR_FORWARD_PWM,
                                    FRONT_SAFE_DISTANCE_CM);
    }
}

static void Maze_BeginTurn(uint8_t state, const SensorState_t *s)
{
    (void)s;
    Maze_StopFor(MAZE_STOP_BEFORE_TURN_MS);
    Maze_SetState(state);
}

static void Maze_FinishTurn(void)
{
    Maze_StopFor(MAZE_TURN_STEP_PAUSE_MS);
    Maze_SetState(MAZE_STATE_FORWARD);
    Maze_PostTurnForwardIfClear();
}

static uint8_t Maze_ForwardIfBothSideLost(const SensorState_t *s)
{
    if (!s->left_blocked &&
        !s->right_blocked &&
        Maze_IsFrontSafe(s, FRONT_SAFE_DISTANCE_CM))
    {
        uint16_t step_ms = Maze_LimitStep(g_state_elapsed_ms,
                                          MAZE_TURN_MAX_MS,
                                          MAZE_TURN_LOST_FORWARD_MS);

        if (step_ms == 0U)
        {
            return 0U;
        }

        Motor_Forward(MOTOR_TURN_LOST_PWM);
        Maze_DelayWithUart(step_ms);
        Motor_Stop();
        g_state_elapsed_ms = (uint16_t)(g_state_elapsed_ms + step_ms);
        Maze_DelayWithUart(MAZE_TURN_STEP_PAUSE_MS);
        return 1U;
    }

    return 0U;
}

static void Maze_RunLeftTurn(const SensorState_t *s)
{
    uint16_t step_ms;

    if ((g_state_elapsed_ms >= MAZE_TURN_MIN_MS) &&
        Maze_ConfirmFinish(s->left_blocked))
    {
        Maze_FinishTurn();
        return;
    }

    if (g_state_elapsed_ms >= MAZE_TURN_MAX_MS)
    {
        Maze_FinishTurn();
        return;
    }

    if (Maze_ForwardIfBothSideLost(s))
    {
        return;
    }

    step_ms = Maze_LimitStep(g_state_elapsed_ms, MAZE_TURN_MAX_MS, MAZE_TURN_STEP_MS);
    Motor_LeftTurn(MOTOR_TURN_INNER_PWM, MOTOR_TURN_OUTER_PWM);
    Maze_DelayWithUart(step_ms);
    Motor_Stop();
    g_state_elapsed_ms = (uint16_t)(g_state_elapsed_ms + step_ms);
    Maze_DelayWithUart(MAZE_TURN_STEP_PAUSE_MS);
}

static void Maze_RunRightTurn(const SensorState_t *s)
{
    uint16_t step_ms;

    if ((g_state_elapsed_ms >= MAZE_TURN_MIN_MS) &&
        Maze_ConfirmFinish(s->right_blocked))
    {
        Maze_FinishTurn();
        return;
    }

    if (g_state_elapsed_ms >= MAZE_TURN_MAX_MS)
    {
        Maze_FinishTurn();
        return;
    }

    if (Maze_ForwardIfBothSideLost(s))
    {
        return;
    }

    step_ms = Maze_LimitStep(g_state_elapsed_ms, MAZE_TURN_MAX_MS, MAZE_TURN_STEP_MS);
    Motor_RightTurn(MOTOR_TURN_INNER_PWM, MOTOR_TURN_OUTER_PWM);
    Maze_DelayWithUart(step_ms);
    Motor_Stop();
    g_state_elapsed_ms = (uint16_t)(g_state_elapsed_ms + step_ms);
    Maze_DelayWithUart(MAZE_TURN_STEP_PAUSE_MS);
}

static void Maze_RunTurnBack(const SensorState_t *s)
{
    uint16_t step_ms;

    if ((g_state_elapsed_ms >= MAZE_TURN_BACK_MIN_MS) &&
        Maze_ConfirmFinish(Maze_IsFrontSafe(s, FRONT_TURN_BACK_CLEAR_CM)))
    {
        Maze_FinishTurn();
        return;
    }

    if (g_state_elapsed_ms >= MAZE_TURN_BACK_MAX_MS)
    {
        Maze_FinishTurn();
        return;
    }

    step_ms = Maze_LimitStep(g_state_elapsed_ms,
                             MAZE_TURN_BACK_MAX_MS,
                             MAZE_TURN_BACK_STEP_MS);
    Motor_SpinLeft(MOTOR_TURN_BACK_SPIN_PWM);
    Maze_DelayWithUart(step_ms);
    Motor_Stop();
    g_state_elapsed_ms = (uint16_t)(g_state_elapsed_ms + step_ms);
    Maze_DelayWithUart(MAZE_TURN_BACK_CHECK_PAUSE_MS);
}

static void Maze_RunMotionState(const SensorState_t *s)
{
    if (g_maze_state == MAZE_STATE_TURN_LEFT)
    {
        Maze_RunLeftTurn(s);
    }
    else if (g_maze_state == MAZE_STATE_TURN_RIGHT)
    {
        Maze_RunRightTurn(s);
    }
    else if (g_maze_state == MAZE_STATE_TURN_BACK)
    {
        Maze_RunTurnBack(s);
    }
}

/**
  * @brief 迷宫寻迹主任务。
  *
  * 当前算法为显式状态机：
  * 1. 普通前进状态按左手原则选择下一动作；
  * 2. 一旦进入左转、右转或掉头状态，动作完成前不重新执行路口决策；
  * 3. 左右转达到最小时间后，等待对应侧红外连续检测到墙再退出；
  * 4. 若传感器没有确认完成，则达到最大时间后兜底退出；
  * 5. 掉头达到最小时间后，前方超声波连续恢复安全距离即退出。
  */
void Maze_Task(void)
{
    SensorState_t s = Maze_ReadSensorState();

    if (g_maze_state != MAZE_STATE_FORWARD)
    {
        Maze_RunMotionState(&s);
        return;
    }

    if (!s.front_valid)
    {
        Maze_StopFor(MAZE_SENSOR_FAULT_RETRY_MS);
        return;
    }

    if (!s.left_blocked)
    {
        Maze_BeginTurn(MAZE_STATE_TURN_LEFT, &s);
    }
    else if (Maze_IsFrontSafe(&s, FRONT_SAFE_DISTANCE_CM))
    {
        Maze_ForwardOneLoop(&s);
    }
    else if (!s.right_blocked)
    {
        Maze_BeginTurn(MAZE_STATE_TURN_RIGHT, &s);
    }
    else
    {
        Maze_BeginTurn(MAZE_STATE_TURN_BACK, &s);
    }
}
