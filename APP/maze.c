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

#define MAZE_TURN_LOCK_NONE  0U
#define MAZE_TURN_LOCK_LEFT  1U
#define MAZE_TURN_LOCK_RIGHT 2U

static uint8_t g_turn_lock_side = MAZE_TURN_LOCK_NONE;

// 立即通过 USART1 输出一次最近缓存的传感器和电机 PWM 状态。
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

// 由延时函数周期调用，累计到设定间隔后自动输出一次运行状态。
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

// 带串口状态刷新能力的毫秒延时，避免长动作期间串口完全不输出。
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

// 初始化迷宫控制层状态，确保电机停止并清除转弯锁定。
void Maze_Init(void)
{
    Motor_Stop();
    g_turn_lock_side = MAZE_TURN_LOCK_NONE;
#if UART_STATUS_ENABLE
    g_uart_status_elapsed_ms = UART_STATUS_INTERVAL_MS;
#endif
}

/**
  * @brief 读取当前迷宫状态。
  * @note  输出形式：前方连续距离和有效状态 + 左右红外通断状态。
  */
// 读取一次前方超声波、左右红外，并更新串口输出使用的缓存状态。
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

// 判断前方距离是否有效且大于给定安全阈值。
static uint8_t Maze_IsFrontSafe(const SensorState_t *s, uint16_t safe_cm)
{
    return (s->front_valid && s->front_cm > safe_cm) ? 1U : 0U;
}

// 转弯后锁定触发转弯的那一侧，防止车还在路口边缘时反复转弯。
static void Maze_StartTurnLock(uint8_t side)
{
    g_turn_lock_side = side;
}

// 清除转弯锁定，允许下一次按左手原则重新判断路口。
static void Maze_ClearTurnLock(void)
{
    g_turn_lock_side = MAZE_TURN_LOCK_NONE;
}

// 检查当前被锁定的一侧是否已经重新检测到墙。
static uint8_t Maze_TurnLockSideBlocked(const SensorState_t *s)
{
    if (g_turn_lock_side == MAZE_TURN_LOCK_LEFT)
    {
        return s->left_blocked;
    }

    if (g_turn_lock_side == MAZE_TURN_LOCK_RIGHT)
    {
        return s->right_blocked;
    }

    return 1U;
}

// 执行一个主循环周期的直行，根据前方距离选择正常速度或低速。
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

// 分段前进并持续检测前方安全距离，发现危险立即停车返回失败。
static uint8_t Maze_GoForwardChecked(uint16_t ms, uint16_t pwm, uint16_t safe_cm)
{
    while (ms > 0U)
    {
        uint16_t step_ms;
        SensorState_t s = Maze_ReadSensorState();

        // 前方障碍物检测，不安全则停止并返回
        if (!Maze_IsFrontSafe(&s, safe_cm))
        {
            Motor_Stop();
            Maze_DelayWithUart(80U);
            return 0U;
        }

        // 前进一段后停止，分段执行以实现周期性检测
        step_ms = (ms > MAZE_FORWARD_CHECK_STEP_MS) ? MAZE_FORWARD_CHECK_STEP_MS : ms;
        Motor_Forward(pwm);
        Maze_DelayWithUart(step_ms);
        Motor_Stop();

        // 多段前进时插入暂停间隔
        if (ms > step_ms)
        {
            Maze_DelayWithUart(MAZE_FORWARD_CHECK_PAUSE_MS);
        }

        ms = (uint16_t)(ms - step_ms);
    }

    Motor_Stop();
    Maze_DelayWithUart(80U);
    return 1U;
}

// 如果前方仍安全，在转弯前低速往前挪一小段，让车身进入路口。
static void Maze_DelayBeforeTurnIfClear(const SensorState_t *s)
{
    if (Maze_IsFrontSafe(s, FRONT_SAFE_DISTANCE_CM))
    {
        (void)Maze_GoForwardChecked(MAZE_TURN_DELAY_FORWARD_MS,
                                    MOTOR_TURN_DELAY_PWM,
                                    FRONT_SAFE_DISTANCE_CM);
    }
}

// 转弯完成后，如果前方安全则短距离前进，减少在路口重复触发。
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

// 将左转拆成多个短脉冲动作，便于蠕动式修正角度。
static void Maze_PulsedLeftTurn(uint16_t total_ms, uint16_t inner_pwm, uint16_t outer_pwm)
{
    while (total_ms > 0U)
    {
        uint16_t step_ms = (total_ms > MAZE_TURN_STEP_MS) ? MAZE_TURN_STEP_MS : total_ms;

        Motor_LeftTurn(inner_pwm, outer_pwm);
        Maze_DelayWithUart(step_ms);
        Motor_Stop();

        total_ms = (uint16_t)(total_ms - step_ms);

        if (total_ms > 0U)
        {
            Maze_DelayWithUart(MAZE_TURN_STEP_PAUSE_MS);
        }
    }
}

// 将右转拆成多个短脉冲动作，和左转一样采用内轮慢、外轮快。
static void Maze_PulsedRightTurn(uint16_t total_ms, uint16_t inner_pwm, uint16_t outer_pwm)
{
    while (total_ms > 0U)
    {
        uint16_t step_ms = (total_ms > MAZE_TURN_STEP_MS) ? MAZE_TURN_STEP_MS : total_ms;

        Motor_RightTurn(inner_pwm, outer_pwm);
        Maze_DelayWithUart(step_ms);
        Motor_Stop();

        total_ms = (uint16_t)(total_ms - step_ms);

        if (total_ms > 0U)
        {
            Maze_DelayWithUart(MAZE_TURN_STEP_PAUSE_MS);
        }
    }
}

// 死路时分段左旋掉头，每小段后检测前方是否已经重新变安全。
static uint8_t Maze_TurnBackUntilFrontClear(void)
{
    uint16_t elapsed_ms = 0U;

    while (elapsed_ms < MAZE_TURN_BACK_MAX_MS)
    {
        uint16_t step_ms = (uint16_t)(MAZE_TURN_BACK_MAX_MS - elapsed_ms);
        SensorState_t s;

        if (step_ms > MAZE_TURN_BACK_STEP_MS)
        {
            step_ms = MAZE_TURN_BACK_STEP_MS;
        }

        Motor_SpinLeft(MOTOR_TURN_BACK_SPIN_PWM);
        Maze_DelayWithUart(step_ms);
        Motor_Stop();

        elapsed_ms = (uint16_t)(elapsed_ms + step_ms);
        Maze_DelayWithUart(MAZE_TURN_BACK_CHECK_PAUSE_MS);

        s = Maze_ReadSensorState();
        if (Maze_IsFrontSafe(&s, FRONT_TURN_BACK_CLEAR_CM))
        {
            return 1U;
        }
    }

    return 0U;
}

// 执行一次右 90 度差速转弯，并在完成后进入右侧转弯锁定。
static void Maze_Right90(const SensorState_t *s)
{
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    /* 检测到需要转弯后，先低速前进一小段，让车身更深入路口再转。 */
    Maze_DelayBeforeTurnIfClear(s);

    /* 差速右转：左轮快、右轮慢，保留侧墙参考，避免原地甩头。 */
    Maze_PulsedRightTurn(MAZE_TURN_90_MS,
                         MOTOR_TURN_INNER_PWM,
                         MOTOR_TURN_OUTER_PWM);

    Motor_Stop();
    Maze_DelayWithUart(100U);
    Maze_StartTurnLock(MAZE_TURN_LOCK_RIGHT);

    /* 转向后向新通道内走一点，避免传感器还在路口边缘反复触发 */
    Maze_PostTurnForwardIfClear();
}

// 执行一次左 90 度差速转弯，并在完成后进入左侧转弯锁定。
static void Maze_Left90(const SensorState_t *s)
{
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    Maze_DelayBeforeTurnIfClear(s);

    /* 差速左转：右轮快、左轮慢，保留侧墙参考，避免原地甩头。 */
    Maze_PulsedLeftTurn(MAZE_TURN_90_MS,
                        MOTOR_TURN_INNER_PWM,
                        MOTOR_TURN_OUTER_PWM);

    Motor_Stop();
    Maze_DelayWithUart(100U);
    Maze_StartTurnLock(MAZE_TURN_LOCK_LEFT);

    Maze_PostTurnForwardIfClear();
}

// 当前方、左侧、右侧都不可通行时执行掉头动作。
static void Maze_TurnBack(void)
{
    Motor_Stop();
    Maze_DelayWithUart(MAZE_STOP_BEFORE_TURN_MS);

    /*
     * 原地左旋掉头：左轮反转、右轮正转。
     * 每小段动作后重新测距，前方恢复到 FRONT_TURN_BACK_CLEAR_CM 以上就退出。
     */
    (void)Maze_TurnBackUntilFrontClear();

    Motor_Stop();
    Maze_DelayWithUart(120U);
    Maze_StartTurnLock(MAZE_TURN_LOCK_LEFT);

    Maze_PostTurnForwardIfClear();
}

/**
  * @brief 迷宫寻迹主任务。
  *
  * 本版修改点：
  * 1. 右红外管脚保持 PA11；
  * 2. 红外检测到障碍物为低电平；
  * 3. USART1 使用 PA9/PA10，每 0.3 s 输出一次传感器和电机 PWM 状态；
  * 4. 左转、右转、掉头均采用差速转弯，避免原地甩头导致红外丢墙；
  * 5. 算法由右手原则改为左手原则；
  * 6. 转弯后锁定触发转弯的一侧，直到该侧红外重新检测到墙后才允许新转弯。
  *
  * 左手原则决策顺序：
  * 1. 左侧无障碍：优先左转；
  * 2. 左侧有障碍但前方安全：继续前进；
  * 3. 左侧和前方不可通行但右侧无障碍：右转；
  * 4. 左、前、右均不可通行：原地左旋掉头。
  */
// 主迷宫任务：按左手原则决定左转、直行、右转或掉头。
void Maze_Task(void)
{
    SensorState_t s = Maze_ReadSensorState();

    uint8_t front_safe = Maze_IsFrontSafe(&s, FRONT_SAFE_DISTANCE_CM);

    if (!s.front_valid)
    {
        Motor_Stop();
        Maze_ClearTurnLock();
        Maze_DelayWithUart(MAZE_SENSOR_FAULT_RETRY_MS);
        return;
    }

    if (g_turn_lock_side != MAZE_TURN_LOCK_NONE)
    {
        if (Maze_TurnLockSideBlocked(&s))
        {
            Maze_ClearTurnLock();
            Motor_Stop();
            Maze_DelayWithUart(MAZE_FORWARD_CHECK_PAUSE_MS);
        }
        else
        {
            if (g_turn_lock_side == MAZE_TURN_LOCK_LEFT)
            {
                Maze_PulsedLeftTurn(MAZE_TURN_STEP_MS,
                                    MOTOR_TURN_INNER_PWM,
                                    MOTOR_TURN_OUTER_PWM);
                Motor_Stop();
                Maze_DelayWithUart(MAZE_TURN_STEP_PAUSE_MS);
            }
            else
            {
                Maze_PulsedRightTurn(MAZE_TURN_STEP_MS,
                                     MOTOR_TURN_INNER_PWM,
                                     MOTOR_TURN_OUTER_PWM);
                Motor_Stop();
                Maze_DelayWithUart(MAZE_TURN_STEP_PAUSE_MS);
            }
        }

        return;
    }

    /* 左手优先：左侧没有障碍时，优先进入左侧通道 */
    if (!s.left_blocked)
    {
        Maze_Left90(&s);
    }
    else if (front_safe)
    {
        Maze_ForwardOneLoop(&s);
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
