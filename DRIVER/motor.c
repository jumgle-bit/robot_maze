#include "motor.h"
#include "config.h"
#include "system_stm32f103.h"
#include "bsp_gpio.h"

/*
 * L298N 接线：
 * 左电机 Motor A：IN1->PB6，IN2->PB7，ENA->PA2(TIM2_CH3)
 * 右电机 Motor B：IN3->PB8，IN4->PB9，ENB->PA1(TIM2_CH2)
 *
 * 若实物左右相反，可只修改本文件上面的映射注释或 config.h 中的反向宏。
 */

#define LEFT_IN1_PORT    GPIOB
#define LEFT_IN1_PIN     6
#define LEFT_IN2_PORT    GPIOB
#define LEFT_IN2_PIN     7

#define RIGHT_IN1_PORT   GPIOB
#define RIGHT_IN1_PIN    8
#define RIGHT_IN2_PORT   GPIOB
#define RIGHT_IN2_PIN    9

static uint16_t Motor_LimitPwm(uint16_t pwm)
{
    return (pwm > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : pwm;
}

static int16_t g_left_pwm = 0;
static int16_t g_right_pwm = 0;

static uint16_t Motor_ApplyTrim(uint16_t pwm, int16_t trim)
{
    int32_t value = (int32_t)pwm + (int32_t)trim;

    if (pwm == 0U)
    {
        return 0U;
    }

    if (value < 0)
    {
        value = 0;
    }

    return Motor_LimitPwm((uint16_t)value);
}

static uint16_t Motor_SetLeftPwm(uint16_t pwm)
{
    uint16_t applied_pwm = Motor_ApplyTrim(pwm, MOTOR_LEFT_PWM_TRIM);
    TIM2->CCR3 = applied_pwm; /* PA2, TIM2_CH3, ENA */
    return applied_pwm;
}

static uint16_t Motor_SetRightPwm(uint16_t pwm)
{
    uint16_t applied_pwm = Motor_ApplyTrim(pwm, MOTOR_RIGHT_PWM_TRIM);
    TIM2->CCR2 = applied_pwm; /* PA1, TIM2_CH2, ENB */
    return applied_pwm;
}

static void Motor_SetLeftDir(int8_t dir)
{
#if MOTOR_LEFT_REVERSE
    dir = -dir;
#endif

    if (dir > 0)
    {
        GPIO_SetPin(LEFT_IN1_PORT, LEFT_IN1_PIN);
        GPIO_ResetPin(LEFT_IN2_PORT, LEFT_IN2_PIN);
    }
    else if (dir < 0)
    {
        GPIO_ResetPin(LEFT_IN1_PORT, LEFT_IN1_PIN);
        GPIO_SetPin(LEFT_IN2_PORT, LEFT_IN2_PIN);
    }
    else
    {
        GPIO_ResetPin(LEFT_IN1_PORT, LEFT_IN1_PIN);
        GPIO_ResetPin(LEFT_IN2_PORT, LEFT_IN2_PIN);
    }
}

static void Motor_SetRightDir(int8_t dir)
{
#if MOTOR_RIGHT_REVERSE
    dir = -dir;
#endif

    if (dir > 0)
    {
        GPIO_SetPin(RIGHT_IN1_PORT, RIGHT_IN1_PIN);
        GPIO_ResetPin(RIGHT_IN2_PORT, RIGHT_IN2_PIN);
    }
    else if (dir < 0)
    {
        GPIO_ResetPin(RIGHT_IN1_PORT, RIGHT_IN1_PIN);
        GPIO_SetPin(RIGHT_IN2_PORT, RIGHT_IN2_PIN);
    }
    else
    {
        GPIO_ResetPin(RIGHT_IN1_PORT, RIGHT_IN1_PIN);
        GPIO_ResetPin(RIGHT_IN2_PORT, RIGHT_IN2_PIN);
    }
}

void Motor_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* 方向控制引脚：PB6~PB9 推挽输出 */
    GPIO_ConfigPin(GPIOB, 6, GPIO_CNF_OUTPUT_PP_50MHZ);
    GPIO_ConfigPin(GPIOB, 7, GPIO_CNF_OUTPUT_PP_50MHZ);
    GPIO_ConfigPin(GPIOB, 8, GPIO_CNF_OUTPUT_PP_50MHZ);
    GPIO_ConfigPin(GPIOB, 9, GPIO_CNF_OUTPUT_PP_50MHZ);

    /* PWM 引脚：PA1=TIM2_CH2，PA2=TIM2_CH3，复用推挽输出 */
    GPIO_ConfigPin(GPIOA, 1, GPIO_CNF_AF_PP_50MHZ);
    GPIO_ConfigPin(GPIOA, 2, GPIO_CNF_AF_PP_50MHZ);

    /*
     * TIM2 PWM 频率：
     * SystemCoreClock=8MHz，PSC=7 -> 1MHz，ARR=999 -> 1kHz。
     */
    TIM2->PSC = (uint16_t)(SystemCoreClock / 1000000UL - 1UL);
    TIM2->ARR = MOTOR_PWM_MAX - 1;

    /* CH2 PWM1：OC2M=110，OC2PE=1 */
    TIM2->CCMR1 &= ~(0x7UL << 12);
    TIM2->CCMR1 |=  (0x6UL << 12) | (1UL << 11);

    /* CH3 PWM1：OC3M=110，OC3PE=1 */
    TIM2->CCMR2 &= ~(0x7UL << 4);
    TIM2->CCMR2 |=  (0x6UL << 4) | (1UL << 3);

    TIM2->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

    Motor_Stop();
}

void Motor_SetLeft(int16_t speed)
{
    if (speed > 0)
    {
        Motor_SetLeftDir(1);
        g_left_pwm = (int16_t)Motor_SetLeftPwm((uint16_t)speed);
    }
    else if (speed < 0)
    {
        Motor_SetLeftDir(-1);
        g_left_pwm = -(int16_t)Motor_SetLeftPwm((uint16_t)(-(int32_t)speed));
    }
    else
    {
        Motor_SetLeftDir(0);
        Motor_SetLeftPwm(0);
        g_left_pwm = 0;
    }
}

void Motor_SetRight(int16_t speed)
{
    if (speed > 0)
    {
        Motor_SetRightDir(1);
        g_right_pwm = (int16_t)Motor_SetRightPwm((uint16_t)speed);
    }
    else if (speed < 0)
    {
        Motor_SetRightDir(-1);
        g_right_pwm = -(int16_t)Motor_SetRightPwm((uint16_t)(-(int32_t)speed));
    }
    else
    {
        Motor_SetRightDir(0);
        Motor_SetRightPwm(0);
        g_right_pwm = 0;
    }
}

void Motor_Stop(void)
{
    Motor_SetLeft(0);
    Motor_SetRight(0);
}

int16_t Motor_GetLeftPwm(void)
{
    return g_left_pwm;
}

int16_t Motor_GetRightPwm(void)
{
    return g_right_pwm;
}

void Motor_Forward(uint16_t speed)
{
    Motor_SetLeft((int16_t)speed);
    Motor_SetRight((int16_t)speed);
}

void Motor_Backward(uint16_t speed)
{
    Motor_SetLeft(-(int16_t)speed);
    Motor_SetRight(-(int16_t)speed);
}

void Motor_SpinLeft(uint16_t speed)
{
    Motor_SetLeft(-(int16_t)speed);
    Motor_SetRight((int16_t)speed);
}

void Motor_SpinRight(uint16_t speed)
{
    Motor_SetLeft((int16_t)speed);
    Motor_SetRight(-(int16_t)speed);
}

/* 差速左转：左轮慢，右轮快 */
void Motor_LeftTurn(uint16_t inner_speed, uint16_t outer_speed)
{
    Motor_SetLeft((int16_t)inner_speed);
    Motor_SetRight((int16_t)outer_speed);
}

/* 差速右转：右轮慢，左轮快 */
void Motor_RightTurn(uint16_t inner_speed, uint16_t outer_speed)
{
    Motor_SetLeft((int16_t)outer_speed);
    Motor_SetRight((int16_t)inner_speed);
}
