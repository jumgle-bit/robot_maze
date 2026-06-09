#include "ultrasonic.h"
#include "config.h"
#include "delay.h"
#include "bsp_gpio.h"
#include "system_stm32f103.h"

#define US_TRIG_PORT       GPIOB
#define US_TRIG_PIN        14
#define US_ECHO_PORT       GPIOB
#define US_ECHO_PIN        15

#if (ULTRASONIC_TRIGGER_INTERVAL_MS == 0U) || (ULTRASONIC_TRIGGER_INTERVAL_MS > 65U)
#error "ULTRASONIC_TRIGGER_INTERVAL_MS must be in the range 1..65"
#endif

#if (ULTRASONIC_RETRY_COUNT == 0U)
#error "ULTRASONIC_RETRY_COUNT must be at least 1"
#endif

static uint8_t g_last_read_valid = 0U;
static uint8_t g_has_triggered = 0U;
static uint16_t g_last_trigger_us = 0U;
static UltrasonicStatus_t g_last_status = ULTRASONIC_STATUS_NO_ECHO_RISE;

static uint8_t Ultrasonic_WaitEchoLevel(uint8_t expected_level, uint16_t timeout_us)
{
    uint16_t start = Delay_GetMicros16();

    while (GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) != expected_level)
    {
        if ((uint16_t)(Delay_GetMicros16() - start) >= timeout_us)
        {
            return 0U;
        }
    }

    return 1U;
}

static void Ultrasonic_WaitForTriggerInterval(void)
{
    if (g_has_triggered)
    {
        while ((uint16_t)(Delay_GetMicros16() - g_last_trigger_us) <
               (uint16_t)(ULTRASONIC_TRIGGER_INTERVAL_MS * 1000U))
        {
            /* HC-SR04 requires enough time between trigger pulses. */
        }
    }
}

void Ultrasonic_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    GPIO_ConfigPin(US_TRIG_PORT, US_TRIG_PIN, GPIO_CNF_OUTPUT_PP_50MHZ);
    GPIO_ConfigPin(US_ECHO_PORT, US_ECHO_PIN, GPIO_CNF_INPUT_PULL);

    GPIO_ResetPin(US_TRIG_PORT, US_TRIG_PIN);
    GPIO_ResetPin(US_ECHO_PORT, US_ECHO_PIN); /* 输入下拉：断线时避免浮空产生虚假距离 */
    g_last_read_valid = 0U;
    g_has_triggered = 0U;
    g_last_status = ULTRASONIC_STATUS_NO_ECHO_RISE;
}

static uint16_t Ultrasonic_ReadRawCm(void)
{
    uint16_t start;
    uint16_t width_us;
    uint16_t cm;

    Ultrasonic_WaitForTriggerInterval();

    /* 正常情况下触发前 ECHO 必须处于低电平，否则本次读数无效。 */
    if (!Ultrasonic_WaitEchoLevel(0U, ULTRASONIC_ECHO_IDLE_TIMEOUT_US))
    {
        g_last_status = ULTRASONIC_STATUS_ECHO_IDLE_HIGH;
        return 0U;
    }

    /* TRIG 触发：至少 10 us 高电平 */
    g_last_trigger_us = Delay_GetMicros16();
    g_has_triggered = 1U;
    GPIO_ResetPin(US_TRIG_PORT, US_TRIG_PIN);
    Delay_us(2);
    GPIO_SetPin(US_TRIG_PORT, US_TRIG_PIN);
    Delay_us(12);
    GPIO_ResetPin(US_TRIG_PORT, US_TRIG_PIN);

    /* 等待 ECHO 拉高 */
    if (!Ultrasonic_WaitEchoLevel(1U, ULTRASONIC_TIMEOUT_US))
    {
        g_last_status = ULTRASONIC_STATUS_NO_ECHO_RISE;
        return 0U;
    }

    /* 测量 ECHO 高电平宽度 */
    start = Delay_GetMicros16();
    while (GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == 1)
    {
        if ((uint16_t)(Delay_GetMicros16() - start) > ULTRASONIC_TIMEOUT_US)
        {
            g_last_status = ULTRASONIC_STATUS_ECHO_HIGH_TIMEOUT;
            return 0U;
        }
    }

    width_us = (uint16_t)(Delay_GetMicros16() - start);

    /* HC-SR04 常用换算：距离(cm) = 高电平时间(us) / 58 */
    cm = (uint16_t)(width_us / 58U);
    if (cm < ULTRASONIC_MIN_CM)
    {
        g_last_status = ULTRASONIC_STATUS_TOO_CLOSE;
    }
    else if (cm > ULTRASONIC_MAX_CM)
    {
        g_last_status = ULTRASONIC_STATUS_TOO_FAR;
    }
    else
    {
        g_last_status = ULTRASONIC_STATUS_OK;
    }
    return cm;
}

uint16_t Ultrasonic_ReadFrontCm(void)
{
    uint8_t attempt;
    uint16_t cm;

    for (attempt = 0U; attempt < ULTRASONIC_RETRY_COUNT; ++attempt)
    {
        cm = Ultrasonic_ReadRawCm();

        if (cm >= ULTRASONIC_MIN_CM && cm <= ULTRASONIC_MAX_CM)
        {
            g_last_read_valid = 1U;
            g_last_status = ULTRASONIC_STATUS_OK;
            return cm;
        }

    }

    g_last_read_valid = 0U;
    return 0U;
}

uint8_t Ultrasonic_LastReadValid(void)
{
    return g_last_read_valid;
}

UltrasonicStatus_t Ultrasonic_GetLastStatus(void)
{
    return g_last_status;
}

const char *Ultrasonic_StatusText(UltrasonicStatus_t status)
{
    switch (status)
    {
        case ULTRASONIC_STATUS_OK:
            return "NONE";
        case ULTRASONIC_STATUS_ECHO_IDLE_HIGH:
            return "ECHO_IDLE_HIGH";
        case ULTRASONIC_STATUS_NO_ECHO_RISE:
            return "NO_ECHO_RISE";
        case ULTRASONIC_STATUS_ECHO_HIGH_TIMEOUT:
            return "ECHO_HIGH_TIMEOUT";
        case ULTRASONIC_STATUS_TOO_CLOSE:
            return "TOO_CLOSE";
        case ULTRASONIC_STATUS_TOO_FAR:
            return "TOO_FAR";
        default:
            return "UNKNOWN";
    }
}

uint8_t Ultrasonic_IsFrontClear(uint16_t safe_cm)
{
    return (Ultrasonic_ReadFrontCm() > safe_cm) ? 1U : 0U;
}
