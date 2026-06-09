#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ULTRASONIC_STATUS_OK = 0,
    ULTRASONIC_STATUS_ECHO_IDLE_HIGH,
    ULTRASONIC_STATUS_NO_ECHO_RISE,
    ULTRASONIC_STATUS_ECHO_HIGH_TIMEOUT,
    ULTRASONIC_STATUS_TOO_CLOSE,
    ULTRASONIC_STATUS_TOO_FAR
} UltrasonicStatus_t;

void Ultrasonic_Init(void);
uint16_t Ultrasonic_ReadFrontCm(void);
uint8_t Ultrasonic_LastReadValid(void);
UltrasonicStatus_t Ultrasonic_GetLastStatus(void);
const char *Ultrasonic_StatusText(UltrasonicStatus_t status);
uint8_t Ultrasonic_IsFrontClear(uint16_t safe_cm);

#ifdef __cplusplus
}
#endif

#endif
