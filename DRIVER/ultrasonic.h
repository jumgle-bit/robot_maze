#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Ultrasonic_Init(void);
uint16_t Ultrasonic_ReadFrontCm(void);
uint8_t Ultrasonic_LastReadValid(void);
uint8_t Ultrasonic_LastError(void);
uint8_t Ultrasonic_IsFrontClear(uint16_t safe_cm);

#ifdef __cplusplus
}
#endif

#endif
