#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 HC-SR04：TRIG=PB14，ECHO=PB15。
void Ultrasonic_Init(void);
// 读取前方距离，单位 cm；无效时返回 0。
uint16_t Ultrasonic_ReadFrontCm(void);
// 最近一次测距是否有效。
uint8_t Ultrasonic_LastReadValid(void);
// 最近一次测距错误码，用于定位 INVALID 原因。
uint8_t Ultrasonic_LastError(void);
// 判断前方距离是否大于 safe_cm。
uint8_t Ultrasonic_IsFrontClear(uint16_t safe_cm);

#ifdef __cplusplus
}
#endif

#endif
