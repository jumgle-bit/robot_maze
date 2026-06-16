#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化延时计时器。
void Delay_Init(void);
// 获取 16 位微秒计数。
uint16_t Delay_GetMicros16(void);
// 微秒级阻塞延时。
void Delay_us(uint16_t us);
// 毫秒级阻塞延时。
void Delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
