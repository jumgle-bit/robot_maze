#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Delay_Init(void);
uint16_t Delay_GetMicros16(void);
void Delay_us(uint16_t us);
void Delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
