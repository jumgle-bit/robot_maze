#ifndef __IR_SENSOR_H
#define __IR_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void IR_Init(void);
uint8_t IR_LeftBlocked(void);
uint8_t IR_RightBlocked(void);

#ifdef __cplusplus
}
#endif

#endif
