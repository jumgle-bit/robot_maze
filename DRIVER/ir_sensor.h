#ifndef __IR_SENSOR_H
#define __IR_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化左右红外输入引脚。
void IR_Init(void);
// 判断左侧是否有障碍。
uint8_t IR_LeftBlocked(void);
// 判断右侧是否有障碍。
uint8_t IR_RightBlocked(void);

#ifdef __cplusplus
}
#endif

#endif
