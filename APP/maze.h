#ifndef __MAZE_H
#define __MAZE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t front_cm;       // 前方超声波距离，单位 cm；无效时通常为 0。
    uint8_t front_valid;     // 前方距离是否有效，1 有效，0 表示本次超声波无效。
    uint8_t left_blocked;    // 左红外是否检测到障碍，1 有障碍，0 通畅。
    uint8_t right_blocked;   // 右红外是否检测到障碍，1 有障碍，0 通畅。
} SensorState_t;

// 初始化迷宫控制层内部状态。
void Maze_Init(void);
// 读取并返回当前传感器状态。
SensorState_t Maze_ReadSensorState(void);
// 立即输出一次串口状态行。
void Maze_SendStatusNow(void);
// 迷宫主任务，循环调用后按左手原则控制小车。
void Maze_Task(void);

#ifdef __cplusplus
}
#endif

#endif
