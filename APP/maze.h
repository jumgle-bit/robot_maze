#ifndef __MAZE_H
#define __MAZE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t front_cm;
    uint8_t front_valid;
    uint8_t left_blocked;
    uint8_t right_blocked;
} SensorState_t;

void Maze_Init(void);
SensorState_t Maze_ReadSensorState(void);
void Maze_SendStatusNow(void);
void Maze_Task(void);

#ifdef __cplusplus
}
#endif

#endif
