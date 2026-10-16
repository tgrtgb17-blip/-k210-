#ifndef __Planner_H
#define __Planner_H

#include "stm32f10x.h"

#define ARM_JOINT_NUM 5

typedef struct
{
    float start[ARM_JOINT_NUM];      // 插补起点

    float current[ARM_JOINT_NUM];    // 当前位置

    float target[ARM_JOINT_NUM];     // 目标位置

    uint16_t total_step;             // 总步数

    uint16_t current_step;           // 当前步数

    uint8_t running;
	
	uint8_t done_event; 

}ARM_INTERP_t;

void Planner_Start(ARM_INTERP_t *obj,float current[],float target[]);
uint8_t Planner_Run(ARM_INTERP_t *obj,float out[]);
void Planner_Reset(ARM_INTERP_t *obj);
#endif
