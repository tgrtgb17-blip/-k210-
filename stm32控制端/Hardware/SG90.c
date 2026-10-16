#include "stm32f10x.h"                  // 舵机控制
#include "PWM.h"
#include "SG90.h"
#include "FreeRTOS.h"
#include "task.h"


static uint8_t j6_request = 0;   
static uint8_t j6_done_flag = 0; 
static TickType_t j6_tick = 0;

static uint16_t SG90_Map(float angle)
{
	if(angle < 0)
    angle = 0;

	if(angle > 180)
    angle = 180;

   return (uint16_t)(angle / 180.0f * 2000.0f + 500.0f);
}

void SG90_init(void)
{
    PWM_Init();

    j6_request = 0;
    j6_done_flag = 0;
    j6_tick = 0;

    PWM_Set(6, SG90_Map(0));
}

void SG90_set_angle(float *angle)
{
    static uint16_t last_ccr[5] = {0};

    for(int i = 0; i < 5; i++)
    {
        uint16_t ccr = SG90_Map(angle[i]);

        if(ccr != last_ccr[i])
        {
            PWM_Set(i + 1, ccr);
            last_ccr[i] = ccr;
        }
    }
}

void SG90_j6_start(uint8_t mode)
{
    if(mode == 0) 
    {
        PWM_Set(6, SG90_Map(0));
    }
    else if(mode == 1) 
    {
        PWM_Set(6, SG90_Map(80));
    }

    j6_request = 1;
    j6_done_flag = 0;
    j6_tick = xTaskGetTickCount();
}

uint8_t SG90_j6_done(void)
{
    if(j6_request == 0)
        return 0;

    if(xTaskGetTickCount() - j6_tick >= pdMS_TO_TICKS(300))
    {
        j6_request = 0;
        j6_done_flag = 1;
        return 1;
    }

    return 0;
}
