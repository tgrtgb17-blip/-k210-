#include "Planner.h"    //插补算法
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

void Planner_Start(ARM_INTERP_t *obj,float current[],float target[])
{
    float max_delta = 0;

    obj->current_step = 0;

    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        obj->start[i]   = current[i];
        obj->current[i] = current[i];
        obj->target[i]  = target[i];

        float delta = fabsf(target[i] - current[i]);

        if(delta > max_delta)
        {
            max_delta = delta;
        }
    }


    obj->total_step = (uint16_t)(max_delta / 0.8f);

    if(obj->total_step < 20)
    {
        obj->total_step = 20;
    }

    if(obj->total_step > 300)
    {
        obj->total_step = 300;
    }

    obj->running = 1;
}


uint8_t Planner_Run(ARM_INTERP_t *obj,float out[])
{
    if(obj->running == 0)
    {
        memcpy(out,obj->target,sizeof(float) * ARM_JOINT_NUM);
        return 1;
    }

    float s =(float)obj->current_step /(float)obj->total_step;

    if(s > 1.0f)
    {
        s = 1.0f;
    }

    float s2 = s * s;
    float s3 = s2 * s;
    float s4 = s3 * s;
    float s5 = s4 * s;
    float ratio =10.0f * s3- 15.0f * s4+  6.0f * s5;

    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        out[i] =obj->start[i]+(obj->target[i] - obj->start[i]) * ratio;
        obj->current[i] = out[i];
    }
    obj->current_step++;

    if(obj->current_step >= obj->total_step)
    {
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            obj->current[i] = obj->target[i];
            out[i] = obj->target[i];
        }
		obj->done_event = 1;
        obj->running = 0;

        return 1;
    }
	
    return 0;
	
}
void Planner_Reset(ARM_INTERP_t *obj)
{
    obj->running = 0;
    obj->done_event = 0;
    obj->current_step = 0;
}

