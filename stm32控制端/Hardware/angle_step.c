#include "stm32f10x.h"                  // Device header

uint8_t angle_step(float *target,float *current)
{
	uint8_t all_ok = 1;
	
	for(uint8_t i = 0; i < 4; i++)
	{
            if(target[i] - current[i] > 1)
            {
                current[i] += 3;
                all_ok = 0;
            }
            else if(current[i] - target[i] > 1)
            {
                current[i] -= 3;
                all_ok = 0;
            }
            else
            {
                current[i] = target[i];
            }
	}
return 	all_ok;
}
