#include "stm32f10x.h"                  // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "Key.h"
#include "LED.h"
#include "OLED.h"
#include "Serial.h"
#include "SG90.h"
#include "PWM.h"
#include "ik.h"
#include "EXTI.h"
#include "Planner.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define USART_len 10

QueueHandle_t Set_angle;
QueueHandle_t USART_R;
QueueHandle_t Mount;

SemaphoreHandle_t sg90_mutex;
SemaphoreHandle_t angle;
SemaphoreHandle_t j_mutex;
SemaphoreHandle_t current_mutex;
ARM_INTERP_t arm_interp;
ARM_INTERP_t mount_interp;

volatile TickType_t last_tick = 0;
volatile uint8_t flag_1 = 0;
uint8_t USART_ch[USART_len];
uint8_t Receive_data_flag=0;
uint8_t Receive[USART_len];
float send[2];
float j_out[5];
uint8_t color;
uint8_t Run = 0;
float out[5];
static float current[5] = {82,115,0,0,180};
static float start_place[5] = {82,115,0,0,180};
typedef enum
{
	FOLLOW,
	HELD,

}Arm_state;

volatile Arm_state arm_state = FOLLOW;

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line10) != RESET)
    {
        TickType_t now = xTaskGetTickCountFromISR();

        if((now - last_tick) > pdMS_TO_TICKS(30))
        {
            last_tick = now;

            Run = !Run;
			
			OLED_Clear();	
        }

        EXTI_ClearITPendingBit(EXTI_Line10);
    }
}


void USART1_IRQHandler(void)
{	
	volatile int tem;
	BaseType_t xPartmant = pdFALSE;
	if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET)
	{
	tem = USART1->SR;
	tem = USART1->DR;
	DMA_Cmd(DMA1_Channel5, DISABLE);		
	xQueueSendFromISR(USART_R,USART_ch,&xPartmant);	
	DMA_SetCurrDataCounter(DMA1_Channel5, USART_len);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	portYIELD_FROM_ISR(xPartmant);
	}
}	


void Task_OLED(void *p)
{
    float local[5];
    while(1)
    {
        xSemaphoreTake(j_mutex,portMAX_DELAY);
        memcpy(local,j_out,sizeof(local));
        xSemaphoreGive(j_mutex);
		if(Run == 1)
		{	
		
		OLED_ShowString(1,1,"j1:");
		OLED_ShowNum(1,4,(uint32_t)local[0],3);

		OLED_ShowString(2,1,"j2:");
		OLED_ShowNum(2,4,(uint32_t)local[1],3);

		OLED_ShowString(3,1,"j3:");
		OLED_ShowNum(3,4,(uint32_t)local[2],3);

		OLED_ShowString(4,1,"j4:");
		OLED_ShowNum(4,4,(uint32_t)local[3],3);

        vTaskDelay(pdMS_TO_TICKS(200));
		}
		else 
		{
		OLED_ShowString(1,1,"off");
		}
    }
}

void Task_UA(void *p)
{	
	while(1)
	{
		if(xQueueReceive(USART_R,Receive,portMAX_DELAY))
		{	
			if(Receive[0] == 0x31 && Receive[1] == 0x51 && Receive[9] == 0x5B)
			{			
			color = Receive[2];
			int16_t cx,cy,angle_r;
			memcpy(&cx, &Receive[3], 2);
			memcpy(&cy, &Receive[5], 2);
			memcpy(&angle_r, &Receive[7], 2);
			float cx_f = (float)cx;
			float cy_f = (float)cy;
			float angle_rf = (float)angle_r;
			
				float temp[5];	

				ik_get_angle(cx_f,cy_f,angle_rf,temp);
				
				xSemaphoreTake(j_mutex,portMAX_DELAY);
				memcpy(j_out,temp,sizeof(temp));
				xSemaphoreGive(j_mutex);
				if(arm_state == FOLLOW)
			{		
				xQueueOverwrite(Set_angle,temp);				
			}
			vTaskDelay(pdMS_TO_TICKS(50));
			}		
		}
	}
}
void Task_SET_SG90(void *p) 
{ 
float target[5] = {82,115,0,0,180};
float last_target[5];
memset(&arm_interp,0,sizeof(arm_interp)); 
xSemaphoreTake(sg90_mutex, portMAX_DELAY); 
SG90_set_angle(current); 
xSemaphoreGive(sg90_mutex); 


while(1) 
{ 			
	float new_target[5]; 
	while(xQueueReceive(Set_angle,new_target,0) == pdPASS) 
	{ 
		memcpy(target, new_target, sizeof(target));
	} 
	if(arm_state != HELD)
	{
	xSemaphoreTake(current_mutex, portMAX_DELAY);
	if(arm_interp.running == 0) 
		{ 
			uint8_t need_move = 0; 
			
			for(uint8_t i=0;i<5;i++) 
			{ 
				if(fabsf(target[i] - start_place[i]) > 0.01f)
				{
				need_move = 1;
				break;
				}
			} 
			if(need_move) 
			{ 
			Planner_Start(&arm_interp,current,target);
			} 
			xSemaphoreGive(current_mutex);
		} 
	if(arm_interp.running) 
			{
				Planner_Run(&arm_interp,out); 
				memcpy(current, out, sizeof(out));
				xSemaphoreGive(current_mutex); 
				if(arm_state == FOLLOW && Run) 
				{ 
					xSemaphoreTake(sg90_mutex,portMAX_DELAY); 
					SG90_set_angle(current); 
					xSemaphoreGive(sg90_mutex); 
				} 
			}
	if(arm_interp.done_event == 1 && flag_1 == 0) 
		{ 
			arm_interp.done_event = 0; 
			vTaskDelay(pdMS_TO_TICKS(50)); 
			SG90_j6_start(1); 
			flag_1 = 1; 
		} 
		if(flag_1 == 1 && SG90_j6_done()) 
		{ 
		xQueueSend(Mount, current, portMAX_DELAY); 
		flag_1 = 2; 
		} 
		
	}
	vTaskDelay(pdMS_TO_TICKS(20)); 
	} 
}
void Task_Mount(void *p)
{
    float Mount_target[5];
    memset(&mount_interp,0,sizeof(mount_interp));

    while(1)
    {
        xQueueReceive(Mount,Mount_target,portMAX_DELAY);
        arm_state = HELD;	
		xQueueReset(Set_angle);
		Planner_Reset(&arm_interp);
		Mount_target[1]= 120;
		Mount_target[2]= 90;
		Mount_target[3]= 0;
		Mount_target[4]= 180;
		xSemaphoreTake(current_mutex, portMAX_DELAY);
		Planner_Start(&mount_interp,current,Mount_target);
        
		while(mount_interp.running)
        {
            Planner_Run(&mount_interp,out);
			memcpy(current,out,sizeof(out));
			xSemaphoreTake(sg90_mutex,portMAX_DELAY);
			SG90_set_angle(current);
			xSemaphoreGive(sg90_mutex);	
			vTaskDelay(pdMS_TO_TICKS(20));
        }
		vTaskDelay(pdMS_TO_TICKS(200)); 
		if(color == 'R')Mount_target[0] = 170;
		if(color == 'B')Mount_target[0] = 10;
        Planner_Start(&mount_interp,current,Mount_target);
        while(mount_interp.running)
        {
            Planner_Run(&mount_interp,out);
			memcpy(current,out,sizeof(out));
			xSemaphoreTake(sg90_mutex,portMAX_DELAY);
			SG90_set_angle(current);
			xSemaphoreGive(sg90_mutex);	
            vTaskDelay(pdMS_TO_TICKS(20));
        }
		SG90_j6_start(0);

		while(!SG90_j6_done())
		{
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		Planner_Start(&mount_interp,current,start_place);
        while(mount_interp.running)
        {
            Planner_Run(&mount_interp,out);
			memcpy(current,out,sizeof(out));
			xSemaphoreTake(sg90_mutex,portMAX_DELAY);
			SG90_set_angle(current);
			xSemaphoreGive(sg90_mutex);	
            vTaskDelay(pdMS_TO_TICKS(20));
        }
		flag_1 = 0;
		Planner_Reset(&arm_interp);
		memcpy(current, start_place, sizeof(start_place));
		xQueueOverwrite(Set_angle, current);
		vTaskDelay(pdMS_TO_TICKS(100));
		
		Planner_Reset(&arm_interp);

		memcpy(current,start_place,sizeof(start_place));
		xSemaphoreGive(current_mutex); 
		arm_state = FOLLOW;
	}
}
int main(void)
{	
	My_EXTI_Init();
	OLED_Init();	
	PWM_Init();
	SG90_init();
	Serial_Init(USART_ch,USART_len);
	USART_R = xQueueCreate(5,sizeof(Receive));
	angle = xSemaphoreCreateBinary();
	Set_angle = xQueueCreate(1,sizeof(float)*5);
	Mount = xQueueCreate(5,sizeof(float)*5);
	j_mutex = xSemaphoreCreateMutex();
	current_mutex = xSemaphoreCreateMutex();
	sg90_mutex = xSemaphoreCreateMutex();
	xTaskCreate(Task_UA,"chu",128,NULL,3,NULL);
	xTaskCreate(Task_SET_SG90,"angle",128,NULL,8,NULL);
	xTaskCreate(Task_OLED,"OLED",128,NULL,3,NULL);
	xTaskCreate(Task_Mount,"mount",128,NULL,7,NULL);	
	vTaskStartScheduler();
	
    while(1)
	{}
}
