#include "stm32f10x.h"                  // Device header

void My_EXTI_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitTypeDef GPIO_Instructure;
	GPIO_Instructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_Instructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_Instructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Instructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);
	
	EXTI_InitTypeDef EXTI_Instructure;
	EXTI_Instructure.EXTI_Line = EXTI_Line10;
	EXTI_Instructure.EXTI_LineCmd = ENABLE;
	EXTI_Instructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_Instructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_Instructure);

	NVIC_InitTypeDef NVIC_Instructure;	
	NVIC_Instructure.NVIC_IRQChannel=EXTI15_10_IRQn;
	NVIC_Instructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Instructure.NVIC_IRQChannelPreemptionPriority=5;
	NVIC_Instructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_Instructure);

}	


