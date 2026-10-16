#include "stm32f10x.h"                  // Device header

void Serial_Init(uint8_t *address,uint16_t size)
{
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

GPIO_InitTypeDef GPIO_Instructure;	
GPIO_Instructure.GPIO_Mode = GPIO_Mode_AF_PP ;
GPIO_Instructure.GPIO_Pin = GPIO_Pin_9;
GPIO_Instructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA,&GPIO_Instructure);	

GPIO_Instructure.GPIO_Mode = GPIO_Mode_IPU ;
GPIO_Instructure.GPIO_Pin = GPIO_Pin_10;
GPIO_Instructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA,&GPIO_Instructure);	

USART_InitTypeDef USART_Instructure;	
USART_Instructure.USART_BaudRate = 115200;
USART_Instructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_Instructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
USART_Instructure.USART_Parity = USART_Parity_No;
USART_Instructure.USART_StopBits = USART_StopBits_1;
USART_Instructure.USART_WordLength = USART_WordLength_8b;
	
USART_Init(USART1,&USART_Instructure);	

USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);

USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);


NVIC_InitTypeDef NVIC_Instructure;
NVIC_Instructure.NVIC_IRQChannel = USART1_IRQn;
NVIC_Instructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Instructure.NVIC_IRQChannelPreemptionPriority = 5;
NVIC_Instructure.NVIC_IRQChannelSubPriority = 0;
NVIC_Init(&NVIC_Instructure);
	
DMA_InitTypeDef DMA_Instructure;
DMA_Instructure.DMA_BufferSize = size;
DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
DMA_Instructure.DMA_M2M = DMA_M2M_Disable;
DMA_Instructure.DMA_MemoryBaseAddr =(uint32_t) address;
DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
DMA_Instructure.DMA_PeripheralBaseAddr =(uint32_t)&USART1->DR;
DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; ;
DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
DMA_Instructure.DMA_Priority = DMA_Priority_Medium;
DMA_Init(DMA1_Channel5, &DMA_Instructure);

DMA_Cmd(DMA1_Channel5, ENABLE);	

USART_Cmd(USART1,ENABLE);
}


void Usart_receive(uint8_t *address,uint16_t size)
{
	DMA_Cmd(DMA1_Channel5, DISABLE);	
	DMA_SetCurrDataCounter(DMA1_Channel5,size);
	DMA_Cmd(DMA1_Channel5, ENABLE);	
}	

