//20141222
#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f10x.h"


// SPI总线速度设置 也就是设置预分频寄存器
#define SPI_SPEED_2   	0
#define SPI_SPEED_4   	1
#define SPI_SPEED_8  	2
#define SPI_SPEED_16 	3
#define SPI_SPEED_32   	4
#define SPI_SPEED_64   	5
#define SPI_SPEED_128  	6
#define SPI_SPEED_256 	7
			  	    													  
void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
void SPI2_Init(void);			 //初始化SPI口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
		 
#endif

