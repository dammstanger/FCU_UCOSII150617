
/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：usart.h
 * 描	述	：stm32串口通信驱动
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.2.150418
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.24
 * 最后编辑	：2015.4.18
 * 备	注	：
 **-------------------------------------------------------------------------------

 * 作	者		：Damm Stanger
 * 邮	箱		：dammstanger@qq.com
**********************************************************************************************/

#ifndef USART_H
#define USART_H

#include "Project_cfg.h"
#include "stm32f10x.h"
#include <stdio.h>

#define UART4_DEBUG

extern u8 ReceiveData[];
extern u8 DebugData[20];


void USART1_NVIC_Configuration(void);
void DMA1CH5_NVIC_Configuration(void);
void USART1_Config(int BR);
void UART4_Config(int BR);
void USART1_DMAConfig(void);
void USART1_DMAConfig_Sim(void);
void UART4_NVIC_Configuration(void);
void USART1_EnableRevDMA(void);

void USART1_SendData(unsigned char dat);
void USART1_SendStr(unsigned char *str,unsigned char len);

void SendDebugDat_SSCOM32(int dat1,int dat2,int dat3,int dat4);	//发送匹配SSCOM32的数据
void SendDebugDat_Hunter(int dat1,int dat2,int dat3,int dat4);//发送匹配串口猎人的数据

void USARTPIDDebug(u8 *dat);

int fputc(int ch, FILE *f);

#endif
