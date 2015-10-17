/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名		：SysTick.c
 * 描述    ：SysTick 系统滴答时钟10us中断函数库,中断时间可自由配置，
 *           常用的有 1us 10us 1ms 中断。         
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.1.150408
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本		：ST3.0.0
 * 创建时间	：2014.8.31
 * 最后编辑	：2015.6.13
 * 备			注：
 **-------------------------------------------------------------------------------

 * 作	者		：Damm Stanger
 * 邮	箱		：dammstanger@qq.com
**********************************************************************************************/

#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"

#define UCOSII_RUN			1u

extern uint32_t g_Systime;

#if !UCOSII_RUN

void SysTick_Init(void);
//void Delay_us(__IO u32 nTime);
#endif

void delay_nus(u32 n);
void Delay_ms(__IO u32 nTime);
void get_ms(unsigned long  *time);
void TimingDelay_Decrement(void);
uint32_t GetSystemTime(void);
#endif /* __SYSTICK_H */
