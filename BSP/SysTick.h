/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���		��SysTick.c
 * ����    ��SysTick ϵͳ�δ�ʱ��10us�жϺ�����,�ж�ʱ����������ã�
 *           ���õ��� 1us 10us 1ms �жϡ�         
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.1.150408
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾		��ST3.0.0
 * ����ʱ��	��2014.8.31
 * ���༭	��2015.6.13
 * ��			ע��
 **-------------------------------------------------------------------------------

 * ��	��		��Damm Stanger
 * ��	��		��dammstanger@qq.com
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
