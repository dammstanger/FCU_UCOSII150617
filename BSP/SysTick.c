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
#include "SysTick.h"
#include "I2C_1.h"


static __IO u32 TimingDelay;

uint32_t g_Systime = 0;

/*
 * ��������SysTick_Init
 * ����  ������ϵͳ�δ�ʱ�� SysTick
 * ����  ����
 * ���  ����
 * ����  ���ⲿ���� 
 */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	if (SysTick_Config(SystemFrequency / 1000))
  { 
    while (1);
  }
}

/*
 * ��������Delay_ms
 * ����  ��ms��ʱ����,1msΪһ����λ
 * ����  ��- nTime
 * ���  ����
 * ����  ��Delay_ms( 1 ) ��ʵ�ֵ���ʱΪ 1 * 1ms = 1ms
 *       ���ⲿ���� 
 */

//void Delay_ms(__IO u32 nTime)
//{ 
//  TimingDelay = nTime;

//  while(TimingDelay != 0);
//}

void delay_nus(u32 n)
{
	u8 j;
	while(n--)
	for(j=0;j<10;j++);
}

void Delay_ms(__IO u32 nTime)
{
	while(nTime--)
	delay_nus(1000);
}


/*
 * ��������TimingDelay_Decrement
 * ����  ����ȡ���ĳ���
 * ����  ����
 * ���  ����
 * ����  ���� SysTick �жϺ��� SysTick_Handler()����
 */  
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}


void get_ms(unsigned long *time)
{
	;
}

uint32_t GetSystemTime()
{
	return g_Systime;
}


/******************* (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� *****END OF FILE****/
