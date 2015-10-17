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
#include "SysTick.h"
#include "I2C_1.h"


static __IO u32 TimingDelay;

uint32_t g_Systime = 0;

/*
 * 函数名：SysTick_Init
 * 描述  ：启动系统滴答定时器 SysTick
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用 
 */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
	if (SysTick_Config(SystemFrequency / 1000))
  { 
    while (1);
  }
}

/*
 * 函数名：Delay_ms
 * 描述  ：ms延时程序,1ms为一个单位
 * 输入  ：- nTime
 * 输出  ：无
 * 调用  ：Delay_ms( 1 ) 则实现的延时为 1 * 1ms = 1ms
 *       ：外部调用 
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
 * 函数名：TimingDelay_Decrement
 * 描述  ：获取节拍程序
 * 输入  ：无
 * 输出  ：无
 * 调用  ：在 SysTick 中断函数 SysTick_Handler()调用
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


/******************* (C) COPYRIGHT 2011 野火嵌入式开发工作室 *****END OF FILE****/
