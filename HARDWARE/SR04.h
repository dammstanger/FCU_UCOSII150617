/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：SR04.h
 * 描	述	：超声测距
 *                    
 * 实验平台	：STM32_最小系统
 * 硬件连接	：
 * 版 	本	：V1.0
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.24
 * 最后编辑	：2014.12.25
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/
#ifndef _SR04_H_
#define _SR04_H_
/****************************包含头文件*******************************************/

#include "Project_cfg.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"

#if	SONAR_ONBOARD

/****************************宏定义***********************************************/

#define SR04_GPIO_CLK		RCC_APB2Periph_GPIOB
#define SR04_PORT_T			GPIOB
#define SR04_PORT_R			GPIOB
#define SR04_PIN_T  		GPIO_Pin_6  //
#define SR04_PIN_R  		GPIO_Pin_7  //
#define SR04_SRTTROGLE_H	GPIO_SetBits(SR04_PORT_T,SR04_PIN_T)
#define SR04_SRTTROGLE_L	GPIO_ResetBits(SR04_PORT_T,SR04_PIN_T)

/****************************结构体定义*******************************************/

/****************************变量声明*********************************************/
extern volatile u8 TIM4_Cap2STA;		     	//定义 7位完成标志 6bit 捕获开始标志
extern volatile u16 TIM4_Cap2Val;	//
extern volatile u16 TIM4_Cap2temp;

/****************************函数声明*********************************************/

void SR04_GPIOInit();	//PB6口作为发射触发
void SR04_Start();
u16   SR04_HightCal();
void TIM4_PulseW_Config();
void TIM4_NVIC_Configuration();
void TIM4_PulseW_GPIOInit(void);	//PB7作为接受CAP口

#endif //#if	SONAR_ONBOARD




#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

