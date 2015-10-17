//20150102
#ifndef __TIMER_PWMIN_H_
#define	__TIMER_PWMIN_H_

#include "Project_cfg.h"
#include "stm32f10x.h"

//#define START_TIME  time=0;RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);TIM_Cmd(TIM2, ENABLE)
//#define STOP_TIME  TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)

extern volatile u8 TIM2_Cap1STA;		//定义 7位完成标志 6bit 捕获开始标志   0~5bit 溢出计数
extern volatile u8 TIM2_Cap2STA;
extern volatile u8 TIM2_Cap3STA;
extern volatile u8 TIM2_Cap4STA;
extern volatile u16 TIM2_Cap1Val;
extern volatile u16 TIM2_Cap2Val;	//
extern volatile u16 TIM2_Cap3Val;	//
extern volatile u16 TIM2_Cap4Val;	//

extern volatile u8 TIM3_Cap3STA;
extern volatile u8 TIM3_Cap4STA;
extern volatile u16 TIM3_Cap3Val;	//
extern volatile u16 TIM3_Cap4Val;	//
 
void TIM2_NVIC_Configuration(void);
void TIM3_NVIC_Configuration(void);
void TIM2_PWMIN_Configuration(void);
void TIM2_PWMIN_GPIOInit(void);
void TIM3_PWMIN_Configuration(void);
void TIM3_PWMIN_GPIOInit(void);

#endif

