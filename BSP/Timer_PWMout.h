#ifndef _TIMER_PWMOUT_H_
#define	_TIMER_PWMOUT_H_

#include "Project_cfg.h"
#include "stm32f10x.h"
void TIM1_GPIO_Config(void);
void TIM8_GPIO_Config(void);

void TIM1_Mode_Config(void);
void TIM8_Mode_Config(void);
void TIM1_PWM_Init(void);
void TIM8_PWM_Init(void);
void MotoOutPut(int32_t MOTO1_PWM,int32_t MOTO2_PWM,int32_t MOTO3_PWM,int32_t MOTO4_PWM,int32_t MOTO5_PWM,int32_t MOTO6_PWM);


#endif /* _TIMER_PWMOUT_H_ */

