/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��SR04.h
 * ��	��	���������
 *                    
 * ʵ��ƽ̨	��STM32_��Сϵͳ
 * Ӳ������	��
 * �� 	��	��V1.0
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.24
 * ���༭	��2014.12.25
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/
#ifndef _SR04_H_
#define _SR04_H_
/****************************����ͷ�ļ�*******************************************/

#include "Project_cfg.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"

#if	SONAR_ONBOARD

/****************************�궨��***********************************************/

#define SR04_GPIO_CLK		RCC_APB2Periph_GPIOB
#define SR04_PORT_T			GPIOB
#define SR04_PORT_R			GPIOB
#define SR04_PIN_T  		GPIO_Pin_6  //
#define SR04_PIN_R  		GPIO_Pin_7  //
#define SR04_SRTTROGLE_H	GPIO_SetBits(SR04_PORT_T,SR04_PIN_T)
#define SR04_SRTTROGLE_L	GPIO_ResetBits(SR04_PORT_T,SR04_PIN_T)

/****************************�ṹ�嶨��*******************************************/

/****************************��������*********************************************/
extern volatile u8 TIM4_Cap2STA;		     	//���� 7λ��ɱ�־ 6bit ����ʼ��־
extern volatile u16 TIM4_Cap2Val;	//
extern volatile u16 TIM4_Cap2temp;

/****************************��������*********************************************/

void SR04_GPIOInit();	//PB6����Ϊ���䴥��
void SR04_Start();
u16   SR04_HightCal();
void TIM4_PulseW_Config();
void TIM4_NVIC_Configuration();
void TIM4_PulseW_GPIOInit(void);	//PB7��Ϊ����CAP��

#endif //#if	SONAR_ONBOARD




#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

