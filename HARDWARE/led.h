/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：led.h
 * 描	述	：各个应用程序
 *                    
 * 实验平台	：FCU v1.0
 * 硬件连接：-----------------
 *          |   PC1 - LED1     |
 *          |   PC2 - LED2     |
 *          |   PC3 - LED3     |
			|	PA12- LED4
			|	PB5 - ALARM
 *           ----------------- 
 * 版 	本	：V1.0.150407
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.3.29
 * 最后编辑	：2015.4.7
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/
#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"
#include "Project_cfg.h"


#define ST_ON 		0x80
#define ST_OFF		0x00
#define ST_FL_100	0x11
#define ST_FL_200	0x12
#define ST_FL_500	0x15
#define ST_FL_1000	0x1a


#define ON  0
#define OFF 1


#define LED1(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_1);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_1)

#define LED2(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_2);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_2)

#define LED3(a)	if (a)	\
					GPIO_SetBits(GPIOC,GPIO_Pin_3);\
					else		\
					GPIO_ResetBits(GPIOC,GPIO_Pin_3)
					
#define LED4(a)	if (a)	\
					GPIO_SetBits(GPIOA,GPIO_Pin_12);\
					else		\
					GPIO_ResetBits(GPIOA,GPIO_Pin_12)					

#define ALARM(a)	if (a)	\
					GPIO_SetBits(GPIOB,GPIO_Pin_5);\
					else		\
					GPIO_ResetBits(GPIOB,GPIO_Pin_5)						

extern u8 LED1State;
extern u8 LED2State;
extern u8 LED3State;
extern u8 LED4State;					
					
void LED_BUTTON_GPIO_Config(void);
void LEDTrg(u8 led);					

#endif /* __LED_H */
