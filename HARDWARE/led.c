/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��led.c
 * ��	��	������Ӧ�ó���
 *                    
 * ʵ��ƽ̨	��FCU v1.0
 * Ӳ�����ӣ�-----------------
 *          |   PC1 - LED1     |
 *          |   PC2 - LED2     |
 *          |   PC3 - LED3     |
			|	PA12- LED4
			|	PB5 - ALARM
 *           ----------------- 
 * �� 	��	��V1.0.150407
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.3.29
 * ���༭	��2015.4.7
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#include "led.h"

u8 LED1State = ST_OFF;
u8 LED2State = ST_OFF;
u8 LED3State = ST_OFF;
u8 LED4State = ST_OFF;

/*
 * ��������LED_BUTTON_GPIO_Config
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
 */
void LED_BUTTON_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_2| GPIO_Pin_3);	 // turn off all led
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_12);	 // turn off all led
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_5);	 // turn off all led
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void LEDTrg(u8 led)
{
	switch(led)
	{
		case 1 : {			if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_1)) LED1(ON);
										else  LED1(OFF);}break;
		case 2 : {			if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_2)) LED2(ON);
										else  LED2(OFF);}break;
		case 3 : {			if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_3)) LED3(ON);
										else  LED3(OFF);}break;
		case 4 : {			if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_12)) LED4(ON);
										else  LED4(OFF);}break;		
		case 5 : {			if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_5)) ALARM(ON);
										else  ALARM(OFF);}break;
		default : break;						
	}
}

/************END OF FILE************/
