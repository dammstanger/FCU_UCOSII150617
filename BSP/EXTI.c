/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��EXTI.c
 * ��	��	���ⲿ�жϳ�ʼ�����жϷ���
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150412
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.4.11
 * ���༭	��2015.4.12
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/



/****************************����ͷ�ļ�*******************************************/
#include "EXTI.h"
#include "ComProtocol.h"
#include "LED.h"
#include "MPU.h"
#include "HMC5883.h"
/****************************�궨��***********************************************/

/****************************��������*********************************************/

/****************************��������*********************************************/

/****************************��������*********************************************/

/********************************************************************************
 * ��������EXTI_PC5_24L01()
 * ����  ����������24L01��IRQ�ⲿ�ж�
 * ����  ��-		    	
 * ����  ��-
 * ����  ���ⲿ
 ********************************************************************************/
void EXTI_PC5_24L01_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource5);

	EXTI_InitStructure.EXTI_Line=EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//ʹ��IRQ���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�0�� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 10;			//�����ȼ�10
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); 	

}

/********************************************************************************
 * ��������EXTI_PC8_MPU6050_Init()
 * ����  ����������MPU6050��IRQ�ⲿ�ж�
 * ����  ��-
 * ����  ��-
 * ����  ���ⲿ
 ********************************************************************************/
void EXTI_PC8_MPU6050_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource8);

	EXTI_InitStructure.EXTI_Line=EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;				//ʹ��IRQ���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�0�� 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;				//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); 	

}


/********************************************************************************
 * ��������EXTI_PC12_HMC5883_Init()
 * ����  ����������HMC5883��IRQ�ⲿ�ж�
 * ����  ��-		    	
 * ����  ��-
 * ����  ���ⲿ
 ********************************************************************************/
void EXTI_PC12_HMC5883_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource12);

	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ��IRQ���ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;					//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); 	

}


//�ⲿ�ж�0������� 
void EXTI9_5_IRQHandler(void)
{	
	if(EXTI_GetITStatus(EXTI_Line5))
	{
//		LEDTrg(5);  				
		RevFinish = RC_Handle();
		EXTI_ClearITPendingBit(EXTI_Line5); //���LINE_5�ϵ��жϱ�־λ  
	}
	if(EXTI_GetITStatus(EXTI_Line8))
	{
		MPUDataRead();						
		HMC5883L_MultRead(&hmc5883l);
		EXTI_ClearITPendingBit(EXTI_Line8); //���LINE5�ϵ��жϱ�־λ
	}

}


//�ⲿ�ж�0������� 
void EXTI15_10_IRQHandler(void)
{	
	EXTI_ClearITPendingBit(EXTI_Line12); //���LINE0�ϵ��жϱ�־λ  
}


		
/******************* (C) COPYRIGHT 2015 DammStanger *****END OF FILE************/
