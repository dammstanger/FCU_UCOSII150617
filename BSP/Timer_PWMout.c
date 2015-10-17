/******************** (C) COPYRIGHT 2014 ********************
 * �ļ���  ��pwm_output.c
 * ����    ��         
 * ʵ��ƽ̨��FCUV1.0
 *           - PC.06: (TIM8_CH1)
 *           - PC.07: (TIM8_CH2)
 *           - PA.08: (TIM1_CH1)
 *           - PA.09: (TIM1_CH2) 
 *           - PA.10: (TIM1_CH3)
 *           - PA.11: (TIM1_CH4)  
 
 *           --------------------- 
��������ź�Ƶ�ʷ�Χ��Refresh rate����50Hz��432Hz��
��������ź�PWM��׼��
1���������йأ���ƽ��ֵ�޹أ�3V5V��ѹ���ݡ�
2��1ms~2ms�ķ�������
3��Ƶ�ʲ�ͬ�����Ǹߵ�ƽ������ƽ�ʱ仯������������������1ms��ʾ0%�����ţ�
	2ms��ʾ100%�����š������˫��������������ת��ɲ��������׼1.5ms��0�㣬
	1ms�Ƿ����������100%���ţ�������ɲ����ת��2ms�����������100%���ţ�
	������ת��

 * ��汾  ��ST3.0.0
 *
**********************************************************************************/
#include "Timer_PWMout.h"
#include "usart.h"
#include "Control.h"
#include "stm32f10x_tim.h"



/*
 * ��������TIM1_GPIO_Config
 * ����  ������TIM1�������PWMʱ�õ���I/O
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
void TIM1_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	/* TIM1 clock enable 72MHz*/

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA, ENABLE); 

  /*GPIOA Configuration: TIM1 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*
 * ��������TIM8_GPIO_Config
 * ����  ������TIM1�������PWMʱ�õ���I/O
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
void TIM8_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
	/* TIM8 clock enable 72MHz*/

  /* GPIOC and  clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 | RCC_APB2Periph_GPIOC, ENABLE); 

  /*GPIOC Configuration: TIM8 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOC, &GPIO_InitStructure);

}


/*
 * ��������TIM8_Mode_Config
 * ����  ������TIM8�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ�
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
void TIM8_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM�źŵ�ƽ����ֵ */
	u16 CCR1_Val = 4000;        //400HzƵ��������ֵ2.5ms ��Ӧ10000��1ms������ֵ��Ӧ4000
	u16 CCR2_Val = 4000;	//

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 9999;       //TIM8_CCR1����ʱ����0������999����Ϊ10000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Prescaler = 18-1;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ4MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ

  TIM_OC1Init(TIM8, &TIM_OCInitStructure);	 //ʹ��ͨ��1

  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM

  TIM_OC2Init(TIM8, &TIM_OCInitStructure);	  //ʹ��ͨ��2

  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM8, ENABLE);			 	// ʹ��TIM8���ؼĴ���ARR
  
  TIM_CtrlPWMOutputs(TIM8,ENABLE);					//ʹ�ܶ�ʱ��8����������ú�����TIM8��TIM8��	���еģ�����TIM����

  /* TIM8 enable counter */
  TIM_Cmd(TIM8, ENABLE);                   //ʹ�ܶ�ʱ��8	
}

/*
 * ��������TIM1_Mode_Config
 * ����  ������TIM1�����PWM�źŵ�ģʽ�������ڡ����ԡ�ռ�ձ�
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
void TIM1_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
#ifdef CALIBRATE_PWMOUT
	/* PWM�źŵ�ƽ����ֵ */
	u16 CCR1_Val = 8000;       
	u16 CCR2_Val = 8000;	
	u16 CCR3_Val = 8000;
	u16 CCR4_Val = 8000;
#else	
	/* PWM�źŵ�ƽ����ֵ */
	u16 CCR1_Val = 4000;    //400HzƵ��������ֵ2.5ms ��Ӧ10000��1ms������ֵ��Ӧ4000
	u16 CCR2_Val = 4000;	//
	u16 CCR3_Val = 4000;
	u16 CCR4_Val = 4000;
#endif
/* -----------------------------------------------------------------------
    TIM1 Configuration: generate 4 PWM signals with 4 different duty cycles:
    TIM1CLK = 72 MHz, Prescaler = 18, TIM1 counter clock = 4 MHz
    TIM1 ARR Register = 9999 => TIM1 Frequency = TIM1 counter clock/(ARR + 1)
    TIM1 Frequency = 400Hz.

  ----------------------------------------------------------------------- */

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 9999;       	//TIM8_CCR1����ʱ����0������999����Ϊ10000�Σ�Ϊһ����ʱ����
  TIM_TimeBaseStructure.TIM_Prescaler = 18-1;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ2MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;		//����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //��������ֵ�������������������ֵʱ����ƽ��������
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);	 	//ʹ��ͨ��1

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  	//����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);	  	//ʹ��ͨ��2

  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;		//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM

  TIM_OC3Init(TIM1, &TIM_OCInitStructure);	 	//ʹ��ͨ��3

  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;		//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM

  TIM_OC4Init(TIM1, &TIM_OCInitStructure);		//ʹ��ͨ��4

  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);			 	// ʹ��TIM1���ؼĴ���ARR
  
  TIM_CtrlPWMOutputs(TIM1,ENABLE);					//ʹ�ܶ�ʱ��1����������ú�����TIM1��TIM8��	���еģ�����TIM����

  /* TIM1 enable counter */
  TIM_Cmd(TIM1, ENABLE);                   			//ʹ�ܶ�ʱ��1	
}



/*
 * ��������TIM1_PWM_Init
 * ����  ��TIM1 ���PWM�źų�ʼ����ֻҪ�����������
 *         TIM1���ĸ�ͨ���ͻ���PWM�ź����
 * ����  ����
 * ���  ����
 * ����  ���ⲿ����
 */
void TIM1_PWM_Init(void)
{
	TIM1_Mode_Config();	
}


void MotoOutPut(int32_t MOTO1_PWM,int32_t MOTO2_PWM,int32_t MOTO3_PWM,int32_t MOTO4_PWM,int32_t MOTO5_PWM,int32_t MOTO6_PWM)
{	
#ifdef CALIBRATE_PWMOUT	
	if(MOTO1_PWM>MOTO_PWM_MAX)	MOTO1_PWM = MOTO_PWM_MAX;
	if(MOTO2_PWM>MOTO_PWM_MAX)	MOTO2_PWM = MOTO_PWM_MAX;
	if(MOTO3_PWM>MOTO_PWM_MAX)	MOTO3_PWM = MOTO_PWM_MAX;
	if(MOTO4_PWM>MOTO_PWM_MAX)	MOTO4_PWM = MOTO_PWM_MAX;
	if(MOTO5_PWM>MOTO_PWM_MAX)	MOTO5_PWM = MOTO_PWM_MAX;
	if(MOTO6_PWM>MOTO_PWM_MAX)	MOTO6_PWM = MOTO_PWM_MAX;
	
	if(MOTO1_PWM<MOTO_PWM_MIN)	MOTO1_PWM = MOTO_PWM_MIN;
	if(MOTO2_PWM<MOTO_PWM_MIN)	MOTO2_PWM = MOTO_PWM_MIN;
	if(MOTO3_PWM<MOTO_PWM_MIN)	MOTO3_PWM = MOTO_PWM_MIN;
	if(MOTO4_PWM<MOTO_PWM_MIN)	MOTO4_PWM = MOTO_PWM_MIN;
	if(MOTO5_PWM<MOTO_PWM_MIN)	MOTO5_PWM = MOTO_PWM_MIN;
	if(MOTO6_PWM<MOTO_PWM_MIN)	MOTO6_PWM = MOTO_PWM_MIN;
#endif	
	
	TIM1->CCR1= 4*MOTO1_PWM;
	TIM1->CCR2= 4*MOTO2_PWM;
	TIM1->CCR3= 4*MOTO3_PWM;
	TIM1->CCR4= 4*MOTO4_PWM;
//	TIM8->CCR1= 4*MOTO5_PWM;
//	TIM8->CCR2= 4*MOTO6_PWM;
	
}

/******************* (C) COPYRIGHT 2014*****END OF FILE*****/
