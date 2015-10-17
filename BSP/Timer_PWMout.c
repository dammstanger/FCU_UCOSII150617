/******************** (C) COPYRIGHT 2014 ********************
 * 文件名  ：pwm_output.c
 * 描述    ：         
 * 实验平台：FCUV1.0
 *           - PC.06: (TIM8_CH1)
 *           - PC.07: (TIM8_CH2)
 *           - PA.08: (TIM1_CH1)
 *           - PA.09: (TIM1_CH2) 
 *           - PA.10: (TIM1_CH3)
 *           - PA.11: (TIM1_CH4)  
 
 *           --------------------- 
电调油门信号频率范围（Refresh rate）：50Hz—432Hz；
电调控制信号PWM标准：
1、与脉宽有关，与平均值无关，3V5V电压兼容。
2、1ms~2ms的方波脉冲
3、频率不同，但是高电平脉宽不随平率变化：单向电调，单向电调，1ms表示0%的油门，
	2ms表示100%的油门。如果是双向电调（有正、反转和刹车），标准1.5ms是0点，
	1ms是反向油门最大（100%油门），用于刹车或反转；2ms正向油门最大（100%油门，
	用于正转。

 * 库版本  ：ST3.0.0
 *
**********************************************************************************/
#include "Timer_PWMout.h"
#include "usart.h"
#include "Control.h"
#include "stm32f10x_tim.h"



/*
 * 函数名：TIM1_GPIO_Config
 * 描述  ：配置TIM1复用输出PWM时用到的I/O
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*
 * 函数名：TIM8_GPIO_Config
 * 描述  ：配置TIM1复用输出PWM时用到的I/O
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
void TIM8_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
	/* TIM8 clock enable 72MHz*/

  /* GPIOC and  clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 | RCC_APB2Periph_GPIOC, ENABLE); 

  /*GPIOC Configuration: TIM8 channel 1 and 2 as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOC, &GPIO_InitStructure);

}


/*
 * 函数名：TIM8_Mode_Config
 * 描述  ：配置TIM8输出的PWM信号的模式，如周期、极性、占空比
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
void TIM8_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM信号电平跳变值 */
	u16 CCR1_Val = 4000;        //400Hz频率下周期值2.5ms 对应10000，1ms高脉宽值对应4000
	u16 CCR2_Val = 4000;	//

  /* Time base configuration */		 
  TIM_TimeBaseStructure.TIM_Period = 9999;       //TIM8_CCR1当定时器从0计数到999，即为10000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Prescaler = 18-1;	    //设置预分频：不预分频，即为4MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平

  TIM_OC1Init(TIM8, &TIM_OCInitStructure);	 //使能通道1

  TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM

  TIM_OC2Init(TIM8, &TIM_OCInitStructure);	  //使能通道2

  TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM8, ENABLE);			 	// 使能TIM8重载寄存器ARR
  
  TIM_CtrlPWMOutputs(TIM8,ENABLE);					//使能定时器8的主输出，该函数是TIM8和TIM8中	特有的，其他TIM不用

  /* TIM8 enable counter */
  TIM_Cmd(TIM8, ENABLE);                   //使能定时器8	
}

/*
 * 函数名：TIM1_Mode_Config
 * 描述  ：配置TIM1输出的PWM信号的模式，如周期、极性、占空比
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
void TIM1_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
#ifdef CALIBRATE_PWMOUT
	/* PWM信号电平跳变值 */
	u16 CCR1_Val = 8000;       
	u16 CCR2_Val = 8000;	
	u16 CCR3_Val = 8000;
	u16 CCR4_Val = 8000;
#else	
	/* PWM信号电平跳变值 */
	u16 CCR1_Val = 4000;    //400Hz频率下周期值2.5ms 对应10000，1ms高脉宽值对应4000
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
  TIM_TimeBaseStructure.TIM_Period = 9999;       	//TIM8_CCR1当定时器从0计数到999，即为10000次，为一个定时周期
  TIM_TimeBaseStructure.TIM_Prescaler = 18-1;	    //设置预分频：不预分频，即为2MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;		//设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);	 	//使能通道1

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;	  	//设置通道2的电平跳变值，输出另外一个占空比的PWM

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);	  	//使能通道2

  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;		//设置通道3的电平跳变值，输出另外一个占空比的PWM

  TIM_OC3Init(TIM1, &TIM_OCInitStructure);	 	//使能通道3

  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;		//设置通道4的电平跳变值，输出另外一个占空比的PWM

  TIM_OC4Init(TIM1, &TIM_OCInitStructure);		//使能通道4

  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);			 	// 使能TIM1重载寄存器ARR
  
  TIM_CtrlPWMOutputs(TIM1,ENABLE);					//使能定时器1的主输出，该函数是TIM1和TIM8中	特有的，其他TIM不用

  /* TIM1 enable counter */
  TIM_Cmd(TIM1, ENABLE);                   			//使能定时器1	
}



/*
 * 函数名：TIM1_PWM_Init
 * 描述  ：TIM1 输出PWM信号初始化，只要调用这个函数
 *         TIM1的四个通道就会有PWM信号输出
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
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
