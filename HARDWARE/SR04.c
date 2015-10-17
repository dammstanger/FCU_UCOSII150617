//20140416
#include "SR04.h"
#include "SysTick.h"

#if	SONAR_ONBOARD

/**********************************
TIMER4的CAPture 来测量脉宽
***********************************/

volatile u8  TIM4_Cap2STA = 0;		     	//定义 7位完成标志 6bit 捕获开始标志
volatile u16 TIM4_Cap2Val = 0;	//
volatile u16 TIM4_Cap2temp = 0;	//


void SR04_GPIOInit()	//PB6口作为发射触发
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = SR04_PIN_T;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, SR04_PIN_T);	 //turn off all led
}

/*
 * 函数名：TIM4_NVIC_Configuration
 * 描述  ：TIM4中断优先级配置
 * 输入  ：无
 * 输出  ：无	
 */

void TIM4_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM4_PulseW_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

	GPIO_InitStructure.GPIO_Pin = SR04_PIN_R;				//TIM4CAP2 PB7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;       
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*TIM_Period--0xffff   TIM_Prescaler--71 -->中断周期为1us*/
void TIM4_PulseW_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM4_ICInitStructure;
	
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Period=0xffff;		 								/* 自动重装载寄存器周期的值(计数值) 10 */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* 时钟预分频数 72M/72=1us*/
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* 采样分频 不分频*/
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* 向上计数模式 */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//----------------------------TIM4ch1--------------------------------------
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM4_ICInitStructure.TIM_ICFilter = 0;//不滤波
	TIM_ICInit(TIM4,&TIM4_ICInitStructure);
	
	
    TIM_ClearFlag(TIM4, TIM_FLAG_Update|TIM_FLAG_CC2);		/* 清除溢出中断标志 */
    
	TIM_Cmd(TIM4, ENABLE);
	
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,DISABLE);
}


void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)			//定时器溢出
	{	//---------------------------------2------------------------------------
		if(TIM4_Cap2STA&0x40)								//之前已捕获到电压上升沿
		{	
			TIM4_Cap2STA++; 
			if((TIM4_Cap2STA&0x0f)>=2)					
			{
				TIM4_Cap2STA = 0x80;						//强制完成 
				TIM4_Cap2Val = 65535;							//指示错误	
				TIM4->CCER &=~(1<<5);						//CC1P=0 设置为上升沿捕获
				TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,DISABLE);
			}
		}
	}
	//---------------------------------B-----------------------------------------
	if(TIM_GetITStatus(TIM4,TIM_IT_CC2)!=RESET) 		//捕获到电压沿开始
	{	
		if(TIM4_Cap2STA&0x40)								//捕获到下降沿
		{		
			TIM4_Cap2STA = 0x80;								//完成标记
			TIM4_Cap2Val = TIM4->CCR2;		//与之前值做差，得到脉宽时间
			if(TIM4_Cap2Val>TIM4_Cap2temp)
				TIM4_Cap2Val = TIM4_Cap2Val - TIM4_Cap2temp;
			else 
				TIM4_Cap2Val = 0xffff - TIM4_Cap2temp + TIM4_Cap2Val;
			
			TIM4->CCER &=~(1<<5); 								//CC2P=0 设置为上升沿捕获
			TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,DISABLE);
		}
		else
		{
			TIM4_Cap2STA = 0;								//清零保证下次使用不混乱
			TIM4_Cap2Val = 0;								//
			TIM4_Cap2temp = TIM4->CCR2;						//获取当前的捕获值.
			TIM4_Cap2STA  = 0x40;							//设置开始标志
			TIM4->CCER|=1<<5; 								//CC2P=1 设置为下降沿捕获
		}
	}
	TIM_ClearFlag(TIM4, TIM_FLAG_Update|TIM_FLAG_CC2); //标志位必须有软件清除
	
}


void SR04_Start()
{
	TIM_ClearFlag(TIM4, TIM_FLAG_Update|TIM_FLAG_CC2);						/* 清除溢出中断标志 */
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,ENABLE);
	SR04_SRTTROGLE_H;
	delay_nus(20);
	SR04_SRTTROGLE_L;
}

u16 SR04_HightCal()
{
	static int hightlast = 0;
	int highttemp = 0;
	
	highttemp = (TIM4_Cap2Val*1.7)/10;//单位mm
//	if(highttemp>=500||highttemp<=0) highttemp = hightlast;
//	hightlast = highttemp;
	return highttemp;
}



#endif


