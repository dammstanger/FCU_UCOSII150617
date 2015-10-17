
#include "Timer_PWMin.h"
#include "led.h"



volatile u8 TIM2_Cap1STA = 0;		     	//定义 7位完成标志 6bit 捕获开始标志 0bit指示捕捉错误
volatile u8 TIM2_Cap2STA = 0;		     	//定义 7位完成标志 6bit 捕获开始标志
volatile u8 TIM2_Cap3STA = 0;		     	//定义 7位完成标志 6bit 捕获开始标志
volatile u8 TIM2_Cap4STA = 0;		     	//定义 7位完成标志 6bit 捕获开始标志
volatile u16 TIM2_Cap1temp = 0;	//
volatile u16 TIM2_Cap2temp = 0;	//
volatile u16 TIM2_Cap3temp = 0;	//
volatile u16 TIM2_Cap4temp = 0;	//
volatile u16 TIM2_Cap1Val = 0;	//
volatile u16 TIM2_Cap2Val = 0;	//
volatile u16 TIM2_Cap3Val = 0;	//
volatile u16 TIM2_Cap4Val = 0;	//

volatile u8 TIM3_Cap3STA = 0;		     	//定义 7位完成标志 6bit 捕获开始标志
volatile u8 TIM3_Cap4STA = 0;		     	//定义 7位完成标志 6bit 捕获开始标志
volatile u16 TIM3_Cap3Val = 0;	//
volatile u16 TIM3_Cap4Val = 0;	//
volatile u16 TIM3_Cap3temp = 0;	//
volatile u16 TIM3_Cap4temp = 0;	//
/*
 * 函数名：TIM2_NVIC_Configuration
 * 描述  ：TIM2中断优先级配置
 * 输入  ：无
 * 输出  ：无	
 */
void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


/*
 * 函数名：TIM3_NVIC_Configuration
 * 描述  ：TIM3中断优先级配置
 * 输入  ：无
 * 输出  ：无	
 */
void TIM3_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_PWMIN_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;       
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM3_PWMIN_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;       
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*TIM_Period--0xffff   TIM_Prescaler--71 -->中断周期为1us*/
void TIM3_PWMIN_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM3_ICInitStructure;
	
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Period=0xffff;		 								/* 自动重装载寄存器周期的值(计数值) 10 */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* 时钟预分频数 72M/72=1us*/
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* 采样分频 不分频*/
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* 向上计数模式 */
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//----------------------------TIM3ch3-ch3--------------------------------------
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM3_ICInitStructure.TIM_ICFilter = 0;//不滤波
	TIM_ICInit(TIM3,&TIM3_ICInitStructure);
	
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM3_ICInitStructure.TIM_ICFilter = 0;//不滤波
	TIM_ICInit(TIM3,&TIM3_ICInitStructure);
	
    TIM_ClearFlag(TIM3, TIM_FLAG_Update|TIM_FLAG_CC3|TIM_FLAG_CC4);		/* 清除溢出中断标志 */
    
	TIM_Cmd(TIM3, ENABLE);
	
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4,DISABLE);
}


/*TIM_Period--0xffff   TIM_Prescaler--71 -->中断周期为1us*/
void TIM2_PWMIN_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period=0xffff;		 				/* 自动重装载寄存器周期的值(计数值) 10 */
    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* 时钟预分频数 72M/72=1us*/
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 			/* 采样分频 不分频*/
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		/* 向上计数模式 */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	//----------------------------TIM2ch2-ch1--------------------------------------
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;//不滤波
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;//不滤波
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;//不滤波
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;//不滤波
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
    TIM_ClearFlag(TIM2, TIM_FLAG_Update|TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4);						/* 清除溢出中断标志 */
    
	TIM_Cmd(TIM2, ENABLE);
	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,DISABLE);
}


//TIM2 负责RC信号输入捕捉
void TIM2_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)			//定时器溢出
	{	//-----------------------------1----------------------------------------
		if(TIM2_Cap1STA&0x40)								//之前已捕获到电压上升沿
		{	
			TIM2_Cap1STA++; 
			if((TIM2_Cap1STA&0x0f)>=2)					
			{
				TIM2_Cap1STA = 0x80;						//强制完成 
				TIM2_Cap1Val = 0;						//指示错误	
				TIM2->CCER &=~(1<<1);						//CC1P=0 设置为上升沿捕获
			}
		}
		//-----------------------------2----------------------------------------
		if(TIM2_Cap2STA&0x40)								//之前已捕获到电压上升沿
		{	
			TIM2_Cap2STA++; 
			if((TIM2_Cap2STA&0x0f)>=2)					
			{
				TIM2_Cap2STA = 0x80;						//强制完成 
				TIM2_Cap2Val = 0;						//指示错误	
				TIM2->CCER &=~(1<<5);						//CC2P=0 设置为上升沿捕获
			}
		}
		//------------------------------3---------------------------------------
		if(TIM2_Cap3STA&0x40)								//之前已捕获到电压上升沿
		{	
			TIM2_Cap3STA++; 
			if((TIM2_Cap3STA&0x0f)>=2)					
			{
				TIM2_Cap3STA = 0x80;						//强制完成 
				TIM2_Cap3Val = 0;						//指示错误	
				TIM2->CCER &=~(1<<9);						//CC3P=0 设置为上升沿捕获
			}
		}
		//------------------------------4---------------------------------------
		if(TIM2_Cap4STA&0x40)								//之前已捕获到电压上升沿
		{	
			TIM2_Cap4STA++; 
			if((TIM2_Cap4STA&0x0f)>=2)					
			{
				TIM2_Cap4STA = 0x80;						//强制完成 
				TIM2_Cap4Val = 0;						//指示错误	
				TIM2->CCER &=~(1<<13);						//CC4P=0 设置为上升沿捕获
			}
		}
	}
	//---------------------------------A-----------------------------------------
	if(TIM_GetITStatus(TIM2,TIM_IT_CC1)!=RESET) 		//捕获到电压沿开始
	{						
		if(TIM2_Cap1STA&0x40)								//捕获到下降沿
		{		
			TIM2_Cap1STA = 0x80;								//完成标记
			TIM2_Cap1Val = TIM2->CCR1;		//与之前值做差，得到脉宽时间
			if(TIM2_Cap1Val>TIM2_Cap1temp)
				TIM2_Cap1Val = TIM2_Cap1Val - TIM2_Cap1temp;
			else 
				TIM2_Cap1Val = 0xffff - TIM2_Cap1temp + TIM2_Cap1Val;
			
			TIM2->CCER &=~(1<<1); 								//CC1P=0 设置为上升沿捕获
		}
		else
		{
			TIM2_Cap1STA = 0;								//清零保证下次使用不混乱
			TIM2_Cap1Val = 0;								//
			TIM2_Cap1temp = TIM2->CCR1;						//获取当前的捕获值.
			TIM2_Cap1STA  = 0x40;							//设置开始标志
			TIM2->CCER|=1<<1; 								//CC1P=1 设置为下降沿捕获
		}						
	} 
	//---------------------------------B-----------------------------------------
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2)!=RESET) 		//捕获到电压沿开始
	{	
		if(TIM2_Cap2STA&0x40)								//捕获到下降沿
		{		
			TIM2_Cap2STA = 0x80;								//完成标记
			TIM2_Cap2Val = TIM2->CCR2;		//与之前值做差，得到脉宽时间
			if(TIM2_Cap2Val>TIM2_Cap2temp)
				TIM2_Cap2Val = TIM2_Cap2Val - TIM2_Cap2temp;
			else 
				TIM2_Cap2Val = 0xffff - TIM2_Cap2temp + TIM2_Cap2Val;
			
			TIM2->CCER &=~(1<<5); 								//CC2P=0 设置为上升沿捕获
		}
		else
		{
			TIM2_Cap2STA = 0;								//清零保证下次使用不混乱
			TIM2_Cap2Val = 0;								//
			TIM2_Cap2temp = TIM2->CCR2;						//获取当前的捕获值.
			TIM2_Cap2STA  = 0x40;							//设置开始标志
			TIM2->CCER|=1<<5; 								//CC2P=1 设置为下降沿捕获
		}
	} 
		//---------------------------------C-----------------------------------------
	if(TIM_GetITStatus(TIM2,TIM_IT_CC3)!=RESET) 		//捕获到电压沿开始
	{	
		if(TIM2_Cap3STA&0x40)								//捕获到下降沿
		{		
			TIM2_Cap3STA = 0x80;								//完成标记
			TIM2_Cap3Val = TIM2->CCR3;		//与之前值做差，得到脉宽时间
			if(TIM2_Cap3Val>TIM2_Cap3temp)
				TIM2_Cap3Val = TIM2_Cap3Val - TIM2_Cap3temp;
			else 
				TIM2_Cap3Val = 0xffff - TIM2_Cap3temp + TIM2_Cap3Val;
			
			TIM2->CCER &=~(1<<9); 								//CC3P=0 设置为上升沿捕获
		}
		else
		{
			TIM2_Cap3STA = 0;								//清零保证下次使用不混乱
			TIM2_Cap3Val = 0;								//
			TIM2_Cap3temp = TIM2->CCR3;						//获取当前的捕获值.
			TIM2_Cap3STA  = 0x40;							//设置开始标志
			TIM2->CCER|=1<<9; 								//CC3P=1 设置为下降沿捕获
		}
	} 
		//---------------------------------D-----------------------------------------
	if(TIM_GetITStatus(TIM2,TIM_IT_CC4)!=RESET) 		//捕获到电压沿开始
	{	
		if(TIM2_Cap4STA&0x40)								//捕获到下降沿
		{		
			TIM2_Cap4STA = 0x80;								//完成标记
			TIM2_Cap4Val = TIM2->CCR4;		//与之前值做差，得到脉宽时间
			if(TIM2_Cap4Val>TIM2_Cap4temp)
				TIM2_Cap4Val = TIM2_Cap4Val - TIM2_Cap4temp;
			else 
				TIM2_Cap4Val = 0xffff - TIM2_Cap4temp + TIM2_Cap4Val;
			
			TIM2->CCER &=~(1<<13); 								//CC4P=0 设置为上升沿捕获
		}
		else
		{
			TIM2_Cap4STA = 0;								//清零保证下次使用不混乱
			TIM2_Cap4Val = 0;								//
			TIM2_Cap4temp = TIM2->CCR4;						//获取当前的捕获值.
			TIM2_Cap4STA  = 0x40;							//设置开始标志
			TIM2->CCER|=1<<13; 								//CC4P=1 设置为下降沿捕获
		}
	} 
	TIM_ClearFlag(TIM2, TIM_FLAG_Update|TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4); //标志位必须有软件清除	有软件清除	
}



//TIM3 负责RC信号输入捕捉
void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)			//定时器溢出
	{	//---------------------------------3------------------------------------
		if(TIM3_Cap3STA&0x40)								//之前已捕获到电压上升沿
		{	
			TIM3_Cap3STA++; 
			if((TIM3_Cap3STA&0x0f)>=2)					
			{
				TIM3_Cap3STA = 0x80;						//强制完成 
				TIM3_Cap3Val = 0;						//指示错误	
				TIM3->CCER &=~(1<<9);						//CC1P=0 设置为上升沿捕获
			}
		}
		//---------------------------------4-----------------------------------------
		if(TIM3_Cap4STA&0x40)								//之前已捕获到电压上升沿
		{	
			TIM3_Cap4STA++; 
			if((TIM3_Cap4STA&0x0f)>=2)					
			{
				TIM3_Cap4STA = 0x80;						//强制完成 
				TIM3_Cap4Val = 0;						//指示错误	
				TIM3->CCER &=~(1<<13);						//CC1P=0 设置为上升沿捕获
			}
		}
	}
//---------------------------------C-----------------------------------------
	if(TIM_GetITStatus(TIM3,TIM_IT_CC3)!=RESET) 		//捕获到电压沿开始
	{	
		if(TIM3_Cap3STA&0x40)								//捕获到下降沿
		{		
			TIM3_Cap3STA = 0x80;								//完成标记
			TIM3_Cap3Val = TIM3->CCR3;		//与之前值做差，得到脉宽时间
			if(TIM3_Cap3Val>TIM3_Cap3temp)
				TIM3_Cap3Val = TIM3_Cap3Val - TIM3_Cap3temp;
			else 
				TIM3_Cap3Val = 0xffff - TIM3_Cap3temp + TIM3_Cap3Val;
			
			TIM3->CCER &=~(1<<9); 								//CC3P=0 设置为上升沿捕获
		}
		else
		{
			TIM3_Cap3STA = 0;								//清零保证下次使用不混乱
			TIM3_Cap3Val = 0;								//
			TIM3_Cap3temp = TIM3->CCR3;						//获取当前的捕获值.
			TIM3_Cap3STA  = 0x40;							//设置开始标志
			TIM3->CCER|=1<<9; 								//CC3P=1 设置为下降沿捕获
		}
	} 
//---------------------------------D-----------------------------------------
	if(TIM_GetITStatus(TIM3,TIM_IT_CC4)!=RESET) 		//捕获到电压沿开始
	{	
		if(TIM3_Cap4STA&0x40)								//捕获到下降沿
		{		
			TIM3_Cap4STA = 0x80;								//完成标记
			TIM3_Cap4Val = TIM3->CCR4;		//与之前值做差，得到脉宽时间
			if(TIM3_Cap4Val>TIM3_Cap4temp)
				TIM3_Cap4Val = TIM3_Cap4Val - TIM3_Cap4temp;
			else 
				TIM3_Cap4Val = 0xffff - TIM3_Cap4temp + TIM3_Cap4Val;
			
			TIM3->CCER &=~(1<<13); 								//CC4P=0 设置为上升沿捕获
		}
		else
		{
			TIM3_Cap4STA = 0;								//清零保证下次使用不混乱
			TIM3_Cap4Val = 0;								//
			TIM3_Cap4temp = TIM3->CCR4;						//获取当前的捕获值.
			TIM3_Cap4STA  = 0x40;							//设置开始标志
			TIM3->CCER|=1<<13; 								//CC4P=1 设置为下降沿捕获
		}
	} 
	TIM_ClearFlag(TIM3, TIM_FLAG_Update|TIM_FLAG_CC3|TIM_FLAG_CC4); //标志位必须有软件清除
}	
	
	
