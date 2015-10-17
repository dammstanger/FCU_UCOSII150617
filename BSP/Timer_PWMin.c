
#include "Timer_PWMin.h"
#include "led.h"



volatile u8 TIM2_Cap1STA = 0;		     	//���� 7λ��ɱ�־ 6bit ����ʼ��־ 0bitָʾ��׽����
volatile u8 TIM2_Cap2STA = 0;		     	//���� 7λ��ɱ�־ 6bit ����ʼ��־
volatile u8 TIM2_Cap3STA = 0;		     	//���� 7λ��ɱ�־ 6bit ����ʼ��־
volatile u8 TIM2_Cap4STA = 0;		     	//���� 7λ��ɱ�־ 6bit ����ʼ��־
volatile u16 TIM2_Cap1temp = 0;	//
volatile u16 TIM2_Cap2temp = 0;	//
volatile u16 TIM2_Cap3temp = 0;	//
volatile u16 TIM2_Cap4temp = 0;	//
volatile u16 TIM2_Cap1Val = 0;	//
volatile u16 TIM2_Cap2Val = 0;	//
volatile u16 TIM2_Cap3Val = 0;	//
volatile u16 TIM2_Cap4Val = 0;	//

volatile u8 TIM3_Cap3STA = 0;		     	//���� 7λ��ɱ�־ 6bit ����ʼ��־
volatile u8 TIM3_Cap4STA = 0;		     	//���� 7λ��ɱ�־ 6bit ����ʼ��־
volatile u16 TIM3_Cap3Val = 0;	//
volatile u16 TIM3_Cap4Val = 0;	//
volatile u16 TIM3_Cap3temp = 0;	//
volatile u16 TIM3_Cap4temp = 0;	//
/*
 * ��������TIM2_NVIC_Configuration
 * ����  ��TIM2�ж����ȼ�����
 * ����  ����
 * ���  ����	
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
 * ��������TIM3_NVIC_Configuration
 * ����  ��TIM3�ж����ȼ�����
 * ����  ����
 * ���  ����	
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


/*TIM_Period--0xffff   TIM_Prescaler--71 -->�ж�����Ϊ1us*/
void TIM3_PWMIN_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM3_ICInitStructure;
	
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Period=0xffff;		 								/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) 10 */
    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* ʱ��Ԥ��Ƶ�� 72M/72=1us*/
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* ������Ƶ ����Ƶ*/
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//----------------------------TIM3ch3-ch3--------------------------------------
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM3_ICInitStructure.TIM_ICFilter = 0;//���˲�
	TIM_ICInit(TIM3,&TIM3_ICInitStructure);
	
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM3_ICInitStructure.TIM_ICFilter = 0;//���˲�
	TIM_ICInit(TIM3,&TIM3_ICInitStructure);
	
    TIM_ClearFlag(TIM3, TIM_FLAG_Update|TIM_FLAG_CC3|TIM_FLAG_CC4);		/* �������жϱ�־ */
    
	TIM_Cmd(TIM3, ENABLE);
	
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4,DISABLE);
}


/*TIM_Period--0xffff   TIM_Prescaler--71 -->�ж�����Ϊ1us*/
void TIM2_PWMIN_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;
	
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period=0xffff;		 				/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) 10 */
    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* ʱ��Ԥ��Ƶ�� 72M/72=1us*/
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 			/* ������Ƶ ����Ƶ*/
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		/* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	//----------------------------TIM2ch2-ch1--------------------------------------
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;//���˲�
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;//���˲�
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;//���˲�
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM2_ICInitStructure.TIM_ICFilter = 0;//���˲�
	TIM_ICInit(TIM2,&TIM2_ICInitStructure);
	
    TIM_ClearFlag(TIM2, TIM_FLAG_Update|TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4);						/* �������жϱ�־ */
    
	TIM_Cmd(TIM2, ENABLE);
	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,DISABLE);
}


//TIM2 ����RC�ź����벶׽
void TIM2_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)			//��ʱ�����
	{	//-----------------------------1----------------------------------------
		if(TIM2_Cap1STA&0x40)								//֮ǰ�Ѳ��񵽵�ѹ������
		{	
			TIM2_Cap1STA++; 
			if((TIM2_Cap1STA&0x0f)>=2)					
			{
				TIM2_Cap1STA = 0x80;						//ǿ����� 
				TIM2_Cap1Val = 0;						//ָʾ����	
				TIM2->CCER &=~(1<<1);						//CC1P=0 ����Ϊ�����ز���
			}
		}
		//-----------------------------2----------------------------------------
		if(TIM2_Cap2STA&0x40)								//֮ǰ�Ѳ��񵽵�ѹ������
		{	
			TIM2_Cap2STA++; 
			if((TIM2_Cap2STA&0x0f)>=2)					
			{
				TIM2_Cap2STA = 0x80;						//ǿ����� 
				TIM2_Cap2Val = 0;						//ָʾ����	
				TIM2->CCER &=~(1<<5);						//CC2P=0 ����Ϊ�����ز���
			}
		}
		//------------------------------3---------------------------------------
		if(TIM2_Cap3STA&0x40)								//֮ǰ�Ѳ��񵽵�ѹ������
		{	
			TIM2_Cap3STA++; 
			if((TIM2_Cap3STA&0x0f)>=2)					
			{
				TIM2_Cap3STA = 0x80;						//ǿ����� 
				TIM2_Cap3Val = 0;						//ָʾ����	
				TIM2->CCER &=~(1<<9);						//CC3P=0 ����Ϊ�����ز���
			}
		}
		//------------------------------4---------------------------------------
		if(TIM2_Cap4STA&0x40)								//֮ǰ�Ѳ��񵽵�ѹ������
		{	
			TIM2_Cap4STA++; 
			if((TIM2_Cap4STA&0x0f)>=2)					
			{
				TIM2_Cap4STA = 0x80;						//ǿ����� 
				TIM2_Cap4Val = 0;						//ָʾ����	
				TIM2->CCER &=~(1<<13);						//CC4P=0 ����Ϊ�����ز���
			}
		}
	}
	//---------------------------------A-----------------------------------------
	if(TIM_GetITStatus(TIM2,TIM_IT_CC1)!=RESET) 		//���񵽵�ѹ�ؿ�ʼ
	{						
		if(TIM2_Cap1STA&0x40)								//�����½���
		{		
			TIM2_Cap1STA = 0x80;								//��ɱ��
			TIM2_Cap1Val = TIM2->CCR1;		//��֮ǰֵ����õ�����ʱ��
			if(TIM2_Cap1Val>TIM2_Cap1temp)
				TIM2_Cap1Val = TIM2_Cap1Val - TIM2_Cap1temp;
			else 
				TIM2_Cap1Val = 0xffff - TIM2_Cap1temp + TIM2_Cap1Val;
			
			TIM2->CCER &=~(1<<1); 								//CC1P=0 ����Ϊ�����ز���
		}
		else
		{
			TIM2_Cap1STA = 0;								//���㱣֤�´�ʹ�ò�����
			TIM2_Cap1Val = 0;								//
			TIM2_Cap1temp = TIM2->CCR1;						//��ȡ��ǰ�Ĳ���ֵ.
			TIM2_Cap1STA  = 0x40;							//���ÿ�ʼ��־
			TIM2->CCER|=1<<1; 								//CC1P=1 ����Ϊ�½��ز���
		}						
	} 
	//---------------------------------B-----------------------------------------
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2)!=RESET) 		//���񵽵�ѹ�ؿ�ʼ
	{	
		if(TIM2_Cap2STA&0x40)								//�����½���
		{		
			TIM2_Cap2STA = 0x80;								//��ɱ��
			TIM2_Cap2Val = TIM2->CCR2;		//��֮ǰֵ����õ�����ʱ��
			if(TIM2_Cap2Val>TIM2_Cap2temp)
				TIM2_Cap2Val = TIM2_Cap2Val - TIM2_Cap2temp;
			else 
				TIM2_Cap2Val = 0xffff - TIM2_Cap2temp + TIM2_Cap2Val;
			
			TIM2->CCER &=~(1<<5); 								//CC2P=0 ����Ϊ�����ز���
		}
		else
		{
			TIM2_Cap2STA = 0;								//���㱣֤�´�ʹ�ò�����
			TIM2_Cap2Val = 0;								//
			TIM2_Cap2temp = TIM2->CCR2;						//��ȡ��ǰ�Ĳ���ֵ.
			TIM2_Cap2STA  = 0x40;							//���ÿ�ʼ��־
			TIM2->CCER|=1<<5; 								//CC2P=1 ����Ϊ�½��ز���
		}
	} 
		//---------------------------------C-----------------------------------------
	if(TIM_GetITStatus(TIM2,TIM_IT_CC3)!=RESET) 		//���񵽵�ѹ�ؿ�ʼ
	{	
		if(TIM2_Cap3STA&0x40)								//�����½���
		{		
			TIM2_Cap3STA = 0x80;								//��ɱ��
			TIM2_Cap3Val = TIM2->CCR3;		//��֮ǰֵ����õ�����ʱ��
			if(TIM2_Cap3Val>TIM2_Cap3temp)
				TIM2_Cap3Val = TIM2_Cap3Val - TIM2_Cap3temp;
			else 
				TIM2_Cap3Val = 0xffff - TIM2_Cap3temp + TIM2_Cap3Val;
			
			TIM2->CCER &=~(1<<9); 								//CC3P=0 ����Ϊ�����ز���
		}
		else
		{
			TIM2_Cap3STA = 0;								//���㱣֤�´�ʹ�ò�����
			TIM2_Cap3Val = 0;								//
			TIM2_Cap3temp = TIM2->CCR3;						//��ȡ��ǰ�Ĳ���ֵ.
			TIM2_Cap3STA  = 0x40;							//���ÿ�ʼ��־
			TIM2->CCER|=1<<9; 								//CC3P=1 ����Ϊ�½��ز���
		}
	} 
		//---------------------------------D-----------------------------------------
	if(TIM_GetITStatus(TIM2,TIM_IT_CC4)!=RESET) 		//���񵽵�ѹ�ؿ�ʼ
	{	
		if(TIM2_Cap4STA&0x40)								//�����½���
		{		
			TIM2_Cap4STA = 0x80;								//��ɱ��
			TIM2_Cap4Val = TIM2->CCR4;		//��֮ǰֵ����õ�����ʱ��
			if(TIM2_Cap4Val>TIM2_Cap4temp)
				TIM2_Cap4Val = TIM2_Cap4Val - TIM2_Cap4temp;
			else 
				TIM2_Cap4Val = 0xffff - TIM2_Cap4temp + TIM2_Cap4Val;
			
			TIM2->CCER &=~(1<<13); 								//CC4P=0 ����Ϊ�����ز���
		}
		else
		{
			TIM2_Cap4STA = 0;								//���㱣֤�´�ʹ�ò�����
			TIM2_Cap4Val = 0;								//
			TIM2_Cap4temp = TIM2->CCR4;						//��ȡ��ǰ�Ĳ���ֵ.
			TIM2_Cap4STA  = 0x40;							//���ÿ�ʼ��־
			TIM2->CCER|=1<<13; 								//CC4P=1 ����Ϊ�½��ز���
		}
	} 
	TIM_ClearFlag(TIM2, TIM_FLAG_Update|TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4); //��־λ������������	��������	
}



//TIM3 ����RC�ź����벶׽
void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)			//��ʱ�����
	{	//---------------------------------3------------------------------------
		if(TIM3_Cap3STA&0x40)								//֮ǰ�Ѳ��񵽵�ѹ������
		{	
			TIM3_Cap3STA++; 
			if((TIM3_Cap3STA&0x0f)>=2)					
			{
				TIM3_Cap3STA = 0x80;						//ǿ����� 
				TIM3_Cap3Val = 0;						//ָʾ����	
				TIM3->CCER &=~(1<<9);						//CC1P=0 ����Ϊ�����ز���
			}
		}
		//---------------------------------4-----------------------------------------
		if(TIM3_Cap4STA&0x40)								//֮ǰ�Ѳ��񵽵�ѹ������
		{	
			TIM3_Cap4STA++; 
			if((TIM3_Cap4STA&0x0f)>=2)					
			{
				TIM3_Cap4STA = 0x80;						//ǿ����� 
				TIM3_Cap4Val = 0;						//ָʾ����	
				TIM3->CCER &=~(1<<13);						//CC1P=0 ����Ϊ�����ز���
			}
		}
	}
//---------------------------------C-----------------------------------------
	if(TIM_GetITStatus(TIM3,TIM_IT_CC3)!=RESET) 		//���񵽵�ѹ�ؿ�ʼ
	{	
		if(TIM3_Cap3STA&0x40)								//�����½���
		{		
			TIM3_Cap3STA = 0x80;								//��ɱ��
			TIM3_Cap3Val = TIM3->CCR3;		//��֮ǰֵ����õ�����ʱ��
			if(TIM3_Cap3Val>TIM3_Cap3temp)
				TIM3_Cap3Val = TIM3_Cap3Val - TIM3_Cap3temp;
			else 
				TIM3_Cap3Val = 0xffff - TIM3_Cap3temp + TIM3_Cap3Val;
			
			TIM3->CCER &=~(1<<9); 								//CC3P=0 ����Ϊ�����ز���
		}
		else
		{
			TIM3_Cap3STA = 0;								//���㱣֤�´�ʹ�ò�����
			TIM3_Cap3Val = 0;								//
			TIM3_Cap3temp = TIM3->CCR3;						//��ȡ��ǰ�Ĳ���ֵ.
			TIM3_Cap3STA  = 0x40;							//���ÿ�ʼ��־
			TIM3->CCER|=1<<9; 								//CC3P=1 ����Ϊ�½��ز���
		}
	} 
//---------------------------------D-----------------------------------------
	if(TIM_GetITStatus(TIM3,TIM_IT_CC4)!=RESET) 		//���񵽵�ѹ�ؿ�ʼ
	{	
		if(TIM3_Cap4STA&0x40)								//�����½���
		{		
			TIM3_Cap4STA = 0x80;								//��ɱ��
			TIM3_Cap4Val = TIM3->CCR4;		//��֮ǰֵ����õ�����ʱ��
			if(TIM3_Cap4Val>TIM3_Cap4temp)
				TIM3_Cap4Val = TIM3_Cap4Val - TIM3_Cap4temp;
			else 
				TIM3_Cap4Val = 0xffff - TIM3_Cap4temp + TIM3_Cap4Val;
			
			TIM3->CCER &=~(1<<13); 								//CC4P=0 ����Ϊ�����ز���
		}
		else
		{
			TIM3_Cap4STA = 0;								//���㱣֤�´�ʹ�ò�����
			TIM3_Cap4Val = 0;								//
			TIM3_Cap4temp = TIM3->CCR4;						//��ȡ��ǰ�Ĳ���ֵ.
			TIM3_Cap4STA  = 0x40;							//���ÿ�ʼ��־
			TIM3->CCER|=1<<13; 								//CC4P=1 ����Ϊ�½��ز���
		}
	} 
	TIM_ClearFlag(TIM3, TIM_FLAG_Update|TIM_FLAG_CC3|TIM_FLAG_CC4); //��־λ������������
}	
	
	
