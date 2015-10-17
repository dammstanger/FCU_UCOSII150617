//20140416
#include "SR04.h"
#include "SysTick.h"

#if	SONAR_ONBOARD

/**********************************
TIMER4��CAPture ����������
***********************************/

volatile u8  TIM4_Cap2STA = 0;		     	//���� 7λ��ɱ�־ 6bit ����ʼ��־
volatile u16 TIM4_Cap2Val = 0;	//
volatile u16 TIM4_Cap2temp = 0;	//


void SR04_GPIOInit()	//PB6����Ϊ���䴥��
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
 * ��������TIM4_NVIC_Configuration
 * ����  ��TIM4�ж����ȼ�����
 * ����  ����
 * ���  ����	
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

/*TIM_Period--0xffff   TIM_Prescaler--71 -->�ж�����Ϊ1us*/
void TIM4_PulseW_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM4_ICInitStructure;
	
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Period=0xffff;		 								/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) 10 */
    /* �ۼ� TIM_Period��Ƶ�ʺ����һ�����»����ж� */
    TIM_TimeBaseStructure.TIM_Prescaler= (72 - 1);				    /* ʱ��Ԥ��Ƶ�� 72M/72=1us*/
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		/* ������Ƶ ����Ƶ*/
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; /* ���ϼ���ģʽ */
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//----------------------------TIM4ch1--------------------------------------
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM4_ICInitStructure.TIM_ICFilter = 0;//���˲�
	TIM_ICInit(TIM4,&TIM4_ICInitStructure);
	
	
    TIM_ClearFlag(TIM4, TIM_FLAG_Update|TIM_FLAG_CC2);		/* �������жϱ�־ */
    
	TIM_Cmd(TIM4, ENABLE);
	
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,DISABLE);
}


void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)			//��ʱ�����
	{	//---------------------------------2------------------------------------
		if(TIM4_Cap2STA&0x40)								//֮ǰ�Ѳ��񵽵�ѹ������
		{	
			TIM4_Cap2STA++; 
			if((TIM4_Cap2STA&0x0f)>=2)					
			{
				TIM4_Cap2STA = 0x80;						//ǿ����� 
				TIM4_Cap2Val = 65535;							//ָʾ����	
				TIM4->CCER &=~(1<<5);						//CC1P=0 ����Ϊ�����ز���
				TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,DISABLE);
			}
		}
	}
	//---------------------------------B-----------------------------------------
	if(TIM_GetITStatus(TIM4,TIM_IT_CC2)!=RESET) 		//���񵽵�ѹ�ؿ�ʼ
	{	
		if(TIM4_Cap2STA&0x40)								//�����½���
		{		
			TIM4_Cap2STA = 0x80;								//��ɱ��
			TIM4_Cap2Val = TIM4->CCR2;		//��֮ǰֵ����õ�����ʱ��
			if(TIM4_Cap2Val>TIM4_Cap2temp)
				TIM4_Cap2Val = TIM4_Cap2Val - TIM4_Cap2temp;
			else 
				TIM4_Cap2Val = 0xffff - TIM4_Cap2temp + TIM4_Cap2Val;
			
			TIM4->CCER &=~(1<<5); 								//CC2P=0 ����Ϊ�����ز���
			TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,DISABLE);
		}
		else
		{
			TIM4_Cap2STA = 0;								//���㱣֤�´�ʹ�ò�����
			TIM4_Cap2Val = 0;								//
			TIM4_Cap2temp = TIM4->CCR2;						//��ȡ��ǰ�Ĳ���ֵ.
			TIM4_Cap2STA  = 0x40;							//���ÿ�ʼ��־
			TIM4->CCER|=1<<5; 								//CC2P=1 ����Ϊ�½��ز���
		}
	}
	TIM_ClearFlag(TIM4, TIM_FLAG_Update|TIM_FLAG_CC2); //��־λ������������
	
}


void SR04_Start()
{
	TIM_ClearFlag(TIM4, TIM_FLAG_Update|TIM_FLAG_CC2);						/* �������жϱ�־ */
	TIM_ITConfig(TIM4,TIM_IT_Update|TIM_IT_CC2,ENABLE);
	SR04_SRTTROGLE_H;
	delay_nus(20);
	SR04_SRTTROGLE_L;
}

u16 SR04_HightCal()
{
	static int hightlast = 0;
	int highttemp = 0;
	
	highttemp = (TIM4_Cap2Val*1.7)/10;//��λmm
//	if(highttemp>=500||highttemp<=0) highttemp = hightlast;
//	hightlast = highttemp;
	return highttemp;
}



#endif


