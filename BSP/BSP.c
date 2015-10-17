#include "includes.h"
#include "Project_cfg.h"


/*
 * ��������BSP_Init
 * ����  ��ʱ�ӳ�ʼ����Ӳ����ʼ��
 * ����  ����
 * ���  ����
 */
void BSP_Init(void)
{
	SystemInit();				/* ����ϵͳʱ��Ϊ72M */	
	SysTick_init();				/* ��ʼ����ʹ��SysTick��ʱ�� */
	LED_BUTTON_GPIO_Config();  	/* LED �� ������ʼ�� */
	LED1(ON);
	LED2(ON);
	LED3(ON);
	LED4(ON);	
	Delay_ms(2000);				//��Ҫ�ȴ��ⲿ�豸��ʼ����
	LED1(OFF);
	LED2(OFF);
	LED3(OFF);
	LED4(OFF);	
	UART4_Config(115200);		/* ����4��ʼ�� */
	USART1_Config(115200);		/* ����1��ʼ�� */

	TIM1_GPIO_Config();
	My_I2C_Init();
	
	TIM2_PWMIN_GPIOInit();
	TIM2_PWMIN_Configuration();
	
	TIM3_PWMIN_GPIOInit();
	TIM3_PWMIN_Configuration();
	
#if	SONAR_ONBOARD
	SR04_GPIOInit();
	TIM4_PulseW_GPIOInit();
	TIM4_PulseW_Config();
#endif	
#if OPFLOW_SENSOR_ONBOARD_SPI_EN
	ADNS3080_SPIInit();
#endif	

	NRF24L01_Init(MODEL_RX, 40);
	EXTI_PC5_24L01_Init();
	
	EXTI_PC8_MPU6050_Init();
	EXTI_PC12_HMC5883_Init();

	DMA1CH5_NVIC_Configuration();
	TIM2_NVIC_Configuration();
	TIM3_NVIC_Configuration();
//	
#if	SONAR_ONBOARD
	TIM4_NVIC_Configuration();
#endif
//	EXTI5_NVIC_Configuration();
	#ifdef UART4_DEBUG
	UART4_NVIC_Configuration();
	#endif
	USART1_NVIC_Configuration();

}

/*
 * ��������SysTick_init
 * ����  ������SysTick��ʱ��
 * ����  ����
 * ���  ����
 */
void SysTick_init(void)
{
    SysTick_Config(SystemFrequency/OS_TICKS_PER_SEC);//��ʼ����ʹ��SysTick��ʱ��
}
