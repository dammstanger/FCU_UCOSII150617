#include "includes.h"
#include "Project_cfg.h"


/*
 * 函数名：BSP_Init
 * 描述  ：时钟初始化、硬件初始化
 * 输入  ：无
 * 输出  ：无
 */
void BSP_Init(void)
{
	SystemInit();				/* 配置系统时钟为72M */	
	SysTick_init();				/* 初始化并使能SysTick定时器 */
	LED_BUTTON_GPIO_Config();  	/* LED 和 按键初始化 */
	LED1(ON);
	LED2(ON);
	LED3(ON);
	LED4(ON);	
	Delay_ms(2000);				//主要等待外部设备初始化好
	LED1(OFF);
	LED2(OFF);
	LED3(OFF);
	LED4(OFF);	
	UART4_Config(115200);		/* 串口4初始化 */
	USART1_Config(115200);		/* 串口1初始化 */

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
 * 函数名：SysTick_init
 * 描述  ：配置SysTick定时器
 * 输入  ：无
 * 输出  ：无
 */
void SysTick_init(void)
{
    SysTick_Config(SystemFrequency/OS_TICKS_PER_SEC);//初始化并使能SysTick定时器
}
