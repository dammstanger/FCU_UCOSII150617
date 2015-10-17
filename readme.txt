/**------------------readme---------------------*
  * @file    工程说明文档                       *
  * @author  DammStanger                        *
  * @version V3.0.0；                           *
  * @date    2015.6.17；                       *
  * @brief   说明；                             *
  *---------------------------------------------*

  *本工程的芯片是STM32RCT6                      *
  *---------------------------------------------*
  *个人建议：                                   *
  *编译文档时用keil编译。                       *
  *---------------------------------------------**/
RCC_APB1PeriphClockCmd
RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4,
RCC_APB1Periph_TIM5, RCC_APB1Periph_TIM6, RCC_APB1Periph_TIM7,
RCC_APB1Periph_WWDG, RCC_APB1Periph_SPI2, RCC_APB1Periph_SPI3,
RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_USART4, 
RCC_APB1Periph_USART5, RCC_APB1Periph_I2C1, RCC_APB1Periph_I2C2,
RCC_APB1Periph_USB, RCC_APB1Periph_CAN1, RCC_APB1Periph_BKP,
RCC_APB1Periph_PWR, RCC_APB1Periph_DAC, RCC_APB1Periph_CEC,
RCC_APB1Periph_TIM12, RCC_APB1Periph_TIM13, RCC_APB1Periph_TIM14
RCC_APB2PeriphClockCmd
RCC_APB2Periph_AFIO, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB,
  *          RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE,
  *          RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG, RCC_APB2Periph_ADC1,
  *          RCC_APB2Periph_ADC2, RCC_APB2Periph_TIM1, RCC_APB2Periph_SPI1,
  *          RCC_APB2Periph_TIM8, RCC_APB2Periph_USART1, RCC_APB2Periph_ADC3,
  *          RCC_APB2Periph_TIM15, RCC_APB2Periph_TIM16, RCC_APB2Periph_TIM17,
  *          RCC_APB2Periph_TIM9, RCC_APB2Periph_TIM10, RCC_APB2Periph_TIM11   

上一版：FCU_UCOSII150609增加油门模式
1.加入油门模式，
2.将原来的Task_SonarMeasure 任务改名为，Task_ThrottleLoop，并在此进行油门模式的更新和高度高级控制器的运行。
3.增加了auto 飞行模式作为自动起降模式
4.将高度控制器从Control（）函数中取出，提前执行，并将最终的油门数据作为control（）的输入函数。
5.重新制定了飞行模式的状态机，并调整了LED对相关飞行模式的表示。
6.将起始任务挂起，重新启用LED任务，并赋予飞行模式指示功能。
7.将光流对位移的积分放入了是否执行的if判断中。
8.将遥控的输入角度转换从Control（）中取出，以Add_RC_Roll（）返回遥控的输入值，放入FlyModeProcess()中叠加到控制过程角度中。
9.将各种输入角度从角度计算函数如：Add_Optflow_Roll()、Add_RC_Roll()等函数中提出放入函数外。


本版：FCU_UCOSII150617自助功能完善
