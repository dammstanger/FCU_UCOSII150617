/****************************************Copyright (c)****************************************************
**                                       辽宁科技大学
**                                     
**                                         电子协会
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           nrf24l01.c
** Last modified Date: 
** Last Version: 
** Description:         NRF24L01驱动源文件
** 
**--------------------------------------------------------------------------------------------------------
** Created By:          王恺
** Created date:        2014/09/11
** Version:             V1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Description:
**
*********************************************************************************************************/


/*********************************************************************************************************
	包含头文件
*********************************************************************************************************/
#include "24l01.h"
#include "spi.h"
//#include "stm32f10x_exti.h" 


/*********************************************************************************************************
	变量定义
*********************************************************************************************************/
u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址

/*********************************************************************************************************
	申明需要使用的内部函数
*********************************************************************************************************/
void NRF24L01_PortInit(void);


/*********************************************************************************************************
** Function name:       NRF24L01_PortInit
** Descriptions:        NRF24L01端口配置
** input parameters:    none
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void NRF24L01_PortInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	  //使能PORTA口时钟 
  //使能PORTB口时钟 
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC, ENABLE );	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	SPI_CE_L();
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	SPI_CSN_H();
		
	SPI1_Init();    //初始化SPI
	SPI_CE_L(); 	//使能24L01
	SPI_CSN_H();	//SPI片选取消	
//	/*
//	 *	配置MCO作为24L01时钟源
//	 */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	RCC_MCOConfig(RCC_MCO_HSE);
	
	/*
	 *	配置IRQ外部中断
	 */
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
  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能IRQ所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级2， 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;					//子优先级3
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure); 
}


/*********************************************************************************************************
** Function name:       NRF24L01_Write_Reg
** Descriptions:        NRF24L01写寄存器
** input parameters:    reg:	寄存器地址
**						value:	值
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	SPI_CSN_L();					  /* 选通器件 */
	status = SPI1_ReadWriteByte(reg);  /* 写寄存器地址 */
	SPI1_ReadWriteByte(value);		  /* 写数据 */
	SPI_CSN_H();					  /* 禁止该器件 */
	return 	status;
}


/*********************************************************************************************************
** Function name:       NRF24L01_Read_Reg
** Descriptions:        NRF24L01写寄存器
** input parameters:    reg:	寄存器地址
** output parameters:   none
** Returned value:      读到的值
*********************************************************************************************************/
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	SPI_CSN_L();					  /* 选通器件 */
	SPI1_ReadWriteByte(reg);			  /* 写寄存器地址 */
	reg_val = SPI1_ReadWriteByte(0);	  /* 读取该寄存器返回数据 */
	SPI_CSN_H();					  /* 禁止该器件 */
    return 	reg_val;
}


/*********************************************************************************************************
** Function name:       NRF24L01_Write_Buf
** Descriptions:        NRF24L01写缓冲区
** input parameters:    reg:	缓冲区首地址
**						value:	数据首地址
**						uchars:	数据长度
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();				        /* 选通器件 */
	status = SPI1_ReadWriteByte(reg);	/* 写寄存器地址 */
	for(i=0; i<uchars; i++)
	{
		SPI1_ReadWriteByte(pBuf[i]);		/* 写数据 */
	}
	SPI_CSN_H();						/* 禁止该器件 */
    return 	status;	
}


/*********************************************************************************************************
** Function name:       NRF24L01_Read_Buf
** Descriptions:        NRF24L01读缓冲区
** input parameters:    reg:	缓冲区首地址
**						value:	数据首地址
**						uchars:	数据长度
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();						/* 选通器件 */
	status = SPI1_ReadWriteByte(reg);	/* 写寄存器地址 */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = SPI1_ReadWriteByte(0); /* 读取返回数据 */ 	
	}
	SPI_CSN_H();						/* 禁止该器件 */
    return 	status;
}


/*********************************************************************************************************
** Function name:       NRF24L01_TxPacket
** Descriptions:        NRF24L01发送数据包
** input parameters:    tx_buf:	数据首地址
**						len:	数据长度
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void NRF24L01_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
	SPI_CE_L();		 //StandBy I模式	
	
//	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // 装载数据	
	SPI_CE_H();		 //置高CE，激发数据发送
}


void NRF24L01_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
	SPI_CE_L();		 //StandBy I模式	
	NRF24L01_Write_Buf(0xa8, tx_buf, len); 			 // 装载数据
	SPI_CE_H();		 //置高CE
}


/*********************************************************************************************************
** Function name:       NRF24L01_RxPacket
** Descriptions:        NRF24L01接收数据包
** input parameters:    tx_buf:	数据首地址
** output parameters:   none
** Returned value:      0，接收完成；其他，错误代码
*********************************************************************************************************/
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	sta=NRF24L01_Read_Reg(NRFRegSTATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}
	return 1;//没收到任何数据
}


/*********************************************************************************************************
** Function name:       Nrf24l01_Check
** Descriptions:        NRF24L01自检
** input parameters:    none
** output parameters:   none
** Returned value:      0:ERROR		others:SUCCESS
*********************************************************************************************************/
u8 NRF24L01_Check(void)
{ 
	u8 buf1[5]; 
	u8 i; 
	/*写入5个字节的地址. */ 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	/*读出写入的地址 */ 
	NRF24L01_Read_Buf(TX_ADDR,buf1,5); 
	/*比较*/ 
	for(i=0;i<5;i++) 
	{ 
		if(buf1[i]!=TX_ADDRESS[i]) 
			break; 
	} 
	if(i==5)
		return SUCCESS ; //MCU与NRF成功连接 
	else
		return ERROR ; //MCU与NRF不正常连接 
}


/*********************************************************************************************************
** Function name:       NRF24L01_Init
** Descriptions:        NRF24L01初始化
** input parameters:    model:	模式选择
** output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void NRF24L01_Init(u8 model, u8 ch)
{
	NRF24L01_PortInit();
	SPI1_Init();
	
	SPI_CE_L();
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址 
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		//写TX节点地址  
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x00);							//禁止所有通道的自动应答 
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);						//使能通道0的接收地址 
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x00);						//禁止自动重发
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,ch);								//设置RF通道为CHANAL
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 						//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	NRF24L01_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,MAX_TX|TX_OK|RX_OK); 		//清除中断
/////////////////////////////////////////////////////////
	if(model==1)			 //RX
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
	}
	else if(model==2)		//TX
	{
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x3e);   		 // IRQ收完成中断开启,16位CRC,主发送
	}
	else if(model==3)		//RX2
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
		
		SPI1_ReadWriteByte(0x50);
		SPI1_ReadWriteByte(0x73);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
	}
	else								//TX2
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		
		SPI1_ReadWriteByte(0x50);
		SPI1_ReadWriteByte(0x73);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x07);
	}
	SPI_CE_H();
}


void NRF24L01_Tx(void)
{
	SPI_CE_L();
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x3e);   		 // IRQ收完成中断开启,16位CRC,主发送
	SPI_CE_H();
}


void NRF24L01_Rx(void)
{
	u8 sta = 0;
	while(!(sta&TX_OK))							//等待发送完成
	{
		sta=NRF24L01_Read_Reg(NRFRegSTATUS);  	//读取状态寄存器的值
	}
	SPI_CE_L();
	NRF24L01_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,sta); 			//清除TX_DS或MAX_RT中断标志
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x3f);   		 	// IRQ收完成中断开启,16位CRC,主接收
	SPI_CE_H();
}
