/****************************************Copyright (c)****************************************************
**                                       辽宁科技大学
**                                     
**                                         电子协会
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           24l01.h
** Last modified Date: 
** Last Version: 
** Description:         NRF24L01驱动头文件
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
#ifndef __24L01_H__
#define __24L01_H__


/*********************************************************************************************************
	包含头文件
*********************************************************************************************************/
#include "Project_cfg.h"
#include "stm32f10x.h"


/*********************************************************************************************************
	常量宏定义--NRF24L01内部寄存器指令
*********************************************************************************************************/
#define NRF_READ_REG	0x00  	// 读寄存器指令
#define NRF_WRITE_REG	0x20 	// 写寄存器指令
#define R_RX_PL_WID   	0x60
#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留


/*********************************************************************************************************
	常量宏定义--NRF24L01(SPI)内部寄存器地址
*********************************************************************************************************/
#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define NRFRegSTATUS    0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道1接收数据长度
#define RX_PW_P2        0x13  // 接收频道2接收数据长度
#define RX_PW_P3        0x14  // 接收频道3接收数据长度
#define RX_PW_P4        0x15  // 接收频道4接收数据长度
#define RX_PW_P5        0x16  // 接收频道5接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置


/*********************************************************************************************************
	常量宏定义--NRF24L01操作
*********************************************************************************************************/
#define RX_DR			6		//中断标志
#define TX_DS			5
#define MAX_RT			4
#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define MODEL_RX		1			//普通接收
#define MODEL_TX		2			//普通发送
#define MODEL_RX2		3			//接收模式2,用于双向传输
#define MODEL_TX2		4			//发送模式2,用于双向传输

#define RX_PLOAD_WIDTH  32  	
#define TX_PLOAD_WIDTH  10  	
#define TX_ADR_WIDTH    5 	 	
#define RX_ADR_WIDTH    5   	


/*********************************************************************************************************
	外部变量声明
*********************************************************************************************************/
//extern 	u8 	NRF24L01_RXDATA[RX_PLOAD_WIDTH];		//nrf24l01接收到的数据
//extern 	u8 	NRF24L01_TXDATA[RX_PLOAD_WIDTH];		//nrf24l01需要发送的数据


/*********************************************************************************************************
	申明需要使用的外部函数
*********************************************************************************************************/
extern void NRF24L01_Init(u8 model, u8 ch);				//初始化,model=1/2/3/4,ch为实用的通道号
extern void NRF24L01_TxPacket(uint8_t * tx_buf, uint8_t len);			//发送数据包,用于model 2/4
extern void NRF24L01_TxPacket_AP(uint8_t * tx_buf, uint8_t len);	//发送数据包,用于model 3
extern uint8_t NRF24L01_RxPacket(u8 *rxbuf);
extern uint8_t NRF24L01_Read_Reg(uint8_t reg);
extern uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);
extern uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
extern uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
extern uint8_t NRF24L01_Check(void);
void NRF24L01_Tx(void);
void NRF24L01_Rx(void);
#define NRF24l01_IRQ_ISLow()	!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)

/*********************************************************************************************************
	端口定义
*********************************************************************************************************/
#define SPI_CE_H()   GPIO_SetBits(GPIOC, GPIO_Pin_4) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOC, GPIO_Pin_4)

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_4)



#endif
