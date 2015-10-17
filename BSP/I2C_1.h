/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名		：I2C_1.h
 * 描	述		：STM32通用模拟IIC驱动
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.2.150409
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本		：ST3.0.0
 * 创建时间	：2014.8.31
 * 最后编辑	：2015.4.9
 * 备			注：根据FCU飞控程序中的IIC驱动修改，简化了时序,在野火开发板中测试好使,
							V1.2版重新恢复对硬件的IIC ，并对IIC的初始化做了修改，预重置参数，使其能正常工作 **-------------------------------------------------------------------------------
**-------------------------------------------------------------------------------

 * 作	者		：Damm Stanger
 * 邮	箱		：dammstanger@qq.com
**********************************************************************************************/
#ifndef _I2C_1_H_
#define	_I2C_1_H_


#include "Project_cfg.h"
#include "stm32f10x.h"


//模拟IIC硬件IIC
//#define SOFT_I2C
#define HARDWARE_I2C

//IIC 速率
#define I2C_Speed              400000											//定义IIC速率为400K

#ifdef SOFT_I2C
/******************************************************/


//IO
#define IIC_GPIO_CLK RCC_APB2Periph_GPIOB 
#define IIC_GPIO     GPIOB 
#define IIC_SCL_PIN  GPIO_Pin_8  //
#define IIC_SDA_PIN  GPIO_Pin_9  //


//IO方向设置
#define SDA_IN()  {IIC_GPIO->BSRR|=IIC_SDA_PIN;IIC_GPIO->CRH&=0XFFFFFF0F;IIC_GPIO->CRH|=8<<4;} 	//Pin9 在上拉输入模式
#define SDA_OUT() {IIC_GPIO->CRH&=0XFFFFFF0F;IIC_GPIO->CRH|=5<<4;} 															//Pin9 在开漏输出模式 速度10MHz

#define IIC_SCL(a)   if (a)  \
				 	 IIC_GPIO->BSRR|=IIC_SCL_PIN;else IIC_GPIO->BRR|=IIC_SCL_PIN //在输出模式(MODE[1:0]>00)： 
#define IIC_SDA(a)   if (a)  \
				 	 IIC_GPIO->BSRR|=IIC_SDA_PIN;else IIC_GPIO->BRR|=IIC_SDA_PIN //在输出模式(MODE[1:0]>00)： 

#define READ_SDA	 GPIOB->IDR  & IIC_SDA_PIN		//等同于GPIO_ReadInputDataBit(IIC_GPIO,IIC_SDA_PIN)


#endif

//void I2C_GPIO_Config(void);
void My_I2C_Init(void);
u8 IIC1_MultRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead);
u8 IIC1_ReadByte(u8 slaveAddr, u8 readAddr);
bool IIC1_WriteByte(u8 slaveAddr,u8 REG_Address,u8 REG_data);
u8 IIC1_MultWrite(u8 slaveAddr, u8* pBuffer, u8 wrtieAddr, u16 NumByteToWrite);

#endif

