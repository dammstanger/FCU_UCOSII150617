/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名		：I2C_1.c
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





#include "I2C_1.h" 
/***********************************模拟IIC*********************************************/
#ifdef SOFT_I2C

void delay_us(u8 n)   // 72MHz下调试准确
{ u16 a =n*10;	
 for(;a>0;a--);
    
}


static void I2C_delay(void)	//i=14 约3us
{
    volatile int i = 14; //40;  //
    while (i)
        i--;
}

/*
 * 函数名：I2C_GPIO_Config
 * 描述  ：I2C1 I/O配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */

void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(IIC_GPIO_CLK, ENABLE );	
	GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;// 配置 CLK 和 SDA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//GPIO_Mode_Out_PP;//   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;//GPIO_Speed_50MHz;
	GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(IIC_GPIO,IIC_SCL_PIN | IIC_SDA_PIN);
 
	IIC_SCL(1);//空闲时总线为高
	IIC_SDA(1);
}


//产生IIC起始信号
static bool IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	I2C_delay();
 	IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
	I2C_delay();
	IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
	return TRUE ;
}	  
//产生IIC停止信号
static void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
	IIC_SCL(1); 
	I2C_delay();
	IIC_SDA(1);//发送I2C总线结束信号
	I2C_delay();
}
//等待应答信号到来
//返回值：FALSE，接收应答失败
//        TRUE，接收应答成功
static bool IIC_Wait_Ack(void)
{
	SDA_IN();      //SDA设置为输入  
	IIC_SCL(1);
	I2C_delay();
	if(READ_SDA)
	{
		IIC_SCL(0);
		I2C_delay();
        return FALSE;
	}
	IIC_SCL(0);//时钟输出0 	 
	I2C_delay();	
	return TRUE;  
} 
//产生ACK应答
static void IIC_Ack(void)
{
	SDA_OUT();
	IIC_SDA(0);
	IIC_SCL(1);
	I2C_delay();
	IIC_SCL(0);
	I2C_delay();
}
//不产生ACK应答		    
static void IIC_NAck(void)
{
	SDA_OUT();
	IIC_SDA(1);
	IIC_SCL(1);
    I2C_delay();
	IIC_SCL(0);
	I2C_delay();
}					 	

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
static void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
   	SDA_OUT(); 	    
    for(t=0;t<8;t++)
    {   
        IIC_SDA((txd&0x80)>>7);
        txd<<=1; 
		IIC_SCL(1);		
		I2C_delay();
		IIC_SCL(0);
		I2C_delay();
	}	 
} 	

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
static u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA(1);
	SDA_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
        receive<<=1;
        IIC_SCL(1);
		I2C_delay();
		if(READ_SDA)receive |= 0x01;
		IIC_SCL(0);
		I2C_delay();      
	}
    if (ack)
		IIC_Ack(); //发送ACK   
    else
		IIC_NAck();//发送nACK 
    return receive;
}

bool IIC1_WriteByte(u8 slaveAddr,u8 REG_Address,u8 REG_data)
{	bool retval;
	IIC_Start();                   	//起始信号
	IIC_Send_Byte(slaveAddr);   	//发送设备地址+写信号
	retval=IIC_Wait_Ack();	   
	IIC_Send_Byte(REG_Address);    	//内部寄存器地址
		IIC_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(REG_data);       	//内部寄存器数据
		IIC_Wait_Ack(); 	 											  		   
	IIC_Stop();                    	//发送停止信号
	return retval;
}


u8 IIC1_ReadByte(u8 slaveAddr, u8 readAddr)
{
	u8 REG_data;
	IIC_Start();                   	//起始信号
	IIC_Send_Byte(slaveAddr);    	//发送设备地址+写信号
	IIC_Wait_Ack();
	IIC_Send_Byte(readAddr);     	//发送存储单元地址，从0开始
	IIC_Wait_Ack();					//必须要有
	IIC_Start();                   	//起始信号
	IIC_Send_Byte(slaveAddr+1);  	//发送设备地址+读信号
	IIC_Wait_Ack();
	REG_data=IIC_Read_Byte(0);    	//读出寄存器数据不继续再读,发送NACK  
	return REG_data;	
}

u8 IIC1_MultRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
	IIC_Start();                   //起始信号 				 
	IIC_Send_Byte(slaveAddr);				//发送写器件指令	 
	if(FALSE==IIC_Wait_Ack())
	{
		IIC_Stop();                    		//发送停止信号
		return 1;
	}	IIC_Send_Byte(readAddr);   			//发送寄存器地址
	IIC_Wait_Ack(); 	 										  		   
	IIC_Start();  	 	   					//重新启动
	IIC_Send_Byte(slaveAddr+1);				//发送读器件指令
	IIC_Wait_Ack();
	/* While there is data to be read */
	while(NumByteToRead)
	{
		if(NumByteToRead == 1)
		{
			*pBuffer=IIC_Read_Byte(0);    	//读出寄存器数据不继续再读,发送NACK  
		}
		else 
		{
			*pBuffer=IIC_Read_Byte(1);    	//读出寄存器数据继续读,发送ACK  
			pBuffer++;						/* Point to the next location where the byte read will be saved */
		}
		/* Decrement the read bytes counter */
		NumByteToRead--;
	}
	IIC_Stop();                    			//停止信号
	return 0;
}

u8 IIC1_MultWrite(u8 slaveAddr, u8* pBuffer, u8 wrtieAddr, u16 NumByteToWrite)
{
	IIC_Start();					  		//起始信号

	IIC_Send_Byte(slaveAddr);   			//发送设备地址+写信号
		if(FALSE==IIC_Wait_Ack())
		{
			IIC_Stop();                    	//发送停止信号
			return 1;
		}
	IIC_Send_Byte(wrtieAddr);				//内部寄存器地址
		IIC_Wait_Ack();
	while(NumByteToWrite)
	{
		IIC_Send_Byte(*pBuffer);       		//内部寄存器数据		
		if(FALSE==IIC_Wait_Ack())
		{
			IIC_Stop();						//发送停止信号
			return 1;
		}
		pBuffer++;
		NumByteToWrite--;
	} 										  		   
	IIC_Stop();                    			//发送停止信号
	return 0;
}


#else
/***********************************硬件IIC*********************************************/


/*
 * 函数名：I2C_GPIO_Config
 * 描述  ：I2C1 I/O配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */

void I2C_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	/* 使能与 I2C1 有关的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);//复用IO时钟开启

	/*IIC remapIO config*/
	GPIO_PinRemapConfig(GPIO_Remap_I2C1 , ENABLE);
	
	/* PB8-I2C1_SCL、PB9-I2C1_SDA*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;	       // 复用开漏输出 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
 * 函数名：I2C_Configuration
 * 描述  ：I2C 工作模式配置
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
void I2C_Mode_Config(void)
{
	I2C_InitTypeDef  I2C_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);  

	I2C_DeInit(I2C1);																		//复位外设，一定要有
	/* I2C 配置 */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0;			//在STM32作为从机时的应答地址，做主机时不需理会
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;

	/* 使能 I2C1 */
	I2C_Cmd(I2C1, ENABLE);			//PE = 1;
	
	/* I2C1 初始化 */
	I2C_Init(I2C1, &I2C_InitStructure);

//	/*允许1字节1应答模式*/
//	I2C_AcknowledgeConfig(I2C1, ENABLE);    
}

/***************************************************************************
 * 函数名：IIC1_WriteByte
 * 描述  ：向I2C设备写入一个字节数据
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
***************************************************************************/

bool IIC1_WriteByte(u8 slaveAddr,u8 REG_Address,u8 REG_data)
{
	bool retval;
	/* Send STRAT condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
//	 I2C_Cmd(I2C1, ENABLE);							//20140309
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	  
	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, REG_Address);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send the byte to be written */
	I2C_SendData(I2C1, REG_data); 

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);	
	
	return retval;
}


/***************************************************************************
 * 函数名：IIC1_MultWrite
 * 描述  ：向I2C设备写入多个字节数据
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
***************************************************************************/
u8 IIC1_MultWrite(u8 slaveAddr, u8* pBuffer, u8 wrtieAddr, u16 NumByteToWrite)
{
	/* Send STRAT condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
//	 I2C_Cmd(I2C1, ENABLE);							//20140309
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	  
	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, wrtieAddr);

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	 /* While there is data to be written */
  while(NumByteToWrite--)  
  {
    /* Send the current byte */
    I2C_SendData(I2C1, *pBuffer); 

    /* Point to the next byte to be written */
    pBuffer++; 
  
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  }

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);		
}

/***************************************************************************
 * 函数名：IIC1_ReadByte
 * 描述  ：IIC1 读出一字节
 * 输入  ：
 * 输出  ：
 * 调用  ：外部调用
***************************************************************************/

u8 IIC1_ReadByte(u8 slaveAddr, u8 readAddr)
{
	u8 readval = 0x55;
	//*((u8 *)0x4001080c) |=0x80; 
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008


	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);
	//*((u8 *)0x4001080c) &=~0x80;

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
//	I2C_Cmd(I2C1, ENABLE);							//20140309
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);							

	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, readAddr);  

	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STRAT condition a second time */  
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);


	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	/* Disable Acknowledgement */
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	
	/* Send STOP Condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
	  
	/* Test on EV7 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)); 
	    
	/* Read a byte from the EEPROM */
	readval = I2C_ReceiveData(I2C1);   
	
	/* Enable Acknowledgement to be ready for another reception */
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return readval;
}

/***************************************************************************
 * 函数名：
 * 描述  ：
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
***************************************************************************/

u8 IIC1_MultRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
 // ENTR_CRT_SECTION();

  /* While the bus is busy */
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Transmitter); 

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2C1, ENABLE);

  /* Send the MPU6050's internal address to write to */
  I2C_SendData(I2C1, readAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(I2C1, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for read */
  I2C_Send7bitAddress(I2C1, slaveAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2C1, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(I2C1, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MPU6050 */
      *pBuffer = I2C_ReceiveData(I2C1);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C1, ENABLE);
//  EXT_CRT_SECTION();
return 0;
}

#endif



void My_I2C_Init()
{
	I2C_GPIO_Config(); 
#ifdef	HARDWARE_I2C
	I2C_Mode_Config();
#endif
}






