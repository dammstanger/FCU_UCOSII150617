//20141222
#include "spi.h"

//SPI 驱动 

//以下是SPI模块的初始化代码，配置成主机模式，访问SD Card/W25X16/24L01/JF24C							  
//SPI口初始化
//这里针是对SPI1的初始化
void SPI1_Init(void)
	{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1, ENABLE );

	//这里只针对SPI口初始化
	//PA5.6.7复用 	    
    //PA5.6.7上拉
	/* Configure SPI1 pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
		
	/* SPI1 configuration */	
	#if 1	/*  configuration for 24L01 and etc*/												
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//选择了串行时钟的稳态:时钟悬空低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//数据捕获于第一个时钟沿	(对24L01要设置 CPHA=0;CPOL=0;)
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC值计算的多项式
	#endif
	#if 0
	/*  configuration for ADNS3080 and etc*/	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							//空闲模式下SCK为1 CPOL=1//对ADNS3080要设置 CPHA=1;CPOL=1;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						//数据捕获于第一个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	
	SPI_InitStructure.SPI_CRCPolynomial = 7;	
	#endif
	
	SPI_Init(SPI1, &SPI_InitStructure);  								//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	/* Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设

	}  
 
//SPI 速度设置函数
//SpeedSet:
//SPI_SPEED_2   2分频   (SPI 36M@sys 72M)
//SPI_SPEED_4   4分频   (SPI 18M@sys 72M)
//SPI_SPEED_8  	8分频  	(SPI 9M@sys 72M)
//SPI_SPEED_16 	16分频 	(SPI 4.5@sys 72M)
//SPI_SPEED_32  32分频  (SPI 2.25M@sys 72M)
//SPI_SPEED_64  64分频  (SPI 1.125M@sys 72M)
//SPI_SPEED_128 128分频 (SPI 562.5K@sys 72M)
//SPI_SPEED_256 256分频 (SPI 281.25K@sys 72M)
void SPI1_SetSpeed(u8 SpeedSet)
	{
	SPI_Cmd(SPI1, DISABLE); //先关闭SPI外设，再修改。正在工作时是改不了的  

	SPI1->CR1&=0XFFC7;		
	if(SpeedSet==SPI_SPEED_2)//二分频
		{
		SPI1->CR1|=0<<3;//FPCLK=PCLK/2=36Mhz
		}
	else if(SpeedSet==SPI_SPEED_4)//八分频 
		{
		SPI1->CR1|=1<<3;//FPCLK=PCLK/8=9Mhz	
		}
	else if(SpeedSet==SPI_SPEED_8)//十六分频
		{
		SPI1->CR1|=2<<3;//FPCLK=PCLK/16=4.5Mhz
		}
	else if(SpeedSet==SPI_SPEED_16)//八分频 
		{
		SPI1->CR1|=3<<3;//FPCLK=PCLK/8=9Mhz	
		}
	else if(SpeedSet==SPI_SPEED_32)//十六分频
		{
		SPI1->CR1|=4<<3;//FPCLK=PCLK/16=4.5Mhz
		}
	else if(SpeedSet==SPI_SPEED_64)//十六分频
		{
		SPI1->CR1|=5<<3;//FPCLK=PCLK/16=4.5Mhz
		}
	else if(SpeedSet==SPI_SPEED_128)//十六分频
		{
		SPI1->CR1|=6<<3;//FPCLK=PCLK/16=4.5Mhz
		}
	else			 	 //256分频
		{
		SPI1->CR1|=7<<3; //FPCLK=PCLK/256=281.25Khz 低速模式
		}
	//SPI1->CR1|=1<<6; //SPI设备使能	
	/* Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设  
	} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
	{		
	u8 retry=0;				 
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据
	retry=0;
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1); //返回通过SPIx最近接收的数据				    
	}

/**************************************************************SPI2*******************************************************************/


//SPI2 驱动 

//以下是SPI2模块的初始化代码，配置成主机模式，						  
//SPI口初始化
//这里针是对SPI2的初始化
void SPI2_Init(void)
	{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2, ENABLE );		//FPCLK2 =72Mhz

	//这里只针对SPI口初始化
	//PA5.6.7复用 	    
    //PA5.6.7上拉
	/* Configure SPI2 pins: SCK 13, MISO 14and MOSI 15*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_SetBits(GPIOB,GPIO_Pin_13);

	/* SPI2 configuration */
	#if 0	/*  configuration for 24L01 and etc*/												
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//选择了串行时钟的稳态:时钟悬空低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//数据捕获于第一个时钟沿	(对24L01要设置 CPHA=0;CPOL=0;)
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC值计算的多项式
	#endif
	#if 1
	/*  configuration for ADNS3080 and etc*/	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							//空闲模式下SCK为1 CPOL=1//对ADNS3080要设置 CPHA=1;CPOL=1;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						//数据捕获于第一个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;		
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	
	SPI_InitStructure.SPI_CRCPolynomial = 7;	
	#endif
	
	SPI_Init(SPI2, &SPI_InitStructure);  								//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	/* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设

	}  
 
//SPI 速度设置函数
//SpeedSet:
//SPI_SPEED_2   2分频   (SPI 36M@sys 72M)
//SPI_SPEED_4   4分频   (SPI 18M@sys 72M)
//SPI_SPEED_8  	8分频  	(SPI 9M@sys 72M)
//SPI_SPEED_16 	16分频 	(SPI 4.5@sys 72M)
//SPI_SPEED_32  32分频  (SPI 2.25M@sys 72M)
//SPI_SPEED_64  64分频  (SPI 1.125M@sys 72M)
//SPI_SPEED_128 128分频 (SPI 562.5K@sys 72M)
//SPI_SPEED_256 256分频 (SPI 281.25K@sys 72M)
void SPI2_SetSpeed(u8 SpeedSet)
	{
	SPI_Cmd(SPI2, DISABLE); //先关闭SPI外设，再修改。正在工作时是改不了的  

	SPI2->CR1&=0XFFC7;		
	if(SpeedSet==SPI_SPEED_2)//二分频
		{
		SPI2->CR1|=0<<3;//FPCLK=PCLK/2=36Mhz	
		}
	else if(SpeedSet==SPI_SPEED_4)//八分频 
		{
		SPI2->CR1|=1<<3;//FPCLK=PCLK/8=9Mhz	
		}
	else if(SpeedSet==SPI_SPEED_8)//十六分频
		{
		SPI2->CR1|=2<<3;//FPCLK=PCLK/16=4.5Mhz
		}
	else if(SpeedSet==SPI_SPEED_16)//八分频 
		{
		SPI2->CR1|=3<<3;//FPCLK=PCLK/8=9Mhz	
		}
	else if(SpeedSet==SPI_SPEED_32)//十六分频
		{
		SPI2->CR1|=4<<3;//FPCLK=PCLK/16=4.5Mhz
		}
	else if(SpeedSet==SPI_SPEED_64)//十六分频
		{
		SPI2->CR1|=5<<3;//FPCLK=PCLK/16=4.5Mhz
		}
	else if(SpeedSet==SPI_SPEED_128)//十六分频
		{
		SPI2->CR1|=6<<3;//FPCLK=PCLK/16=4.5Mhz
		}
	else			 	 //256分频
		{
		SPI2->CR1|=7<<3; //FPCLK=PCLK/256=281.25Khz 低速模式
		}
	//SPI2->CR1|=1<<6; //SPI设备使能	
	/* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设  
	} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
	{		
	u8 retry=0;				 
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	/* Send byte through the SPI2 peripheral */
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
	retry=0;
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI2); //返回通过SPI2最近接收的数据				    
	}


	
