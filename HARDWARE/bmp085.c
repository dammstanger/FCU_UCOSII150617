
//20141101
//***************************************
// BMP085 IIC测试程序
// 参考宏晶网站24c04通信程序
// 时间：2014年11月01日
// DammStanger
// 注意：配合IIC驱动一起使用
//****************************************
#include "bmp085.h"
#include "I2C_1.h"

#include  <math.h>    //Keil library  
#include  <stdlib.h>  //Keil library  
#include  <stdio.h>   //Keil library	
  	
BMP085_PAR  bmpcal;


/******************************************************************************
/ 函数功能:读出BMP085内部数据,连续
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/

void BMP085_MultRead( u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{   
	IIC1_MultRead(BMP085_SlaveAddress,pBuffer,readAddr,NumByteToRead);
}

/******************************************************************************
/ 函数功能:向bmp085设备写入一个字节数据
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void BMP085_WriteByte(u8 REG_Address,u8 REG_data)
{
	
	IIC1_WriteByte(BMP085_SlaveAddress,REG_Address,REG_data);
}


/******************************************************************************
/ 函数功能:从bmp085设备启动温度转换
/ 修改日期:none
/ 输入参数:none
/ 输出参数:void
/ 使用说明:none
******************************************************************************/
void BMP085_StartTemp(void)
{	
	BMP085_WriteByte(BMP085_CONTRLREG,BMP085_STRTEMP);
}


/******************************************************************************
/ 函数功能:从bmp085设备读取温度
/ 修改日期:none
/ 输入参数:none
/ 输出参数:u32
/ 使用说明:none
******************************************************************************/
u32 BMP085_ReadTemp(void)
{	u32 temp=0;
	u8  Dat[2]={0};

	BMP085_MultRead(Dat,BMP085_VALREGMSB,2);
	temp = (Dat[0]<<8)|Dat[1];
	temp &= 0x0000FFFF;
	return temp ;
}


/******************************************************************************
/ 函数功能:从bmp085启动压力转换
/ 修改日期:none
/ 输入参数:none
/ 输出参数:void
/ 使用说明:none
******************************************************************************/
void BMP085_StartPress(void)
{	
	BMP085_WriteByte(BMP085_CONTRLREG,BMP085_PRESSMODE);
}


/******************************************************************************
/ 函数功能:从bmp085设备读取温度
/ 修改日期:none
/ 输入参数:none
/ 输出参数:u32
/ 使用说明:none
******************************************************************************/
u32 BMP085_ReadPress(void)
{	u32 press = 0;
	u8 Dat[2]={0};
	
	BMP085_MultRead(Dat,BMP085_VALREGMSB,2);
	press = (Dat[0]<<8)|Dat[1];
	press &= 0x0000FFFF;
	
	return press;	
}

/******************************************************************************
//函数功能:初始化BMP085，根据需要请参考pdf进行修改**************
/ 修改日期:none
/ 输入参数:none
/ 输出参数:u32
/ 使用说明:none
******************************************************************************/

void BMP085_Init()
{	u8 Dat[22];
	BMP085_MultRead(Dat,BMP085_CALIREG_AC1,11);
	bmpcal.ac1 = (Dat[0]<<8)|Dat[1];
	bmpcal.ac2 = (Dat[2]<<8)|Dat[3];
	bmpcal.ac3 = (Dat[4]<<8)|Dat[5];
	bmpcal.ac4 = (Dat[6]<<8)|Dat[7];
	
	bmpcal.ac5 = (Dat[8]<<8)|Dat[9];
	bmpcal.ac6 = (Dat[10]<<8)|Dat[11];
	bmpcal.b1  = (Dat[12]<<8)|Dat[13];
	bmpcal.b2  = (Dat[14]<<8)|Dat[15];
	bmpcal.mb  = (Dat[16]<<8)|Dat[17];
	bmpcal.mc  = (Dat[18]<<8)|Dat[19];
	bmpcal.md  = (Dat[20]<<8)|Dat[21];
	printf("calibval:%d,%d,%d,%d,%d,%d\r\n",bmpcal.ac1,bmpcal.ac2,bmpcal.ac3,bmpcal.ac4,bmpcal.ac5,bmpcal.ac6);
}



/******************************************************************************
//函数功能:BMP085 数据更新**************
/ 修改日期:none
/ 输入参数:none
/ 输出参数:u32
/ 使用说明:none
******************************************************************************/
void Baro_update()
{
	static u8	cycletimes=0,change=1;
	if(change)
	{
		change = 0;
		bmpcal.ut = BMP085_ReadTemp();
		BMP085_StartPress();
	}
	else
	{
		change = 1;
		bmpcal.up = BMP085_ReadPress();
		BMP085_StartTemp();
		BMP085_Calculate();
	}
}

/******************************************************************************
/ 函数功能:bmp085数据计算
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void BMP085_Calculate()
{
	static u8	cycletimes=0;
	s32 x1, x2, b5, b6, x3, b3, p;
	u32 b4, b7;
	
//	if(!cycletimes) bmpcal.ut = BMP085_CallTemp();	   		// 读取温度
//	cycletimes++;									//周期计数
//	if(cycletimes==50) cycletimes = 0;				//每50次进行一次温度读取
//		

	x1 = (((s32)bmpcal.ut - (u32)bmpcal.ac6)*bmpcal.ac5) >> 15;
	x2 = ( (s32)bmpcal.mc << 11) / (x1 + bmpcal.md);
	b5 = x1 + x2;
	bmpcal.temperature = ((b5 + 8) >> 4);				//单位0.1度
											//baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)
	//*************

	b6 = b5 - 4000;
	// Calculate B3
	x1 = (bmpcal.b2 * (b6 * b6 >> 12)) >> 11; 
	x2 = (bmpcal.ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((s32)bmpcal.ac1)*4 + x3)<<OSS) + 2)>>2;

	// Calculate B4
	x1 = (bmpcal.ac3 * b6)>>13;
	x2 = (bmpcal.b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (bmpcal.ac4 * (u32)(x3 + 32768))>>15;

	//  b7 = ((u32)(up - b3) * (50000>>OSS));		// b7 = ((uint32_t) (bmp085_ctx.up.val >> (8-OSS)) - b3) * (50000 >> OSS);
	b7 = ((u32) (bmpcal.up >> (8-OSS)) - b3) * (50000 >> OSS);
	if (b7 < 0x80000000)
		p = (b7<<1)/b4;
	else
		p = (b7/b4)<<1;

	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	bmpcal.pressure = p+((x1 + x2 + 3791)>>4);		//单位Pa
	bmpcal.pressure -= 30000;

}
