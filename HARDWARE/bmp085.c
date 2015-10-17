
//20141101
//***************************************
// BMP085 IIC���Գ���
// �ο��꾧��վ24c04ͨ�ų���
// ʱ�䣺2014��11��01��
// DammStanger
// ע�⣺���IIC����һ��ʹ��
//****************************************
#include "bmp085.h"
#include "I2C_1.h"

#include  <math.h>    //Keil library  
#include  <stdlib.h>  //Keil library  
#include  <stdio.h>   //Keil library	
  	
BMP085_PAR  bmpcal;


/******************************************************************************
/ ��������:����BMP085�ڲ�����,����
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/

void BMP085_MultRead( u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{   
	IIC1_MultRead(BMP085_SlaveAddress,pBuffer,readAddr,NumByteToRead);
}

/******************************************************************************
/ ��������:��bmp085�豸д��һ���ֽ�����
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void BMP085_WriteByte(u8 REG_Address,u8 REG_data)
{
	
	IIC1_WriteByte(BMP085_SlaveAddress,REG_Address,REG_data);
}


/******************************************************************************
/ ��������:��bmp085�豸�����¶�ת��
/ �޸�����:none
/ �������:none
/ �������:void
/ ʹ��˵��:none
******************************************************************************/
void BMP085_StartTemp(void)
{	
	BMP085_WriteByte(BMP085_CONTRLREG,BMP085_STRTEMP);
}


/******************************************************************************
/ ��������:��bmp085�豸��ȡ�¶�
/ �޸�����:none
/ �������:none
/ �������:u32
/ ʹ��˵��:none
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
/ ��������:��bmp085����ѹ��ת��
/ �޸�����:none
/ �������:none
/ �������:void
/ ʹ��˵��:none
******************************************************************************/
void BMP085_StartPress(void)
{	
	BMP085_WriteByte(BMP085_CONTRLREG,BMP085_PRESSMODE);
}


/******************************************************************************
/ ��������:��bmp085�豸��ȡ�¶�
/ �޸�����:none
/ �������:none
/ �������:u32
/ ʹ��˵��:none
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
//��������:��ʼ��BMP085��������Ҫ��ο�pdf�����޸�**************
/ �޸�����:none
/ �������:none
/ �������:u32
/ ʹ��˵��:none
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
//��������:BMP085 ���ݸ���**************
/ �޸�����:none
/ �������:none
/ �������:u32
/ ʹ��˵��:none
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
/ ��������:bmp085���ݼ���
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void BMP085_Calculate()
{
	static u8	cycletimes=0;
	s32 x1, x2, b5, b6, x3, b3, p;
	u32 b4, b7;
	
//	if(!cycletimes) bmpcal.ut = BMP085_CallTemp();	   		// ��ȡ�¶�
//	cycletimes++;									//���ڼ���
//	if(cycletimes==50) cycletimes = 0;				//ÿ50�ν���һ���¶ȶ�ȡ
//		

	x1 = (((s32)bmpcal.ut - (u32)bmpcal.ac6)*bmpcal.ac5) >> 15;
	x2 = ( (s32)bmpcal.mc << 11) / (x1 + bmpcal.md);
	b5 = x1 + x2;
	bmpcal.temperature = ((b5 + 8) >> 4);				//��λ0.1��
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
	bmpcal.pressure = p+((x1 + x2 + 3791)>>4);		//��λPa
	bmpcal.pressure -= 30000;

}
