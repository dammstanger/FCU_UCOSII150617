#ifndef _BMP085_H_
#define _BMP085_H_

#include "Project_cfg.h"
#include "stm32f10x.h"


#define	BMP085_SlaveAddress   	0xEE	  	//����������IIC�����еĴӵ�ַ  ����ַΪ0xEF                             
#define BMP085_CALIREG_AC1		0xAA		//У��ϵ���Ĵ���
#define BMP085_CALIREG_B1		0xB6
#define BMP085_CALIREG_MB		0xBA

#define BMP085_CONTRLREG		0xF4		//���ƼĴ���
#define BMP085_VALREGMSB		0xF6		//����Ĵ�����λ
#define BMP085_VALREGLSB		0xF7		//����Ĵ�����λ
#define BMP085_VALREGXLSB		0xF8		//����Ĵ����ص�λ

#define OSS 2	// Oversampling Setting at high resolution mode

#define BMP085_PRESSMODE				0x34+(OSS<<6)		//ģʽ��������Ҫ����(oversampleing_setting<<6)�ŵ���ģʽ,�������ȼ����ڡ�+��
#define BMP085_STRTEMP			0x2E		//�����¶�ת������


typedef struct 
{
	s16 ac1;
	s16 ac2; 
	s16 ac3; 
	u16 ac4;
	u16 ac5;
	u16 ac6;
	s16 b1; 
	s16 b2;
	s16 mb;
	s16 mc;
	s16 md;
	u32	ut;
	u32 up;
	u32 temperature;
	u32 pressure;
}BMP085_PAR;

extern BMP085_PAR  bmpcal;

void BMP085_Calculate(void);
void BMP085_MultRead(u8* pBuffer, u8 readAddr, u16 NumByteToRead);
//------------------------------------
void BMP085_StartPress(void);
void BMP085_StartTemp(void);
u32 BMP085_ReadPress(void);
u32 BMP085_ReadTemp(void);
void Baro_update(void);

void BMP085_Init(void);


#endif

