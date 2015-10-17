
/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���		��HMC5883.h
 * ��	��		��STM32 HMC5883L ����
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150409
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾		��ST3.0.0
 * ����ʱ��	��2014.8.31
 * ���༭	��2015.4.9
 * ��			ע��
 **-------------------------------------------------------------------------------

 * ��	��		��Damm Stanger
 * ��	��		��dammstanger@qq.com
**********************************************************************************************/


#ifndef _HMC5883_H_
#define	_HMC5883_H_

#include "Project_cfg.h"
#include "stm32f10x.h"
//#include <stdio.h>



/******************************************************************************
***************************** HMC5883L �궨�� *********************************
******************************************************************************/
#define	HMC5883_SlaveAddress   0x3C	  	//����������IIC�����еĴӵ�ַ,����ĵ�ַ0x3c��8λд��ַ������
										//������һλ�ˡ�7λ��ַ0X1E


/*---------------------* 
*  HMC5883L�ڲ��Ĵ���  * 
*----------------------*/
#define HMC5883L_REGA   0x00
#define HMC5883L_REGB   0x01
#define HMC5883L_MODE   0x02
#define HMC5883L_HX_H   0x03
#define HMC5883L_HX_L   0x04 
#define HMC5883L_HZ_H   0x05
#define HMC5883L_HZ_L   0x06
#define HMC5883L_HY_H   0x07
#define HMC5883L_HY_L   0x08
#define HMC5883L_STATE  0x09
#define HMC5883L_IRA    0x0a    //�����к�ʹ�õļĴ���
#define HMC5883L_IRB    0x0b
#define HMC5883L_IRC    0x0c 

#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

/*---------------------* 
*   HMC5883 У������   * 
*----------------------*/
#define HMC58X3L_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3L_Y_SELF_TEST_GAUSS (+1.16)                       //!< Y axis level when bias current is applied.
#define HMC58X3L_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.


/******************************************************************************
********************************* �ں�����   **********************************
******************************************************************************/
/*---------------------* 
*   �ں� ��������      * 
*----------------------*/
typedef struct          //ŷ���Ǳ�ʾ����
{
    int16_t  yaw;       //
    int16_t  pitch;
    int16_t  roll;
}tg_AHRS_TYPE;


/*---------------------* 
*   HMC5883 ��������   * 
*----------------------*/
typedef struct
{
    int16_t  hx;
    int16_t  hy;
    int16_t  hz;
    uint16_t m_yaw;   
}tg_HMC5883L_TYPE;

typedef struct
{
    float  hx;
    float  hy;
    float  hz;
    float  m_yaw;   
}tg_HMC5883L_FLOAT;


extern tg_HMC5883L_TYPE hmc5883l;
extern tg_HMC5883L_FLOAT Magdecp;
extern tg_HMC5883L_TYPE HMCoffset;						//���ƫ��
extern float HMC5883L_GAIN_X ;
extern float HMC5883L_GAIN_Y ;
extern float HMC5883L_GAIN_Z ;

u8 HMC5883L_Init(void);

//��ȡ����������
void HMC5883L_Read(tg_HMC5883L_TYPE * ptResult);     //��ͨ��������(������20msʱ����ʱ)
void HMC5883L_Start(void);                           //����-�ж�-��ȡ (����)��10ms��ʱ
void HMC5883L_MultRead(tg_HMC5883L_TYPE * ptResult); //����-�ж�-��ȡ (��ȡ)
void HMC5883L_ReadBytes(u8* pBuffer,u8 readAddr, u16 NumByteToRead);

//У׼������
void HMC5883L_GainCalibrate(void);
void HMC5883L_BiasCalibrate(void);
//˫��򵥼���
void HMC5883L_Cal(tg_HMC5883L_FLOAT * ptResult);


#endif


