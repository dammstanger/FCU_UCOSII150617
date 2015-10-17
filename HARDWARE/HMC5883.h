
/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名		：HMC5883.h
 * 描	述		：STM32 HMC5883L 驱动
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150409
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本		：ST3.0.0
 * 创建时间	：2014.8.31
 * 最后编辑	：2015.4.9
 * 备			注：
 **-------------------------------------------------------------------------------

 * 作	者		：Damm Stanger
 * 邮	箱		：dammstanger@qq.com
**********************************************************************************************/


#ifndef _HMC5883_H_
#define	_HMC5883_H_

#include "Project_cfg.h"
#include "stm32f10x.h"
//#include <stdio.h>



/******************************************************************************
***************************** HMC5883L 宏定义 *********************************
******************************************************************************/
#define	HMC5883_SlaveAddress   0x3C	  	//定义器件在IIC总线中的从地址,这里的地址0x3c是8位写地址，不用
										//再左移一位了。7位地址0X1E


/*---------------------* 
*  HMC5883L内部寄存器  * 
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
#define HMC5883L_IRA    0x0a    //读序列号使用的寄存器
#define HMC5883L_IRB    0x0b
#define HMC5883L_IRC    0x0c 

#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

/*---------------------* 
*   HMC5883 校正参数   * 
*----------------------*/
#define HMC58X3L_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3L_Y_SELF_TEST_GAUSS (+1.16)                       //!< Y axis level when bias current is applied.
#define HMC58X3L_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.


/******************************************************************************
********************************* 融合数据   **********************************
******************************************************************************/
/*---------------------* 
*   融合 数据类型      * 
*----------------------*/
typedef struct          //欧拉角表示方法
{
    int16_t  yaw;       //
    int16_t  pitch;
    int16_t  roll;
}tg_AHRS_TYPE;


/*---------------------* 
*   HMC5883 数据类型   * 
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
extern tg_HMC5883L_TYPE HMCoffset;						//存放偏移
extern float HMC5883L_GAIN_X ;
extern float HMC5883L_GAIN_Y ;
extern float HMC5883L_GAIN_Z ;

u8 HMC5883L_Init(void);

//读取传感器数据
void HMC5883L_Read(tg_HMC5883L_TYPE * ptResult);     //普通完整测量(有至少20ms时序延时)
void HMC5883L_Start(void);                           //启动-中断-读取 (启动)有10ms延时
void HMC5883L_MultRead(tg_HMC5883L_TYPE * ptResult); //启动-中断-读取 (读取)
void HMC5883L_ReadBytes(u8* pBuffer,u8 readAddr, u16 NumByteToRead);

//校准传感器
void HMC5883L_GainCalibrate(void);
void HMC5883L_BiasCalibrate(void);
//双轴简单计算
void HMC5883L_Cal(tg_HMC5883L_FLOAT * ptResult);


#endif


