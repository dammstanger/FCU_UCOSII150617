/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：ComProtocol.c
 * 描	述	：外部中断初始化和中断服务
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.1506.3
 * 从属关系	：FCU_UCOSII150603_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.4.11
 * 最后编辑	：2015.6.6
 **-------------------------------------------------------------------------------

**********************************************************************************************/
#ifndef __COMPROTOCOL_H__
#define __COMPROTOCOL_H__


#include "Project_cfg.h"
#include "stm32f10x.h"
#include "24l01.h"


typedef struct{
uint16_t CONFIG_REG;							/* CONFIG: config quadcopter work mode */
uint16_t STATE_BSP_REG;							/* STATE_BSP: the hardware on board work state */
uint16_t STATE_SYS_REG;							/* STATE_SYS: system work state */
uint16_t STATE_PWR_REG;							/* STATE_PWR: battery level */
uint16_t MOTOR1_PWM_REG;
uint16_t MOTOR2_PWM_REG;
uint16_t MOTOR3_PWM_REG;
uint16_t MOTOR4_PWM_REG;
uint16_t IMU_AX_REG;
uint16_t IMU_AY_REG;
uint16_t IMU_AZ_REG;
uint16_t IMU_GX_REG;
uint16_t IMU_GY_REG;
uint16_t IMU_GZ_REG;
uint16_t FLY_PITCH_REG;							/* 0xf000 is zero,  greater than 0xf000 stand for positive; less than 0xf000 stand for negative */
uint16_t FLY_YAW_REG;
uint16_t FLY_ROLL_REG;
uint16_t FLY_THRO_REG;
uint16_t PIDPar_CTLTYPE_REG;
uint16_t PIDPar_P_REG;
uint16_t PIDPar_I_REG;
uint16_t PIDPar_D_REG;	
uint16_t PIDPar_IMAX_REG;
} __fly_reg;
extern __fly_reg FlyReg;


//#define FLY_ADDR				0x1234				/* The quadcopter's address in RC */

/* CONFIG_REG: config/control quadcopter work mode */
#define MOTOR_EN				0x0008				/* motor out enable */
#define CONFIG_LVD				0x0007				/* LVD[2:0] bits (Low Voltage Detector) */
#define CONFIG_LVD_0			0x0001				/* Bits 0 */
#define CONFIG_LVD_1			0x0002				/* Bits 1 */
#define CONFIG_LVD_2			0x0004				/* Bits 2 */

/* STATE_BSP_REG: the hardware on board work state */
#define STATE_BSP_ALL_OK		0x8000				/* BSP all OK bit */
#define STATE_BSP_NRF			0x0001				/* NRF OK bit */
#define STATE_BSP_MPU			0x0002				/* MPU OK bit */

/* STATE_SYS_REG: system work state */
#define STATE_SYS_FLY_STATE		0x0002				/* quadcopter state, flying or landing */
#define STATE_SYS_RC_CONNECT	0x0001				/* 0: Communication disconnected, not received message within 200 ms	*/
													/* 1: Communication connecting, received messages within 200 ms */



//协议类型	2字节
#define RC_TYPE_H_P				0x00
#define RC_TYPE_L_P				0x01
//协议类型	飞行控制协议
#define RC_TYPE_FLYASK_V		0x8000
#define RC_TYPE_FLYASK_H_V		0x80
#define RC_TYPE_FLYASK_L_V		0x00
//协议类型	飞行控制协议应答
#define RC_TYPE_FLYACK_V		0x8100
#define RC_TYPE_FLYACK_H_V		0x81
#define RC_TYPE_FLYACK_L_V		0x00
//协议类型	飞行姿态查询协议
#define RC_TYPE_IMUASK_V		0xa000
#define RC_TYPE_IMUASK_H_V		0xa0
#define RC_TYPE_IMUASK_L_V		0x00
//协议类型	飞行姿态查询协议应答
#define RC_TYPE_IMUACK_V		0xa100
#define RC_TYPE_IMUACK_H_V		0xa1
#define RC_TYPE_IMUACK_L_V		0x00
//协议类型	电机控制协议
#define RC_TYPE_MOTOASK_V		0xc000
#define RC_TYPE_MOTOASK_H_V		0xc0
#define RC_TYPE_MOTOASK_L_V		0x00
//协议类型	电机控制协议应答
#define RC_TYPE_MOTOACK_V		0xc100
#define RC_TYPE_MOTOACK_H_V		0xc1
#define RC_TYPE_MOTOACK_L_V		0x00
//协议类型	PID参数调整协议
#define RC_TYPE_PIDPAR_V		0xd000
#define RC_TYPE_PIDPAR_H_V		0xd0
#define RC_TYPE_PIDPAR_L_V		0x00
//协议类型	PID参数调整协议应答
#define RC_TYPE_PIDPARACK_V		0xd100
#define RC_TYPE_PIDPARACK_H_V	0xd1
#define RC_TYPE_PIDPARACK_L_V	0x00
//协议类型	寄存器读写协议
#define RC_TYPE_REGWRASK_V		0xe000
#define RC_TYPE_REGWRASK_H_V	0xe0
#define RC_TYPE_REGWRASK_L_V	0x00
//协议类型	寄存器读写协议应答
#define RC_TYPE_REGWRACK_V		0xe100
#define RC_TYPE_REGWRACK_H_V	0xe1
#define RC_TYPE_REGWRACK_L_V	0x00

extern __fly_reg *Reg;


#define PkgSend()				\
{								\
	NRF24L01_Tx();					\
	NRF24L01_TxPacket(BUF_T, 10);	\
	NRF24L01_Rx();					\
}


//飞行控制协议
#define RC_FLYASK_PITCH_L_P		0x02
#define RC_FLYASK_PITCH_H_P		0x03
#define RC_FLYASK_YAW_L_P		0x04
#define RC_FLYASK_YAW_H_P		0x05
#define RC_FLYASK_ROLL_L_P		0x06
#define RC_FLYASK_ROLL_H_P		0x07
#define RC_FLYASK_THRO_L_P		0x08
#define RC_FLYASK_THRO_H_P		0x09

#define IsFlyAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_FLYASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_FLYASK_L_V))

#define FlyAskPkgHandle()		\
{								\
	Reg->FLY_PITCH_REG = ((u16)(*(BUF_R+RC_FLYASK_PITCH_H_P))<<8)|(*(BUF_R+RC_FLYASK_PITCH_L_P));	\
	Reg->FLY_YAW_REG = ((u16)(*(BUF_R+RC_FLYASK_YAW_H_P))<<8)|(*(BUF_R+RC_FLYASK_YAW_L_P));			\
	Reg->FLY_ROLL_REG = ((u16)(*(BUF_R+RC_FLYASK_ROLL_H_P))<<8)|(*(BUF_R+RC_FLYASK_ROLL_L_P));		\
	Reg->FLY_THRO_REG = ((u16)(*(BUF_R+RC_FLYASK_THRO_H_P))<<8)|(*(BUF_R+RC_FLYASK_THRO_L_P));		\
}

#define FlyAckPkgSend()


//飞行器姿态查询协议
#define RC_IMUASK_AX_L_P		0x02
#define RC_IMUASK_AX_H_P		0x03
#define RC_IMUASK_AY_L_P		0x04
#define RC_IMUASK_AY_H_P		0x05
#define RC_IMUASK_AZ_L_P		0x06
#define RC_IMUASK_AZ_H_P		0x07
#define RC_IMUASK_GX_L_P		0x08
#define RC_IMUASK_GX_H_P		0x09
#define RC_IMUASK_GY_L_P		0x0a
#define RC_IMUASK_GY_H_P		0x0b
#define RC_IMUASK_GZ_L_P		0x0c
#define RC_IMUASK_GZ_H_P		0x0d

#define IsImuAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_IMUASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_IMUASK_L_V))

#define ImuAskPkgHandle()		\
{								\
}

#define ImuAckPkgSend()			\
{								\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_IMUACK_H_V;			\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_IMUACK_L_V;			\
	*(BUF_T+RC_IMUASK_AX_H_P) = Reg->IMU_AX_REG>>8;		\
	*(BUF_T+RC_IMUASK_AX_L_P) = Reg->IMU_AX_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_AY_H_P) = Reg->IMU_AY_REG>>8;		\
	*(BUF_T+RC_IMUASK_AY_L_P) = Reg->IMU_AY_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_AZ_H_P) = Reg->IMU_AZ_REG>>8;		\
	*(BUF_T+RC_IMUASK_AZ_L_P) = Reg->IMU_AZ_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_GX_H_P) = Reg->IMU_GX_REG>>8;		\
	*(BUF_T+RC_IMUASK_GX_L_P) = Reg->IMU_GX_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_GY_H_P) = Reg->IMU_GY_REG>>8;		\
	*(BUF_T+RC_IMUASK_GY_L_P) = Reg->IMU_GY_REG&0x00ff;	\
	*(BUF_T+RC_IMUASK_GZ_H_P) = Reg->IMU_GZ_REG>>8;		\
	*(BUF_T+RC_IMUASK_GZ_L_P) = Reg->IMU_GZ_REG&0x00ff;	\
	PkgSend();											\
}


//电机控制协议
#define RC_MOTOASK_MOTO1_L_P	0x02
#define RC_MOTOASK_MOTO1_H_P	0x03
#define RC_MOTOASK_MOTO2_L_P	0x04
#define RC_MOTOASK_MOTO2_H_P	0x05
#define RC_MOTOASK_MOTO3_L_P	0x06
#define RC_MOTOASK_MOTO3_H_P	0x07
#define RC_MOTOASK_MOTO4_L_P	0x08
#define RC_MOTOASK_MOTO4_H_P	0x09

#define IsMotoAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_MOTOASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_MOTOASK_L_V))

#define MotoAskPkgHandle()		\
{								\
	Reg->MOTOR1_PWM_REG = ((u16)(*(BUF_R+RC_MOTOASK_MOTO1_H_P))<<8)|(*(BUF_R+RC_MOTOASK_MOTO1_L_P));	\
	Reg->MOTOR2_PWM_REG = ((u16)(*(BUF_R+RC_MOTOASK_MOTO2_H_P))<<8)|(*(BUF_R+RC_MOTOASK_MOTO2_L_P));	\
	Reg->MOTOR3_PWM_REG = ((u16)(*(BUF_R+RC_MOTOASK_MOTO3_H_P))<<8)|(*(BUF_R+RC_MOTOASK_MOTO3_L_P));	\
	Reg->MOTOR4_PWM_REG = ((u16)(*(BUF_R+RC_MOTOASK_MOTO4_H_P))<<8)|(*(BUF_R+RC_MOTOASK_MOTO4_L_P));	\
}

#define MotoAckPkgSend()		\
{								\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_MOTOACK_H_V;			\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_MOTOACK_L_V;			\
	PkgSend();											\
}


//PID控制器参数修改协议
#define RC_PID_CTLTYPE_H_P		0x02
#define RC_PID_CTLTYPE_L_P		0x03
#define RC_PIDPAR_P_H_P			0x04
#define RC_PIDPAR_P_L_P			0x05
#define RC_PIDPAR_I_H_P			0x06
#define RC_PIDPAR_I_L_P			0x07
#define RC_PIDPAR_D_H_P			0x08
#define RC_PIDPAR_D_L_P			0x09
#define RC_PIDPAR_IMAX_H_P		0x0a
#define RC_PIDPAR_IMAX_L_P		0x0b

#define PID_CTLTYPE_RPSTB		1			//roll pitch stabilitze controller
#define PID_CTLTYPE_RPA			2			//roll pitch angle controller
#define PID_CTLTYPE_RPARATE		3			//roll pitch angle rate controller
#define PID_CTLTYPE_YAW			4
#define PID_CTLTYPE_ALT			5
#define PID_CTLTYPE_POS			6
#define PID_CTLTYPE_ALTRATE		7
#define PID_CTLTYPE_ALTACC		8


#define IsPIDParAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_PIDPAR_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_PIDPAR_L_V))

#define PIDParAskPkgHandle()		\
{								\
	Reg->PIDPar_CTLTYPE_REG = 	((u16)(*(BUF_R+RC_PID_CTLTYPE_H_P))<<8)|(*(BUF_R+RC_PID_CTLTYPE_L_P));	\
	Reg->PIDPar_P_REG = 		((u16)(*(BUF_R+RC_PIDPAR_P_H_P))<<8)|(*(BUF_R+RC_PIDPAR_P_L_P));	\
	Reg->PIDPar_I_REG = 		((u16)(*(BUF_R+RC_PIDPAR_I_H_P))<<8)|(*(BUF_R+RC_PIDPAR_I_L_P));	\
	Reg->PIDPar_D_REG = 		((u16)(*(BUF_R+RC_PIDPAR_D_H_P))<<8)|(*(BUF_R+RC_PIDPAR_D_L_P));	\
	Reg->PIDPar_IMAX_REG = 		((u16)(*(BUF_R+RC_PIDPAR_IMAX_H_P))<<8)|(*(BUF_R+RC_PIDPAR_IMAX_L_P));	\
}

#define PIDParAckPkgSend()		\
{								\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_PIDPARACK_H_V;		\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_PIDPARACK_L_V;		\
	PkgSend();											\
}


//寄存器读写协议
#define RC_REGWRASK_WRITE_V		0x15
#define RC_REGWRASK_READ_V		0x1a
#define RC_REGWRASK_WR_P		0x02
#define RC_REGWRASK_REG_P		0x03
#define RC_REGWRASK_VAL_L_P		0x04
#define RC_REGWRASK_VAL_H_P		0x05

#define IsRegWrAskPkg()			((*(BUF_R+RC_TYPE_H_P)==RC_TYPE_REGWRASK_H_V) && (*(BUF_R+RC_TYPE_L_P)==RC_TYPE_REGWRASK_L_V))

#define RegWrAskPkgIsWrite()	(*(BUF_R+RC_REGWRASK_WR_P)==RC_REGWRASK_WRITE_V)

#define RegWrAskPkgIsRead()		(*(BUF_R+RC_REGWRASK_WR_P)==RC_REGWRASK_READ_V)

#define RegWritePkgHandle()		\
{								\
	*((uint16_t*)(Reg)+(*(BUF_R+RC_REGWRASK_REG_P))) = ((u16)(*(BUF_R+RC_REGWRASK_VAL_H_P))<<8)|(*(BUF_R+RC_REGWRASK_VAL_L_P));	\
}

#define RegReadPkgHandle()		\
{								\
}

#define RegWrAckPkgSend()		\
{								\
	*(BUF_T+RC_TYPE_H_P) = RC_TYPE_REGWRACK_H_V;		\
	*(BUF_T+RC_TYPE_L_P) = RC_TYPE_REGWRACK_L_V;		\
	*(BUF_T+RC_REGWRASK_REG_P) = 	*(BUF_R+RC_REGWRASK_REG_P);									\
	*(BUF_T+RC_REGWRASK_VAL_H_P) = 	(*((uint16_t*)(Reg)+(*(BUF_R+RC_REGWRASK_REG_P))))>>8;		\
	*(BUF_T+RC_REGWRASK_VAL_L_P) = 	(*((uint16_t*)(Reg)+(*(BUF_R+RC_REGWRASK_REG_P))))&0x00ff;	\
	PkgSend();											\
}

extern uint8_t RevFinish;
extern unsigned char RC_Handle(void);
void SendDebugDat_Hunter_2401(int dat1,int dat2,int dat3,int dat4);//发送匹配串口猎人的数据
void SendDebugDat_LabVIEW_2401(u8 datype,int dat1,int dat2,int dat3,int dat4);//发送匹配labviwe的数据

u8 PIDDebugData(__fly_reg *debdat);
bool _24L01PIDParData(void);


#endif
