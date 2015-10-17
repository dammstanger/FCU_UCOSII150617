/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Controll.h
 * 描	述	：姿态控制器
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.1.150613
 * 从属关系	：FCU_UCOSII150603_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.28
 * 最后编辑	：2015.6.13
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/
/****************************包含头文件*******************************************/
#ifndef __CONTROL_H_
#define	__CONTROL_H_

#include "Project_cfg.h"
#include "stm32f10x.h"
#include "Timer_PWMin.h"
#include "Attitude.h"

#define MOTO_PWM_MAX 2000		//限制输出的最大值
#define MOTO_PWM_MIN 1120		//限制输出的最小值
//#define CALIBRATE_PWMOUT		

typedef struct 
{
	float P;
	float I;
	float D;
	float Pout;
	float Iout;
	float Dout;
	float Imax;
	float ALLout;
}PID_PAR;

typedef struct 
{
	float P;
	float I;
	float Pout;
	float Iout;
	float Imax;
	float ALLout;
}PI_PAR;

typedef struct 
{
	float P;
	float D;
	float Pout;
	float Dout;
	float ALLout;
}PD_PAR;

typedef struct 
{
	float P;
	float Pout;
}P_PAR;

extern PID_PAR PID_ACC_ALT;
extern P_PAR P_SONAR_ALT;
extern P_PAR P_ROOT_ALT;
extern P_PAR P_RATE_ALT;
extern P_PAR P_RATEFILT_ALT;
extern PID_PAR PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_HIGH,PID_POS,PID_STABLE,PID_POS_OF;
extern s16 Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4,Moto_PWM_5,Moto_PWM_6,Moto_PWM_7,Moto_PWM_8;
//extern u16 throttlelast;
extern float Temp_thro;
extern s32 Hight_i;
extern FunctionalState g_Motor_output_enable;
void PID_Para_Init(void);
void CONTROL(EULER_DATA_TYPE ctr_angle,int32_t alt_target_acc,u16 throttlelast,bool enable);
float AltOld_Controller(void);
bool SafeDeal(float rol, float pit);	

#endif
