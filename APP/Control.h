/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Controll.h
 * ��	��	����̬������
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.1.150613
 * ������ϵ	��FCU_UCOSII150603_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.28
 * ���༭	��2015.6.13
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/
/****************************����ͷ�ļ�*******************************************/
#ifndef __CONTROL_H_
#define	__CONTROL_H_

#include "Project_cfg.h"
#include "stm32f10x.h"
#include "Timer_PWMin.h"
#include "Attitude.h"

#define MOTO_PWM_MAX 2000		//������������ֵ
#define MOTO_PWM_MIN 1120		//�����������Сֵ
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
