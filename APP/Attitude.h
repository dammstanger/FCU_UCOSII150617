/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Attitude.h
 * ��	��	����̬����
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.28
 * ���༭	��2015.6.15
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#ifndef __ATTITUDE_H_
#define __ATTITUDE_H_

/****************************����ͷ�ļ�*******************************************/
#include "stm32f10x.h"
#include "RCdata.h"
#include "AHRS.h"
/****************************�궨��***********************************************/


/****************************�ṹ�嶨��*******************************************/
typedef struct 
{
	int32_t x;
	int32_t y;
	int32_t z;
}AXIS_DATA;

typedef struct 
{
//	int32_t roll;
//	int32_t	pitch;
//	int32_t yaw;
	float	roll;
	float	pitch;
	float	yaw;
}EULER_DATA_TYPE;

/****************************��������*********************************************/
extern float yaw_print;
extern float Headhold;
extern float Headoriginal_rad;
extern float Headfreehold;
extern EULER_DATA_TYPE RC_att;
extern EULER_DATA_TYPE Optflow_att;
extern EULER_DATA_TYPE Control_proc_att;
extern EULER_DATA_TYPE Control_ulti_att;	//ultimatez���յĿ��ƽǶ�
extern float dx_cm_accuml,dy_cm_accuml;   
extern float desired_rate;
extern float vertical_rate_error;			//��λmm
extern float vtarget_filter_rate;
extern int32_t target_output;
extern u16 HightHoldVal;					//�߶ȱ��ָ���ֵ
//extern ZHOUDATA Vehiclebias;
extern s16 Controller_target_alt;
extern float vertical_acc_mm;
extern float vacc_error;

/****************************��������*********************************************/
float Add_Optflow_Roll(u8 execute);
float Add_Optflow_Pitch(u8 execute);
float Add_RC_Roll(u16 rc_roll);
float Add_RC_Pitch(u16 rc_pitch);
float Comput_Ctr_Yaw(float input_yaw,RC_DATA RCcomand);
void RecordAltHoldPoint(bool enable);
int16_t Alt_Sonar_Controller(int32_t target_rate);
int16_t Alt_AutoSmooth_Handle(u16 target_alt);
int16_t AltAcc_Controller(int32_t vtarget_accel);
void Set_Alt_Holdtarget(u16 alt_target);


#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

