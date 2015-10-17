/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Optical_Flow.h
 * ��	��	�������������
 *                   
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150416
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2015.1.2
 * ���༭	��2015.4.16
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#ifndef __OPTICAL_FLOW_H__
#define __OPTICAL_FLOW_H__

/****************************����ͷ�ļ�*******************************************/
#include "Project_cfg.h"
#include "stm32f10x.h"

/****************************�궨��***********************************************/

#define OPTICALFLOW_ANGLE_LIMIT_MAX		45
//#define OPTICALFLOW_ANGLE_LIMIT_MIN		45


/****************************�ṹ�嶨��*******************************************/
typedef struct 
{
	s8	DX_REG;
	s8	DY_REG;
	u8	QUAL_REG;
	u8	BRIGHT_H;
	u8	BRIGHT_L;
	s16	ADD_DX;
	s16 ADD_DY;
}AP_TRANS_PAK;
typedef struct 
{
	uint8_t num_pixels;
	float	scaler;
	float	field_of_view;
	float	conv_factor;
	float	radians_to_pixels;

	float	change_x, change_y;            // actual change in x, y coordinates
	float	dx_cm, dy_cm;                    // x,y position in cm
	float	x_add_cm;
	float	y_add_cm;
	
	float	Kp;
	float	Kd;
	float	Ki;
	float	f_ei;						//�����ϵ��
	float	irange_min;					//����ֵ����������
	float	irange;						//����ֵ�����Ĵ�С
	float	irange_max;					//����ֵ�����ʵ������

}OPTICALFLOW_PAR;
/****************************��������*********************************************/
extern AP_TRANS_PAK AP_Pakbuf;
extern OPTICALFLOW_PAR Optflow;
extern int16_t dx_ref,dy_ref;
extern float exp_change_x, exp_change_y;
extern u8 g_intercnt;

/****************************��������*********************************************/

void OPFLOW_Para_Init(void);
void Update_Position(float roll, float pitch,float sin_yaw, float cos_yaw, s16 altitude);
#if OPFLOW_SENSOR_OFFBOARD_USART_EN
void OPFLOW_DataUpdate(void);
#endif

#endif	//

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

















