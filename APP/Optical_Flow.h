/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Optical_Flow.h
 * 描	述	：处理光流数据
 *                   
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150416
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2015.1.2
 * 最后编辑	：2015.4.16
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/

#ifndef __OPTICAL_FLOW_H__
#define __OPTICAL_FLOW_H__

/****************************包含头文件*******************************************/
#include "Project_cfg.h"
#include "stm32f10x.h"

/****************************宏定义***********************************************/

#define OPTICALFLOW_ANGLE_LIMIT_MAX		45
//#define OPTICALFLOW_ANGLE_LIMIT_MIN		45


/****************************结构体定义*******************************************/
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
	float	f_ei;						//变积分系数
	float	irange_min;					//变积分的区间的下限
	float	irange;						//变积分的区间的大小
	float	irange_max;					//变积分的区间实际上限

}OPTICALFLOW_PAR;
/****************************变量声明*********************************************/
extern AP_TRANS_PAK AP_Pakbuf;
extern OPTICALFLOW_PAR Optflow;
extern int16_t dx_ref,dy_ref;
extern float exp_change_x, exp_change_y;
extern u8 g_intercnt;

/****************************函数声明*********************************************/

void OPFLOW_Para_Init(void);
void Update_Position(float roll, float pitch,float sin_yaw, float cos_yaw, s16 altitude);
#if OPFLOW_SENSOR_OFFBOARD_USART_EN
void OPFLOW_DataUpdate(void);
#endif

#endif	//

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

















