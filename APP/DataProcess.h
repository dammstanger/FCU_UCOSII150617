/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：DataProcess.h
 * 描	述	：生成飞行模式，并依据飞行模式对各类数据进行处理，产生不同的控制目标
 *                     
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150616
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.4.1
 * 最后编辑	：2015.6.16
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/

#ifndef __DATAPROCESS_H_
#define	__DATAPROCESS_H_
/****************************包含头文件*******************************************/
#include "Project_cfg.h"
#include "stm32f10x.h"
#include "AHRS.h"
#include "RCdata.h"
/****************************宏定义***********************************************/

//-------------飞行模式-----------------
#define UNARMED			0
#define	ARMED			1
#define ATTITUDE		2
#define	HEADFREE		3
#define ATLHOLD			4
#define POSHOLD			5
#define AUTO			6

//-----飞行任务一般在自主模式下使用-----------

#define TASK_TAKEOFF						1
#define TASK_POSHOLD						2
#define TASK_LAND							3

//---------------油门模式----------------------
#define THROTTLE_MANUAL						0
#define THROTTLE_HOLD                       1   // 
#define THROTTLE_AUTO                       2   // auto pilot altitude controller with target altitude held in cmd
#define THROTTLE_LAND                       3   // landing throttle controller

#define THRO_K				0.85				//线性油门斜率

#define LAND_DETECTOR_CNT			15					//1.5S 确认降落完成

/****************************结构体定义*******************************************/

/****************************变量声明*********************************************/
extern ZHOUDATA MOVEVEL;					//三轴的移动加速度	
extern ZHOUDATA MOVEACC;

extern float sinx,siny,sin2x,sin2y,abs_Roll_Raw,abs_Pitch_Raw;
extern float vx,vy,vz;
extern u8 VehicleBalComp;
extern u8	FlightMode;						//飞行模式
extern u16	Control_proc_thro;				//油门量
extern int32_t Control_targer_altacc;		//高度ACC控制器目标加速度
extern bool g_altacc_controller_active;
extern bool g_land_finish;
/****************************函数声明*********************************************/
void HeadfreeMode(RC_DATA *rccomand);
void AttDataProcess(void);
void FlyModeProcess(void);
void ThrottleModeProcess(void);
 void Update_Land_Detector(void);
#endif

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/
