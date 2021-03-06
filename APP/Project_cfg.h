/********************************Copyright DammStanger 2014(c)*******************************************
**--------------File Info---------------------------------------------------------------------------------
* @FileName:    		Project_cfg.h 
* @Version 				V1.0
* @LastmodifiedDate:    2014.12.20
* @brief   				针对本工程的硬件外设与软件功能的开启与禁用的配置
*          				
**--------------------------------------------------------------------------------------------------------
* @Author 				DammStanger            
* @Email:				dammstanger@qq.com
**
*********************************************************************************************************/
#ifndef __PROJECT_CFG_H_
#define __PROJECT_CFG_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/*****************传感器安装角度相关**********************
规定以IMU的坐标系为机体坐标系x,y,z，航向以北偏东为正，
传感器安装的水平偏差角参考航向角规则。
*********************************************************/

#define INSTALORIEN_0_DEGREE					0u		//x=x0,y=y0
#define INSTALORIEN_45_DEGREE					1u		
#define INSTALORIEN_90_DEGREE					2u		//x=y0,y=-x0
#define INSTALORIEN_135_DEGREE					3u
#define INSTALORIEN_180_DEGREE					4u		//x=-x0,y=-y0
#define INSTALORIEN_225_DEGREE					5u
#define INSTALORIEN_270_DEGREE					6u		//x=-y0,y=x0
#define INSTALORIEN_315_DEGREE					7u


/*****************光流定位相关**********************/
#define T_AHRS_UPDATE					0.004											//姿态更新周期
#define T_ALT_UPDATE 					0.1f											//高度更新周期
#define T_RATECTL_UPDATE 				0.004											//角速度控制更新周期
/***************************************************/

/*****************光流定位相关**********************/
//----------------传感器驱动方式--------------------
#define	OPFLOW_SENSOR_ONBOARD_SPI_EN		0		//主控SPI直接驱动
#define OPFLOW_SENSOR_OFFBOARD_USART_EN		1		//从机负责驱动，串口上传数据

#define POSRATECTL							1		//位置控制器为速度控制与否  
//--------------相关程序功能------------------
#define OPFLOW_SOFT_EN						1

/*****************超声测距相关**********************/
#define SONAR_ONBOARD						0		//声呐不在板上
/* Private structure define --------------------------------------------------*/

/* Private variable declaration ----------------------------------------------*/

/* Private function declaration ----------------------------------------------*/

#endif

/********************************************************************************************************
  END FILE
*********************************************************************************************************/
