/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：app_cfg.h
 * 描	述	：生成飞行模式，并依据飞行模式对各类数据进行处理，产生不同的控制目标
 *                     
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150610
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.4.1
 * 最后编辑	：2015.6.10
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__


/*******************设置任务优先级****0-63****建议用4-60***********/
#define STARTUP_TASK_PRIO       	3	//4 
#define	TASK_LED_PRIO				11	//11
#define TASK_CRUCIALINTERRACT_PRIO	4
#define TASK_MPU6050_PRIO			6	//6	

#define TASK_DEBUG_PRIO				10	//10
#define TASK_PWMout_PRIO			8	//5	
#define TASK_PWMin_PRIO				5	//8
#define TASK_SONAR_PRIO				9	//9
#define TASK_MAGNET_PRIO			7	//7	
/************设置栈大小（单位为 OS_STK ）************/
#define STARTUP_TASK_STK_SIZE   		80   
#define	TASK_LED_STK_SIZE				50		//80
#define TASK_CRUCIALINTERRACT_STK_SIZE 100
#define TASK_MPU6050_STK_SIZE			255
#define TASK_DEBUG_STK_SIZE				80

#define TASK_PWMout_STK_SIZE			200
#define task_PWMin_STK_SIZE				80
#define TASK_SONAR_STK_SIZE				80
#define TASK_MAGNET_STK_SIZE			80

#endif

