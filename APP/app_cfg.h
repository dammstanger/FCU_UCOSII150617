/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��app_cfg.h
 * ��	��	�����ɷ���ģʽ�������ݷ���ģʽ�Ը������ݽ��д���������ͬ�Ŀ���Ŀ��
 *                     
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150610
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.4.1
 * ���༭	��2015.6.10
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__


/*******************�����������ȼ�****0-63****������4-60***********/
#define STARTUP_TASK_PRIO       	3	//4 
#define	TASK_LED_PRIO				11	//11
#define TASK_CRUCIALINTERRACT_PRIO	4
#define TASK_MPU6050_PRIO			6	//6	

#define TASK_DEBUG_PRIO				10	//10
#define TASK_PWMout_PRIO			8	//5	
#define TASK_PWMin_PRIO				5	//8
#define TASK_SONAR_PRIO				9	//9
#define TASK_MAGNET_PRIO			7	//7	
/************����ջ��С����λΪ OS_STK ��************/
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

