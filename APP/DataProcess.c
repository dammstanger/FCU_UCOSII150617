/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��DataProcess.c
 * ��	��	�����ɷ���ģʽ�������ݷ���ģʽ�Ը������ݽ��д���������ͬ�Ŀ���Ŀ��
 *                     
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150616
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.4.1
 * ���༭	��2015.6.16
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

/****************************����ͷ�ļ�*******************************************/
#include "math.h"
#include "DataProcess.h"
#include "MPU.h"
#include "usart.h"		
#include "led.h"
#include "Sonar.h"
#include "HMC5883.h"
#include "AHRS.h"
#include "Attitude.h"
#include "Control.h"
#include "SysTick.h"
/****************************��������*********************************************/
u8	FlightMode = UNARMED;
u8	FlyTask	   = TASK_TAKEOFF;
u8	ThrottleMode  = THROTTLE_MANUAL;
u8 VehicleBalComp = 0;
float temp_roll_view;


//-----------------�߶ȿ�����ز���------------------
u16	Control_proc_thro = 1100;				//���ſ��Ƶ��м���
u16 Control_auto_thro= 1100;				//��¼��ɵ�������
int32_t Control_targer_altacc = 0;			//�߶�ACC������Ŀ����ٶ�
bool g_altacc_controller_active = FALSE;

bool g_takeoff 			= 0;
bool g_takeoff_finish 	= 0;
bool g_land	  			= 0;
bool g_land_finish 		= 0;
u8	 g_land_detector	= 0;
u8   g_takeoff_2sdelay  = 0;				//���ڵ������������ʱ

/****************************��������*********************************************/
u16 Thro_SlowStart(void);
void Fly_Task_Update(u8 flytask);
bool Update_Takeoff_Detector(void);
void Reset_Land_Detector(void);
void Reset_Takeoff_Detector(void);
u16 Thro_SlowDown(void);


/********************************************************************************
 * ��������UpdateFlightMode()
 * ����  �����·���ģʽ
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void UpdateFlightMode(RC_DATA rcdata)
{
	if(rcdata.CH5<1500||(rcdata.THROTTLE>1130&&FlightMode==UNARMED))
	{
		FlightMode = UNARMED;
		return ;
	}
	else
	{
		if(rcdata.THROTTLE<=1130)	
		{
			FlightMode = ARMED;
			return ;	
		}
		else if(FlightMode==ARMED&&rcdata.CH6>1500)
		{
			FlightMode = AUTO;
			return ;
		}
		else if(rcdata.CH6<=1500)
		{
			FlightMode = ATTITUDE;
			return ;			
		}	
		else if((FlightMode==POSHOLD||FlightMode==ATTITUDE)&&(rcdata.CH6>1500&&rcdata.CH6<=1800))
		{
			FlightMode = ATLHOLD;
			return ;
		}
		else if(FlightMode==ATLHOLD&&rcdata.CH6>=1800)
		{
			FlightMode = POSHOLD;
			return ;			
		}	
	}
}

/********************************************************************************
 * ��������FlyModeProcess()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void FlyModeProcess()
{
	static int8_t opt_exectcnt = 0;
	static uint32_t auto_lastcall_time = 0;
	uint32_t nowtime = 0;
	UpdateFlightMode(Rc_Data);
	
	opt_exectcnt++;										//��������ִ�м�����
	
	switch(FlightMode)
	{	
		case UNARMED :
			
		case ARMED :
		case ATTITUDE :{
				ThrottleMode = THROTTLE_MANUAL;
				g_altacc_controller_active = FALSE;
				RC_att.roll  = Add_RC_Roll(Rc_Data.ROLL);
				RC_att.pitch = Add_RC_Pitch(Rc_Data.PITCH);
				if(opt_exectcnt==10)					//����λ�÷������Ƶ�ִ�����ڿ��Ƽ�������ÿ10*5ms����һ��
				{
					Optflow_att.roll  = -Add_Optflow_Roll(0);
					Optflow_att.pitch = Add_Optflow_Pitch(0);	
					opt_exectcnt = 0;
				}	
			}break;
		case ATLHOLD :{
				ThrottleMode = THROTTLE_HOLD;
				g_altacc_controller_active = TRUE;
				RC_att.roll  = Add_RC_Roll(Rc_Data.ROLL);
				RC_att.pitch = Add_RC_Pitch(Rc_Data.PITCH);
			if(opt_exectcnt==10)					//����λ�÷������Ƶ�ִ�����ڿ��Ƽ�������ÿ10*4ms����һ��
				{				
					Optflow_att.roll  = -Add_Optflow_Roll(0);
					Optflow_att.pitch = Add_Optflow_Pitch(0);					
					opt_exectcnt = 0;
				}			
			}break;
		case POSHOLD :{
				ThrottleMode = THROTTLE_HOLD;
				g_altacc_controller_active = TRUE;
				RC_att.roll  = Add_RC_Roll(Rc_Data.ROLL);
				RC_att.pitch = Add_RC_Pitch(Rc_Data.PITCH);
				if(opt_exectcnt==10)
				{
					if(Rc_Data.ROLL>1540||Rc_Data.ROLL<1460||Rc_Data.PITCH>1540||Rc_Data.PITCH<1460)
					{
						Optflow_att.roll = -Add_Optflow_Roll(0);		//���ң�ز�����������г���������
						Optflow_att.pitch= Add_Optflow_Pitch(0);
					}
					else
					{
						Optflow_att.roll = -Add_Optflow_Roll(1);
						Optflow_att.pitch= Add_Optflow_Pitch(1);
					}		
					opt_exectcnt = 0;
				}
			}break;
			case AUTO :{
				ThrottleMode = THROTTLE_AUTO;
				g_altacc_controller_active = TRUE;
				RC_att.roll  = Add_RC_Roll(Rc_Data.ROLL);
				RC_att.pitch = Add_RC_Pitch(Rc_Data.PITCH);
				if(opt_exectcnt==10)
				{
					if(Rc_Data.ROLL>1540||Rc_Data.ROLL<1460||Rc_Data.PITCH>1540||Rc_Data.PITCH<1460)
					{
						Optflow_att.roll = -Add_Optflow_Roll(0);		//���ң�ز�����������г���������
						Optflow_att.pitch= Add_Optflow_Pitch(0);
					}
					else
					{
						Optflow_att.roll = -Add_Optflow_Roll(1);
						Optflow_att.pitch= Add_Optflow_Pitch(1);
					}		
					opt_exectcnt = 0;
				}	
				nowtime = GetSystemTime();
				if(nowtime-auto_lastcall_time>40)						//ms,˵���տ�ʼ������
				{
					FlyTask = TASK_TAKEOFF;
					Reset_Land_Detector();
					Reset_Takeoff_Detector();
					Control_auto_thro = 1100;
					g_takeoff_2sdelay = 0;
				}
				auto_lastcall_time = nowtime;
				
				Fly_Task_Update(FlyTask);
			}break;
		default:
			 break;
	}
	Control_proc_att.yaw = Yaw_Last;
	Control_proc_att.yaw = Comput_Ctr_Yaw(Control_proc_att.yaw,Rc_Data);					
	Control_proc_att.roll 	+= RC_att.roll + Optflow_att.roll;				//
	Control_proc_att.pitch 	+= RC_att.pitch+ Optflow_att.pitch;				//
	Control_ulti_att = Control_proc_att;
}


/********************************************************************************
 * ��������Fly_Task_Takeoff()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void Fly_Task_Takeoff()
{
	if(Update_Takeoff_Detector())
	{
		FlyTask = TASK_POSHOLD;
	}
}

/********************************************************************************
 * ��������Fly_Task_PosHold()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void Fly_Task_PosHold()
{
	if(Rc_Data.THROTTLE<=1300)
		FlyTask = TASK_LAND;
}

/********************************************************************************
 * ��������Fly_Task_Land()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void Fly_Task_Land()
{
	if(g_land_finish)
	{
		Control_auto_thro = 1100;			//
		g_takeoff_2sdelay = 0;
		g_Motor_output_enable = DISABLE;		
	}
}
/********************************************************************************
 * ��������Fly_Task_Update()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void Fly_Task_Update(u8 flytask)
{
	switch(flytask)
	{
		case TASK_TAKEOFF :{
			Fly_Task_Takeoff();
		}break;		
		case TASK_POSHOLD :{
			Fly_Task_PosHold();
		}break;	
		case TASK_LAND :{
			Fly_Task_Land();
		}break;	
	}
}


/********************************************************************************
 * ��������ThrottleModeProcess()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void ThrottleModeProcess()
{
	u16 Control_tmp_thro =0;
	int32_t target_tmp_acc = 0;
	switch(ThrottleMode)
	{
		case THROTTLE_MANUAL:{
			if(FlightMode==ARMED||FlightMode==UNARMED)
			{
				g_Motor_output_enable = DISABLE;
				Control_tmp_thro = 1100;
			}
			else
			{
				Control_tmp_thro = (Rc_Data.THROTTLE-1100)*THRO_K + 1100;
				g_Motor_output_enable = ENABLE;
			}
		}break;
		case THROTTLE_HOLD:{
			Control_tmp_thro = (Rc_Data.THROTTLE-1100)*THRO_K + 1100;
			RecordAltHoldPoint(TRUE);
			target_tmp_acc = Alt_Sonar_Controller(0);
//			Control_tmp_thro += Alt_AutoSmooth_Handle(HightHoldVal);
			g_Motor_output_enable = ENABLE;
			
		}break;
		case THROTTLE_AUTO:{
			RecordAltHoldPoint(TRUE);
			if(FlyTask==TASK_TAKEOFF)
				Control_tmp_thro = Thro_SlowStart();
			else
			{
				if(FlyTask==TASK_POSHOLD)
				{
					Set_Alt_Holdtarget(500);
					Control_tmp_thro = Control_auto_thro;
				}
				else if(FlyTask==TASK_LAND)
				{
					Set_Alt_Holdtarget(0);
					Control_tmp_thro = Thro_SlowDown();
				}
				target_tmp_acc = Alt_AutoSmooth_Handle(HightHoldVal);
			}
			if(FlyTask==TASK_LAND&&g_land_finish)
				g_Motor_output_enable = DISABLE;
			else 
				g_Motor_output_enable = ENABLE;
			
		}break;
		case THROTTLE_LAND:{
			Control_tmp_thro = (Rc_Data.THROTTLE-1100)*THRO_K + 1100;
		}break;
		default : break;
	}
	
	if(ThrottleMode!=THROTTLE_HOLD&&ThrottleMode!=THROTTLE_AUTO)
	{
		RecordAltHoldPoint(FALSE);
	}
	Control_proc_thro = Control_tmp_thro;
	Control_targer_altacc = target_tmp_acc;
	if(Control_proc_thro>2000)	Control_proc_thro = 2000;
}

/********************************************************************************
 * ��������AttDataProcess()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void AttDataProcess()
{
	AHRS_Attitude();
}


/********************************************************************************
 * ��������Reset_Takeoff_Detector()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void Reset_Takeoff_Detector()
{
	g_takeoff_finish = 0;
}
/********************************************************************************
 * ��������Update_Takeoff_Detector()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
bool Update_Takeoff_Detector()
{
	if(SonarHight>30)
	{
		if(FlyTask==TASK_TAKEOFF&&!g_takeoff_finish)
		{
			g_takeoff_finish = TRUE;
		}
	}
	return g_takeoff_finish;
}

/********************************************************************************
 * ��������Thro_SlowStart()
 * ����  ��ֻ���������ʱ������
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
u16 Thro_SlowStart()
{
	if(++g_takeoff_2sdelay>=20)					//0.1s����20�μ���Ϊ2s
	{
		g_takeoff_2sdelay = 20;
		if(!g_takeoff_finish)
		{
			Control_auto_thro += 15;				//2s�ڴ�1100�ۼӵ�1400
		}
		if(Control_auto_thro>1600) Control_auto_thro=1600;
	}
	
	return Control_auto_thro;
}


/********************************************************************************
 * ��������Thro_SlowDown()
 * ����  ��ֻ�ڽ�������ʱ������
 * ����  ��-
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
u16 Thro_SlowDown()
{
	if(!g_land_finish&&SonarHight<=200)			//����û����ɣ��߶�С��10cm��ʱ��ʼ��������
	{
		Control_auto_thro -= 10;				//
	}
	if(Control_auto_thro<1100) Control_auto_thro = 1100;
	
	return Control_auto_thro;
}



/********************************************************************************
 * ��������Update_Land_Detector()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
 void Update_Land_Detector()
{
	if(SonarHight<30&&(g_LVel_n.Z>-150&&g_LVel_n.Z<150))
	{
		if(FlyTask==TASK_LAND&&!g_land_finish)
		{
			if(g_land_detector<LAND_DETECTOR_CNT)
			{
				g_land_detector++;
			}
			else 
			{
				g_land_detector = 0;
				g_land_finish = TRUE;
			}
		}
	}
	else 
	{
		g_land_detector = 0;
		g_land_finish = FALSE;
	}
}

/********************************************************************************
 * ��������Reset_Land_Detector()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
 void Reset_Land_Detector()
{
	g_land_detector = 0;
	g_land_finish = FALSE;	
}

/******************************************************************************
/ ��������:��ͷģʽ�½��Ƕ��ںϵ�RC����
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:
	��Z�����ת����	X |	cosW	-sinW	0
						Y | sinW	 cosW	0
						Z | 0		 0		1
******************************************************************************/
//float error,coserr,sinerr;
void HeadfreeMode(RC_DATA *rccomand)
{	
	float error=0,coserr=0,sinerr=0;
	float temp_pitch=0,temp_roll=0;

	error = (Yaw_Last - Headfreehold)*DEG_TO_RAD; 
	coserr = cos(error);//Mycos(error);
	sinerr = sin(error);//Mysin(error);
	temp_roll = rccomand->ROLL -1500;
	temp_pitch =rccomand->PITCH -1500;
	rccomand->PITCH = (temp_pitch*coserr - temp_roll*sinerr)+1500;		//����������������X��Ӧ����Pitch
	rccomand->ROLL  = (temp_roll*coserr + temp_pitch*sinerr)+1500;		//������������Z����ת����̬�任
}



/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/
