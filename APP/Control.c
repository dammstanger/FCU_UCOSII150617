/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Controll.c
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
#include 	"Timer_PWMin.h"
#include 	"Timer_PWMout.h"
#include 	"Control.h"
#include 	"DataProcess.h"
#include 	"led.h"						//LED��������
#include	"Sonar.h"

//----------------��ȫ����------------------------
#define LIMITANGLGE_MAX		50.0
#define LIMITANGLGE_MIN		-50.0

#define YAWOUTPUT_MAX		450
#define YAWOUTPUT_MIN		-450

#define Filter_D_ATT		7.9577e-3	//1/(2*pi*fcut)	fcut��ֹƵ��=20Hz

u8 Unlock = 0;							//������־

float MaxAngle = 0.0;					//���ڼ�¼���ֵ���̬�ǵ����ֵ
float MinAngle = 0.0;					//���ڼ�¼���ֵ���̬�ǵ���Сֵ

float Target_rate_roll = 0;				//������Ŀ������
float Target_rate_pitch = 0;			//������Ŀ������

//-------------������ʹ������---------------------
FunctionalState g_Motor_output_enable = DISABLE;
//-------------------------------------------------------------------------------
P_PAR 	P_SONAR_ALT;
P_PAR 	P_ROOT_ALT;
P_PAR 	P_RATE_ALT;
P_PAR 	P_RATEFILT_ALT;
PID_PAR PID_ACC_ALT;
PID_PAR PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_HIGH,PID_POS,PID_STABLE,PID_POS_OF;
s16 Moto_PWM_1=0,Moto_PWM_2=0,Moto_PWM_3=0,Moto_PWM_4=0,Moto_PWM_5=0,Moto_PWM_6=0;
float rol_i=0,pit_i=0,yaw_i=0;
////-----�߶ȴ������--------------------------------------------------------------------------

//-------------------------------------------

void PID_Para_Init()
{
	PID_ROL.P = 3.57;//3.57;
	PID_ROL.D = 0.051;//0.052;
	PID_ROL.I = 0.000;//0.0015;
	PID_ROL.Imax = 100;
	
	PID_PIT.P = 3.57;//3.57;
	PID_PIT.D = 0.051;//0.052;
	PID_PIT.I = 0.000;//0.0015;
	PID_PIT.Imax = 100;
	
	PID_YAW.P = 6.0;	//3.9;	
	PID_YAW.D = 0.065;	//0.051;	//0.001;	
	PID_YAW.I = 0.01;
	PID_YAW.Imax = 40;

	PID_STABLE.P = 4.5;
	
	PID_ROL.ALLout = 0;
	PID_PIT.ALLout = 0;
	PID_YAW.ALLout = 0;
	
	PID_HIGH.P=  0.1;	//0.087;		
	PID_HIGH.D = 1.7;	//1.0;
	PID_HIGH.I = 0.002;	//0;//0.001;//0.002;
	PID_HIGH.Imax = 40;
	
	PID_HIGH.Pout = 0;
	PID_HIGH.Iout = 0;
	PID_HIGH.Dout = 0;
	
	PID_POS_OF.P = 0.09;
	PID_POS_OF.D = 2.5;
	PID_POS_OF.I = 0.001;
	
	P_SONAR_ALT.P = 1;
	
	P_ROOT_ALT.P = 0.8;
	
	P_RATE_ALT.P = 5.0;		//3.8
	P_RATEFILT_ALT.P = 0.5;	//1.2
	
	PID_ACC_ALT.P = 0.21;	//0.16
	PID_ACC_ALT.I = 0;		//0.01
	PID_ACC_ALT.D = 0;
	PID_ACC_ALT.Imax = 100;
}



/********************************************************************************
 * ��������CONTROL
 * ����  ��
 * ����  ��-	    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void CONTROL(EULER_DATA_TYPE ctr_angle,int32_t alt_target_acc,u16 throttlelast,bool enable)
{	
	static u8	altexecnt = 0;						//�߶�acc������ִ�м�����
	static u16  altacc_thro_save = 0;				//���ڱ���ÿһ�θ߶�ACC�����������
	
	rol_i+=ctr_angle.roll;							//�����ۼ�
	if(rol_i>4000) rol_i=4000;						//�����޷�
	if(rol_i<-4000)rol_i=-4000;
	
	pit_i+=ctr_angle.pitch;
	if(pit_i>4000) pit_i=4000;
	if(pit_i<-4000)pit_i=-4000;
	 

	PID_ROL.Pout = PID_ROL.P * ctr_angle.roll;
	PID_ROL.Dout = PID_ROL.D * GYRO_Last.Y;//Roll��Ӧ����Y�����ת
	PID_ROL.Iout = PID_ROL.I * rol_i;

	PID_PIT.Pout = PID_PIT.P * ctr_angle.pitch;
	PID_PIT.Dout = PID_PIT.D * GYRO_Last.X;//Ptich��Ӧ����X�����ת
	PID_PIT.Iout = PID_PIT.I * pit_i;

	PID_YAW.Pout = PID_YAW.P*ctr_angle.yaw;
	PID_YAW.Dout = PID_YAW.D * GYRO_Last.Z;
	
	yaw_i += ctr_angle.yaw;
	PID_YAW.Iout = PID_YAW.I * yaw_i;
	if(PID_YAW.Iout >PID_YAW.Imax) PID_YAW.Iout  = PID_YAW.Imax;
	else if(PID_YAW.Iout<-PID_YAW.Imax) PID_YAW.Iout  = -PID_YAW.Imax;	
	//------------------------------------------------------------------

	PID_ROL.ALLout = PID_ROL.Pout + PID_ROL.Dout+ PID_ROL.Iout;
	PID_PIT.ALLout = PID_PIT.Pout + PID_PIT.Dout+ PID_PIT.Iout;
	PID_YAW.ALLout = PID_YAW.Pout + PID_YAW.Dout+ PID_YAW.Iout;
 
	if(PID_YAW.ALLout>YAWOUTPUT_MAX)
	{
		PID_YAW.ALLout=YAWOUTPUT_MAX;
	}
	else if(PID_YAW.ALLout<YAWOUTPUT_MIN)
	{
		PID_YAW.ALLout=YAWOUTPUT_MIN;
	}

	//----------------------�߶�ACC������------------------------------
	if(altexecnt++==10)					//ÿ40msִ��һ��
	{
		altexecnt = 0;
		if(g_altacc_controller_active)
		altacc_thro_save = AltAcc_Controller(alt_target_acc);		
	}
	
	if(g_altacc_controller_active)
		throttlelast += altacc_thro_save;
	if(throttlelast>2000) throttlelast = 2000;
	else if(throttlelast<1000) throttlelast = 1000;
	//-------------------------------------------------------------
	
	if(enable)
	{
			Moto_PWM_1 = throttlelast + PID_ROL.ALLout + PID_PIT.ALLout + PID_YAW.ALLout;
			Moto_PWM_2 = throttlelast - PID_ROL.ALLout + PID_PIT.ALLout - PID_YAW.ALLout;
			Moto_PWM_3 = throttlelast - PID_ROL.ALLout - PID_PIT.ALLout + PID_YAW.ALLout;
			Moto_PWM_4 = throttlelast + PID_ROL.ALLout - PID_PIT.ALLout - PID_YAW.ALLout;
		
//----------������-----------------------------------------------			
//			Moto_PWM_1 = throttlelast + PID_PIT.ALLout;
//			Moto_PWM_2 = throttlelast + PID_PIT.ALLout;
//			Moto_PWM_3 = throttlelast - PID_PIT.ALLout;
//			Moto_PWM_4 = throttlelast - PID_PIT.ALLout;	
			
//			Moto_PWM_1 = throttlelast + PID_ROL.ALLout;
//			Moto_PWM_2 = throttlelast - PID_ROL.ALLout;
//			Moto_PWM_3 = throttlelast - PID_ROL.ALLout;
//			Moto_PWM_4 = throttlelast + PID_ROL.ALLout;	
			
//			Moto_PWM_1 = throttlelast  + PID_YAW.ALLout;
//			Moto_PWM_2 = throttlelast  - PID_YAW.ALLout;
//			Moto_PWM_3 = throttlelast  + PID_YAW.ALLout;
//			Moto_PWM_4 = throttlelast  - PID_YAW.ALLout;			
	}
	else
	{
		Moto_PWM_1 = MOTO_PWM_MIN;
		Moto_PWM_2 = MOTO_PWM_MIN;
		Moto_PWM_3 = MOTO_PWM_MIN;
		Moto_PWM_4 = MOTO_PWM_MIN;
		
		pit_i=0;
		rol_i=0;
		yaw_i=0;
		Hight_i=0;
	}
	MotoOutPut(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4,Moto_PWM_5,Moto_PWM_6);
				
}

/********************************************************************************
 * ��������SafeDeal
 * ����  ��
 * ����  ��-	    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
bool SafeDeal(float rol, float pit)	
{
	if(MaxAngle<rol)	MaxAngle = rol;
	if(MaxAngle<pit)	MaxAngle = pit;
	if(MinAngle>rol)	MinAngle = rol;
	if(MinAngle>pit)	MinAngle = pit;
	
	if(FlightMode==ARMED||(MinAngle<=LIMITANGLGE_MIN)||(MaxAngle>=LIMITANGLGE_MAX)||!g_Motor_output_enable) 
	{
		return FALSE; 					//�ɻ�������̬�쳣����ǹ���ʱ�Զ�����
	}
	else if(FlightMode==UNARMED)
	{
		MaxAngle = 0;			//��ֵ����				
		MinAngle = 0;
		return FALSE;
	}
	return TRUE;
}


/***************************************************************************************************

void HighPIDContrl()
{
//	static s16 HightError;
	static u8  PIDstep = 40;			//PID�Ĳ���
	static s16 HighPIDMin = 1100;
	static s16 HighPIDMax = 1800;
	//��������
	//����б��
	throttlelast = (Rc_Data.THROTTLE-1100)*THRO_K + 1100;
	
	if(Rc_Data.CH6>=1500)				//��ʾ��������
	{	
		if(SavePreThroflag==0)			//�������ߵ�һ�ν���ʱ���浱ǰֵ
		{	LED2(ON);
			HightHoldVal = SonarHight;	//������Ϊ��ǰ�߶�
//			HightHoldVal = 1000;		//ֻ����0.8�׶���
			SavePreThroflag = 1;		//����ѱ���
		} 

		if(throttlelast - PIDstep <1100) HighPIDMin = 1100;		//�ֶ�������ƣ�
		else HighPIDMin = throttlelast - PIDstep;				//���ֶ�����Ϊ��׼��PID����޷�
		if(throttlelast + PIDstep >1800) HighPIDMax = 1800;
		else HighPIDMax = throttlelast + PIDstep;
		
		//------�Ը߶Ƚ���PID����-------------
		HightError = HightHoldVal - SonarHight;
		if(HightError>100||HightError<-100) HightError = HightErrorlast;
		HightErrorlast = HightError;
		//P D
		PID_HIGH.Pout = PID_HIGH.P * HightError;
		PID_HIGH.Dout = -PID_HIGH.D * g_LVel_n.Z;
		//I
		Hight_i += HightError;
		PID_HIGH.Iout = PID_HIGH.I * Hight_i;
		if(PID_HIGH.Iout >PID_HIGH.Imax) PID_HIGH.Iout  = PID_HIGH.Imax;
		else if(PID_HIGH.Iout<-PID_HIGH.Imax) PID_HIGH.Iout  = -PID_HIGH.Imax;		
		//ALL
		PID_HIGH.ALLout = PID_HIGH.Pout + PID_HIGH.Dout + PID_HIGH.Iout;
		if(PID_HIGH.ALLout < -PIDstep)
			PID_HIGH.ALLout = -PIDstep;
		else if(PID_HIGH.ALLout > PIDstep) 
			PID_HIGH.ALLout = PIDstep;
	}//------------------------------------------------------------------------------------------------------
	else
	{	
		if(SavePreThroflag==1)
		{	LED2(OFF);
			SavePreThroflag = 0;	//���������
		}
	}
}
*/

//	SafeDeal(ctr_angle.roll,ctr_angle.pitch);					//���պ���
		
//-------������λУ��-------------------------------------------------
//	if(RCcomand.CH6>2500)
//	{
//		VehicleBalComp = 1;					//��־�������
//	}
//	else
//	{
//		VehicleBalComp = 0;					//CH6��Сʱ�����²���ƫ��
//	}
//	if(VehicleBalComp)
//	{
//		VehicleBal_Compensate(RCcomand.ROLL,RCcomand.PITCH);
//		ctr_angle.roll=ctr_angle.roll-Vehiclebias.X/15.0; //�������ֵ = ��ǰ�� -��ң��ֵ-1500��*һ������ϵ�� (�൱��Ŀ���)15����
//		ctr_angle.pitch=ctr_angle.pitch-Vehiclebias.Y/15.0;	
//	}
//	else
//	{
//		if(RCcomand.ROLL>1510||RCcomand.ROLL<1490)
//		ctr_angle.roll=ctr_angle.roll-(RCcomand.ROLL+Vehiclebias.X-1500)/15.0; //�������ֵ = ��ǰ���� -��ң��ֵ-1500��*һ������ϵ�� (�൱��Ŀ���)15����
//		else ctr_angle.roll=ctr_angle.roll-Vehiclebias.X/15.0; 	
//		if(RCcomand.PITCH>1510||RCcomand.PITCH<1490)
//		ctr_angle.pitch=ctr_angle.pitch-(RCcomand.PITCH+Vehiclebias.Y-1500)/15.0;
//		else ctr_angle.pitch=ctr_angle.pitch-Vehiclebias.Y/15.0;	
//	}


