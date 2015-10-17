/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Attitude.c
 * ��	��	����̬����
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150615
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.28
 * ���༭	��2015.6.15
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************/

/****************************����ͷ�ļ�*******************************************/
#include "ucos_ii.h"
#include "Attitude.h"
#include "math.h"
#include "Optical_Flow.h"
#include "Sonar.h"
#include "led.h"
#include "Control.h"
#include "SysTick.h"


/****************************�궨��***********************************************/
#define ALT_CURVE_PAR				170
#define ALT_CLIMBRATE_MAX			1000			//��λmm/s ��1��/�� 
#define ALT_DESCENTRATE_MAX			500
/****************************��������*********************************************/

/****************************��������*********************************************/
//-���������------------------------------
bool Saveheadholdflag = 0;
bool headfreeflag = 0;
float Yaw_turn	= 0;
float Headhold = 0;
float Headoriginal_rad=0;
float yaw_print = 0;
float Headfreehold = 0;

ZHOUDATA Vehiclebias={0,0,0};		//-416			//����ƽ��λ������ֵ

EULER_DATA_TYPE RC_att;
EULER_DATA_TYPE Optflow_att;
EULER_DATA_TYPE Control_proc_att;	 //���ƹ��̵���̬
EULER_DATA_TYPE Control_ulti_att;	 //ultimatez���յĿ��ƽǶ�


//-----�߶ȴ������--------------------------------------------------------------------------
u16 HightHoldVal = 0;					//�߶ȱ��ָ���ֵ
s16 HightError = 0;
s16 HightErrorlast = 0;
s32 Hight_i=0;							//�߶�PID
s16 Controller_target_alt=0;

/****************************��������*********************************************/
#if POSRATECTL
/********************************************************************************
 * ��������Add_Optflow_Roll()
 * ����  ��
 * ����  ��execute =1ִ�м�������Ƕȿ��ƣ�=0��ִ��		    	
 * ����  ��ʹ�ù���ʱ��������ʹ�ĺ���Ƕ�
 * ����  ��-
 ********************************************************************************/
float dx_cm_accuml = 0;
float x_rate_error_last = 0;
float Add_Optflow_Roll(u8 execute)
{
	float new_roll=0;
	float target_rate_x,target_rate_dx,rate_error;
	float p_term,d_term,i_term;
		
	if(execute)
	{	
		dx_cm_accuml += Optflow.dx_cm;
		if(dx_cm_accuml>-10||dx_cm_accuml<10) target_rate_x = 0;
		else if(dx_cm_accuml<-10&&dx_cm_accuml>-25) target_rate_x = 1;		//�뾶30cm��Χ�ڣ��ٶ����Ա仯
		else if(dx_cm_accuml>10&&dx_cm_accuml<25)	target_rate_x = -1;		
		else if(dx_cm_accuml>25)					target_rate_x = (25-dx_cm_accuml)/5;
		else if(dx_cm_accuml<-25)					target_rate_x = (-dx_cm_accuml-25)/5;
			
		rate_error = target_rate_x - Optflow.dx_cm;							//������ȥʵ�ʣ�ʵ��С��ƫ��Ϊ��
		target_rate_dx = rate_error - x_rate_error_last;					//��һ���ǰһ��
		x_rate_error_last =rate_error;
		
		p_term = Optflow.Kp*rate_error;			//����PID���㣬�õ����������Ƕ�
		d_term = Optflow.Kd*target_rate_dx;	
		i_term = Optflow.Ki*(-dx_cm_accuml);	//=Ŀ��λ��0-ʵ��λ��dx��Ϊ������
		new_roll = p_term+d_term+i_term;
	}
	else
	{
		dx_cm_accuml = 0;							//���Ƕȿ��ƽ��룬ֹͣ������λ����
		new_roll = 0;								//���Ϊ0
	}

	if(new_roll>10) 	new_roll = 10;	//�����������PID����������ƽǶ�
	if(new_roll<-10)	new_roll = -10;
	
	return new_roll;
}


/********************************************************************************
 * ��������Add_Optflow_Pitch()
 * ����  ��
 * ����  ��execute =1ִ�м�������Ƕȿ��ƣ�=0��ִ��		    	
 * ����  ��ʹ�ù���ʱ��������ʹ�ĸ����Ƕ�
 * ����  ��-
 ********************************************************************************/
float dy_cm_accuml = 0;   
float y_rate_error_last = 0;
float Add_Optflow_Pitch(u8 execute)
{
	float target_rate_y,target_rate_dy,rate_error;
	float new_pitch=0;
	float p_term,d_term,i_term;
		
	if(execute)
	{	
		dy_cm_accuml += Optflow.dy_cm;
		if(dy_cm_accuml>-10||dy_cm_accuml<10) target_rate_y = 0;
		else if(dy_cm_accuml<-10&&dy_cm_accuml>-25) target_rate_y = 1;		//�뾶40cm��Χ�ڣ��ٶ����Ա仯
		else if(dy_cm_accuml>10&&dy_cm_accuml<25)	target_rate_y = -1;		
		else if(dy_cm_accuml>25)					target_rate_y = (25-dy_cm_accuml)/5;
		else if(dy_cm_accuml<-25)					target_rate_y = (-dy_cm_accuml-25)/5;
			
		rate_error = target_rate_y - Optflow.dy_cm;
		target_rate_dy = rate_error - y_rate_error_last;
		y_rate_error_last =rate_error;
		
		p_term = Optflow.Kp*rate_error;			//����PID���㣬�õ����������Ƕ�
		d_term = Optflow.Kd*target_rate_dy;
		i_term = Optflow.Ki*(-dy_cm_accuml);
		new_pitch = p_term+d_term+i_term;
	}
	else
	{
		dy_cm_accuml = 0;
		new_pitch = 0;								//���Ϊ0
	}
	
	if(new_pitch>10) 	new_pitch = 10;
	if(new_pitch<-10)	new_pitch = -10;

	return new_pitch;
}
#else		//λ�ÿ�����

float error_i_x;
/********************************************************************************
 * ��������Add_Optflow_Roll()
 * ����  ��
 * ����  ��execute =1ִ�м�������Ƕȿ��ƣ�=0��ִ��		    	
 * ����  ��ʹ�ù���ʱ��������ʹ�ĺ���Ƕ�
 * ����  ��-
 ********************************************************************************/
float dx_cm_accuml = 0;
float Add_Optflow_Roll(u8 execute)
{
	float target_rate_x,error_rate_x,error_x;
	float p_term,d_term,i_term;
	float d_roll;
	static float new_roll = 0;
	static float error_last_x = 0;
		
	if(execute)
	{
		dx_cm_accuml += Optflow.dx_cm;
		error_x = -Optflow.dx_cm;
		error_rate_x = error_x - error_last_x;
		error_last_x = error_x;
		//�����ϵ������ȡ	error_i_xΪ����ֺ�Ļ���ֵ��ͬ��dx_cm_accuml
		//����������ϵ����ʽ��f_ei=(A-|e(k)|+B)/A,AΪ���䷶Χ��BΪ��������
		if(dx_cm_accuml<=Optflow.irange_min&&dx_cm_accuml>=-Optflow.irange_min)
			error_i_x = dx_cm_accuml;													//ϵ��f_eiΪ1
		else if(dx_cm_accuml>Optflow.irange_min&&
				dx_cm_accuml<=Optflow.irange_max)
			error_i_x = (Optflow.f_ei-(dx_cm_accuml/Optflow.irange))*dx_cm_accuml;
		else if(dx_cm_accuml<-Optflow.irange_min&&
				dx_cm_accuml>=-Optflow.irange_max)	
			error_i_x = (Optflow.f_ei+(dx_cm_accuml/Optflow.irange))*dx_cm_accuml;	//dx_cm_accuml<0,ȡ����ֵ���
		else if(dx_cm_accuml>Optflow.irange_max)
			error_i_x = Optflow.irange_max * Optflow.irange_max / Optflow.irange;	//�ӱ���ֵļ��㴦Ϊ��ֵ���˺󱣳����ֵ																//ϵ��f_eiΪ0
		else if(dx_cm_accuml<-Optflow.irange_max)
			error_i_x = -Optflow.irange_max * Optflow.irange_max / Optflow.irange;	//																
		else
			error_i_x = 0;															//ϵ��f_eiΪ0
		
		p_term = Optflow.Kp*error_x;			//����PID���㣬�õ����������Ƕ�
		d_term = Optflow.Kd*error_rate_x;
		i_term = Optflow.Ki*(-error_i_x);
		d_roll = p_term+d_term+i_term;
		
//		if(d_roll<-2.0) d_roll = -2.0;
//		if(d_roll>2.0) d_roll = 2.0;

		new_roll += d_roll;
	}
	else
	{
		dx_cm_accuml = 0;							//���Ƕȿ��ƽ��룬ֹͣ������λ����
		new_roll = 0;								//���Ϊ0
	}
	
	if(new_roll>7) 	new_roll = 7;	//�����������PID����������ƽǶ�
	if(new_roll<-7)	new_roll = -7;
	
	return new_roll;
}

float error_i_y;
/********************************************************************************
 * ��������Add_Optflow_Pitch()
 * ����  ��
 * ����  ��execute =1ִ�м�������Ƕȿ��ƣ�=0��ִ��		    	
 * ����  ��ʹ�ù���ʱ��������ʹ�ĸ����Ƕ�
 * ����  ��-
 ********************************************************************************/
float dy_cm_accuml = 0;
float Add_Optflow_Pitch(u8 execute)
{
	float error_rate_y,target_rate_dy,error_y;
	float d_pitch=0;
	float p_term,d_term,i_term;
	static float new_pitch = 0;
	static float error_last_y = 0;

	if(execute)
	{	
		dy_cm_accuml += Optflow.dy_cm;
		error_y = -Optflow.dy_cm;
		error_rate_y = error_y - error_last_y;
		error_last_y = error_y;
		//�����ϵ������ȡ	error_i_yΪ����ֺ�Ļ���ֵ��ͬ��dy_cm_accuml
		//����������ϵ����ʽ��f_ei=(A-|e(k)|+B)/A,AΪ���䷶Χ��BΪ��������
		if(dy_cm_accuml<=Optflow.irange_min&&dy_cm_accuml>=-Optflow.irange_min)
			error_i_y = dy_cm_accuml;													//ϵ��f_eiΪ1
		else if(dy_cm_accuml>Optflow.irange_min&&
				dy_cm_accuml<=Optflow.irange_max)
			error_i_y = (Optflow.f_ei-(dy_cm_accuml/Optflow.irange))*dy_cm_accuml;
		else if(dy_cm_accuml<-Optflow.irange_min&&
				dy_cm_accuml>=-Optflow.irange_max)	
			error_i_y = (Optflow.f_ei-(-dy_cm_accuml/Optflow.irange))*dy_cm_accuml;	//dy_cm_accuml<0,ȡ����ֵ���
		else if(dy_cm_accuml>Optflow.irange_max)
			error_i_y = Optflow.irange_max * Optflow.irange_max / Optflow.irange;	//�ӱ���ֵļ��㴦Ϊ��ֵ���˺󱣳����ֵ																//ϵ��f_eiΪ0
		else if(dy_cm_accuml<-Optflow.irange_max)
			error_i_y = -Optflow.irange_max * Optflow.irange_max / Optflow.irange;	//																
		else
			error_i_y = 0;															//ϵ��f_eiΪ0
		
		p_term = Optflow.Kp*error_y;			//����PID���㣬�õ����������Ƕ�
		d_term = Optflow.Kd*error_rate_y;
		i_term = Optflow.Ki*(-error_i_y);
		d_pitch = p_term+d_term+i_term;
		
//		if(d_pitch<-2.0) d_pitch = -2.0;
//		if(d_pitch>2.0)	d_pitch = 2.0;
		
		new_pitch += d_pitch; 					//�����ۼ�
	}
	else
	{
		dy_cm_accuml = 0;
		new_pitch = 0;								//���Ϊ0
	}
	
	if(new_pitch>7) 	new_pitch = 7;
	if(new_pitch<-7)	new_pitch = -7;

	return new_pitch;
}

#endif

/********************************************************************************
 * ��������Comput_Ctr_Yaw()
 * ����  ���������ģʽ�º�������
 * ����  ������ǵĵ�ַ		    
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
float Comput_Ctr_Yaw(float input_yaw,RC_DATA RCcomand)
{	
	u8 flymode = 1;
	float yawerror = 0;
	
	switch(flymode)
	{
		case 1 :{
			if(RCcomand.THROTTLE>=1150)		//����ͷ������
			{
				if(Saveheadholdflag==0)		//ת��ǰ���浱ǰֵ
				{
					Headhold = input_yaw;
					Headoriginal_rad = Yaw_Raw_Rad;
					Saveheadholdflag = 1;	//����Ա���
				}
				
				if(RCcomand.YAW<1420||RCcomand.YAW>1570)	
				{
					Yaw_turn = ((RCcomand.YAW-1500)/400.0);
				}
				else Yaw_turn = 0;	
				
				Headhold += Yaw_turn;
				
				if(Headhold>=360) Headhold = Headhold - 360;
				else if(Headhold<0) Headhold = Headhold + 360;		//Ŀ�꺽��ǵķ�Χ�޶���0-2pi֮��
				//------------------------------------------------
				if(180<Headhold&&Headhold<360)
				{
					if(0<=input_yaw&&input_yaw<(Headhold-180)) yawerror = Headhold-input_yaw-360;
					else yawerror = Headhold-input_yaw;
				}
				else
				{
					if(0<=input_yaw&&input_yaw<(Headhold+180)) yawerror = Headhold-input_yaw;
					else yawerror = Headhold-input_yaw+360;
				}
				//-------------------------------------------------
			}
			else
			{
				Saveheadholdflag = 0;	//�˳�ͷ������ģʽʱ�������־���㣬�Ա��´�ʹ�á�
				yawerror = 0;			//ƫ������
			}
		}break;
			
		default:break;	
	}
	return 	yawerror;	//������õ��Ĳ�ֵ������ڿ���

}

/********************************************************************************
 * ��������VehicleBal_Compensate()
 * ����  ������ƽ�ⲹ��
 * ����  ��u16 roll,u16 pitch   
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
void VehicleBal_Compensate(u16 roll,u16 pitch)
{
	if(roll>1520||roll<1480)
	{
		Vehiclebias.X += ((roll-1500)/380.0);
		
	}
	else ;
	if(pitch>1520||pitch<1480)
	{
		Vehiclebias.Y += ((pitch-1500)/380.0);
	}
	else ;	
}


/********************************************************************************
 * ��������Add_RC_Roll()
 * ����  ��ң������ĽǶ�
 * ����  ��rc_roll
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
float Add_RC_Roll(u16 rc_roll)
{
	float roll = 0;
	if(rc_roll>1515||rc_roll<1485)
	roll = -(rc_roll+Vehiclebias.X-1500)/15.0; //�������ֵ = ��ǰ���� -��ң��ֵ-1500��*һ������ϵ�� (�൱��Ŀ���)15����
	else roll= -Vehiclebias.X/15.0; 	

	return roll;
}

/********************************************************************************
 * ��������Add_RC_Pitch()
 * ����  ��ң������ĽǶ�
 * ����  ��rc_pitch  
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
float Add_RC_Pitch(u16 rc_pitch)
{
	float pitch = 0;
	if(rc_pitch>1515||rc_pitch<1485)
	pitch = -(rc_pitch+Vehiclebias.Y-1500)/15.0;
	else pitch = -Vehiclebias.Y/15.0;
	return pitch;
}


void RecordAltHoldPoint(bool enable)
{
	static bool Savethrottle_fistin = FALSE;
	if(enable)
	{
		if(Savethrottle_fistin==0)						//�������ߵ�һ�ν���ʱ���浱ǰֵ
		{	
			HightHoldVal = 600;							//������Ϊ��ǰ�߶�
			Controller_target_alt = SonarHight;			//�ս���ʱ��ʹ�߶ȿ������Ŀ���Ŀ��=��ʵĿ��
			Savethrottle_fistin = 1;					//����ѱ���
		}
	}
	else
	{
		if(Savethrottle_fistin==1)
		{	
			Savethrottle_fistin = 0;	//���������
		}
	}
}

void Set_Alt_Holdtarget(u16 alt_target)
{
	HightHoldVal = alt_target;
}

float vacc_error = 0;
float vertical_acc_mm = 0;

/********************************************************************************
 * ��������AltAcc_Controller()
 * ����  ��
 * ����  ��vtarget_accel  (mm/0.1s^2)
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
int16_t AltAcc_Controller(int32_t vtarget_accel)
{
//	static float vacc_error = 0;
	static float vacc_error_last = 0;
	static int32_t lasttime = 0;
	int32_t nowtime = 0;
	
	nowtime = GetSystemTime();
	if(nowtime-lasttime>100)						//ms,˵���ձ�����
	{
		PID_ACC_ALT.Iout = 0;
		vacc_error = 0;
	}
	lasttime = nowtime;	
	
	vertical_acc_mm = ACC_Linear_n_mm.Z;
	vacc_error = vacc_error + 0.1116 * ((vtarget_accel - vertical_acc_mm) - vacc_error); //0.1116f
	//P D
	PID_ACC_ALT.Pout = PID_ACC_ALT.P * vacc_error;

	if(PID_ACC_ALT.D!=0)
	{
		PID_ACC_ALT.Dout = PID_ACC_ALT.D * (vacc_error - vacc_error_last);
		vacc_error_last = vacc_error;	
	}

	//I
	if(PID_ACC_ALT.I!=0)
	{
		if((PID_ACC_ALT.Iout>0&&vacc_error<0)||(PID_ACC_ALT.Iout<0&&vacc_error>0))
		PID_ACC_ALT.Iout += PID_ACC_ALT.I * vacc_error;
		if(PID_ACC_ALT.Iout > PID_ACC_ALT.Imax) PID_ACC_ALT.Iout  = PID_ACC_ALT.Imax;
		else if(PID_ACC_ALT.Iout < -PID_ACC_ALT.Imax) PID_ACC_ALT.Iout  = -PID_ACC_ALT.Imax;		
	}	
	//ALL
	PID_ACC_ALT.ALLout = PID_ACC_ALT.Pout + PID_ACC_ALT.Dout + PID_ACC_ALT.Iout;			//
	
	return PID_ACC_ALT.ALLout;
}

float vertical_rate_error = 0;			//��λmm
float vtarget_filter_rate = 0;
int32_t target_output;
/********************************************************************************
 * ��������AltRate_Controller()
 * ����  ��	�߶��ٶȿ����� g_LVel_n.Z	��λ��
 * ����  ��vtarget_rate  (mm/s)
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
int16_t AltRate_Controller(int32_t vtarget_rate)
{
//	static float vertical_rate_error = 0;			//��λmm
//	static float vtarget_filter_rate = 0;
    int32_t vtarget_speed_delta;   					// The change in requested speed
//  int32_t target_output;  
	static int32_t lasttime = 0;
	int32_t nowtime = 0;
	
	nowtime = GetSystemTime();
	if(nowtime-lasttime>200)						//ms,˵���ձ�����
	{
		vtarget_filter_rate = vtarget_rate;
		vertical_rate_error = 0;
	}
	lasttime = nowtime;
//	vertical_rate_error = vtarget_rate - g_LVel_n.Z;
	vertical_rate_error = vertical_rate_error + 0.35f * ((vtarget_rate - g_LVel_n.Z) - vertical_rate_error);	//0.20085f
	vtarget_speed_delta = 0.35f * (vtarget_rate - vtarget_filter_rate);
	vtarget_filter_rate = vtarget_filter_rate + vtarget_speed_delta;
	target_output = vtarget_speed_delta * P_RATEFILT_ALT.P;						//�ٶȲ���൱�ڼ��ٶ�
	
	P_RATE_ALT.Pout = P_RATE_ALT.P * vertical_rate_error;			//
	
	target_output += (int32_t)P_RATE_ALT.Pout;
//	target_output = (int32_t)P_RATE_ALT.Pout;

	return target_output;
}

float desired_rate;
/********************************************************************************
 * ��������Alt_Root_Controller()
 * ����  �������ϼ�������Ŀ��߶ȼ����ʱ��Ҫ�ﵽ��Ŀ���ٶȣ����͸���ֱ������ٶȿ�����
 * ����  ��target_alt  (mm)
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
int16_t Alt_Root_Controller(int32_t target_alt)
{
	int32_t alt_error;
	int32_t linear_distance;
//	float desired_rate;
	int16_t retval;
	
	alt_error = target_alt - SonarHight;
	
	if(P_ROOT_ALT.P!=0)
	{
		linear_distance = ALT_CURVE_PAR/(2*P_ROOT_ALT.P*P_ROOT_ALT.P);
		if     (alt_error >  2*linear_distance)
			desired_rate = sqrt(2.0*ALT_CURVE_PAR*(alt_error-linear_distance));
		else if(alt_error < -2*linear_distance)
			desired_rate = -sqrt(2.0*ALT_CURVE_PAR*(-alt_error-linear_distance));		
		else
			desired_rate = P_ROOT_ALT.P * alt_error;
	}
	else
		desired_rate = 0;

	if(desired_rate>ALT_CLIMBRATE_MAX)	desired_rate = ALT_CLIMBRATE_MAX;
	if(desired_rate<-ALT_DESCENTRATE_MAX)	desired_rate = -ALT_DESCENTRATE_MAX;		//-1000<=target_rate<=1000	mm/s
		
	retval = AltRate_Controller(desired_rate);//(desired_rate);
	return retval;
}

/********************************************************************************
 * ��������Alt_AutoSmooth_Handle()
 * ����  ��
 * ����  ��target_rate  (mm)
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
int16_t Alt_AutoSmooth_Handle(u16 target_alt)
{
	int32_t alt_change;
	
	alt_change = target_alt - Controller_target_alt;
	if(alt_change<0||alt_change>0)
	{
		if(alt_change>ALT_CLIMBRATE_MAX*0.1)	Controller_target_alt += ALT_CLIMBRATE_MAX * 0.1 ;		//����0.1s
		else if(alt_change<-ALT_DESCENTRATE_MAX*0.1)	Controller_target_alt += -ALT_DESCENTRATE_MAX*0.1 ;		//����0.1s
		else Controller_target_alt += alt_change ;			
	}
	
	if(Controller_target_alt>SonarHight+1000)	Controller_target_alt = SonarHight+1000;
	else if(Controller_target_alt<SonarHight-1000)	Controller_target_alt = SonarHight-1000;
	
	return Alt_Root_Controller(Controller_target_alt);
}

/********************************************************************************
 * ��������Alt_PilotSmooth_Handle()
 * ����  ��
 * ����  ��target_rate  (mm)
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
int16_t Alt_PilotSmooth_Handle(int32_t target_rate)
{
//	if(target_rate>0||target_rate<0);
//	Controller_target_alt += target_rate*0.1;			//����100ms
//	Controller_target_alt = Controller_target_alt + 0.1*(HightHoldVal-Controller_target_alt);

	
	if(Controller_target_alt<HightHoldVal)
	{
		Controller_target_alt+=25;
		if(Controller_target_alt>HightHoldVal) Controller_target_alt = HightHoldVal;
	}
	else 
	{
		Controller_target_alt-=25;
		if(Controller_target_alt<HightHoldVal) Controller_target_alt = HightHoldVal;		
	}
	return Alt_Root_Controller(Controller_target_alt);
}


/********************************************************************************
 * ��������Alt_Sonar_Controller()
 * ����  ��
 * ����  ��target_rate  (mm)
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
int16_t Alt_Sonar_Controller(int32_t target_rate)
{
	int16_t alt_rate_autoset;
	if(target_rate>0||target_rate<0)
		HightHoldVal += target_rate*0.02;
	
	if(HightHoldVal>SonarHight+1000)	HightHoldVal = SonarHight+1000;
	if(HightHoldVal<SonarHight-1000)	HightHoldVal = SonarHight-1000;
	
	HightError = HightHoldVal - SonarHight;
	P_SONAR_ALT.Pout = HightError * P_SONAR_ALT.P;
	alt_rate_autoset = (int16_t)P_SONAR_ALT.Pout;
	target_rate = target_rate + alt_rate_autoset;
	
	if(target_rate>ALT_CLIMBRATE_MAX)	target_rate = ALT_CLIMBRATE_MAX;
	if(target_rate<-ALT_DESCENTRATE_MAX)	target_rate = -ALT_DESCENTRATE_MAX;		//-1000<=target_rate<=1000	mm/s
		
	return Alt_PilotSmooth_Handle(target_rate);
}

float AltOld_Controller()
{
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
	return PID_HIGH.ALLout;
}

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/
