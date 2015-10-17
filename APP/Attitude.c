/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Attitude.c
 * 描	述	：姿态处理
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150615
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.28
 * 最后编辑	：2015.6.15
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************/

/****************************包含头文件*******************************************/
#include "ucos_ii.h"
#include "Attitude.h"
#include "math.h"
#include "Optical_Flow.h"
#include "Sonar.h"
#include "led.h"
#include "Control.h"
#include "SysTick.h"


/****************************宏定义***********************************************/
#define ALT_CURVE_PAR				170
#define ALT_CLIMBRATE_MAX			1000			//单位mm/s 即1米/秒 
#define ALT_DESCENTRATE_MAX			500
/****************************变量声明*********************************************/

/****************************变量定义*********************************************/
//-航向处理变量------------------------------
bool Saveheadholdflag = 0;
bool headfreeflag = 0;
float Yaw_turn	= 0;
float Headhold = 0;
float Headoriginal_rad=0;
float yaw_print = 0;
float Headfreehold = 0;

ZHOUDATA Vehiclebias={0,0,0};		//-416			//机体平衡位置修正值

EULER_DATA_TYPE RC_att;
EULER_DATA_TYPE Optflow_att;
EULER_DATA_TYPE Control_proc_att;	 //控制过程的姿态
EULER_DATA_TYPE Control_ulti_att;	 //ultimatez最终的控制角度


//-----高度处理变量--------------------------------------------------------------------------
u16 HightHoldVal = 0;					//高度保持给定值
s16 HightError = 0;
s16 HightErrorlast = 0;
s32 Hight_i=0;							//高度PID
s16 Controller_target_alt=0;

/****************************函数声明*********************************************/
#if POSRATECTL
/********************************************************************************
 * 函数名：Add_Optflow_Roll()
 * 描述  ：
 * 输入  ：execute =1执行加入光流角度控制，=0不执行		    	
 * 返回  ：使用光流时，光流驱使的横滚角度
 * 调用  ：-
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
		else if(dx_cm_accuml<-10&&dx_cm_accuml>-25) target_rate_x = 1;		//半径30cm范围内，速度线性变化
		else if(dx_cm_accuml>10&&dx_cm_accuml<25)	target_rate_x = -1;		
		else if(dx_cm_accuml>25)					target_rate_x = (25-dx_cm_accuml)/5;
		else if(dx_cm_accuml<-25)					target_rate_x = (-dx_cm_accuml-25)/5;
			
		rate_error = target_rate_x - Optflow.dx_cm;							//给定减去实际，实际小则偏差为正
		target_rate_dx = rate_error - x_rate_error_last;					//新一项减前一项
		x_rate_error_last =rate_error;
		
		p_term = Optflow.Kp*rate_error;			//误差的PID运算，得到控制期望角度
		d_term = Optflow.Kd*target_rate_dx;	
		i_term = Optflow.Ki*(-dx_cm_accuml);	//=目标位置0-实际位置dx作为积分项
		new_roll = p_term+d_term+i_term;
	}
	else
	{
		dx_cm_accuml = 0;							//外界角度控制接入，停止光流定位控制
		new_roll = 0;								//输出为0
	}

	if(new_roll>10) 	new_roll = 10;	//限制允许光流PID输出的最大控制角度
	if(new_roll<-10)	new_roll = -10;
	
	return new_roll;
}


/********************************************************************************
 * 函数名：Add_Optflow_Pitch()
 * 描述  ：
 * 输入  ：execute =1执行加入光流角度控制，=0不执行		    	
 * 返回  ：使用光流时，光流驱使的俯仰角度
 * 调用  ：-
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
		else if(dy_cm_accuml<-10&&dy_cm_accuml>-25) target_rate_y = 1;		//半径40cm范围内，速度线性变化
		else if(dy_cm_accuml>10&&dy_cm_accuml<25)	target_rate_y = -1;		
		else if(dy_cm_accuml>25)					target_rate_y = (25-dy_cm_accuml)/5;
		else if(dy_cm_accuml<-25)					target_rate_y = (-dy_cm_accuml-25)/5;
			
		rate_error = target_rate_y - Optflow.dy_cm;
		target_rate_dy = rate_error - y_rate_error_last;
		y_rate_error_last =rate_error;
		
		p_term = Optflow.Kp*rate_error;			//误差的PID运算，得到控制期望角度
		d_term = Optflow.Kd*target_rate_dy;
		i_term = Optflow.Ki*(-dy_cm_accuml);
		new_pitch = p_term+d_term+i_term;
	}
	else
	{
		dy_cm_accuml = 0;
		new_pitch = 0;								//输出为0
	}
	
	if(new_pitch>10) 	new_pitch = 10;
	if(new_pitch<-10)	new_pitch = -10;

	return new_pitch;
}
#else		//位置控制器

float error_i_x;
/********************************************************************************
 * 函数名：Add_Optflow_Roll()
 * 描述  ：
 * 输入  ：execute =1执行加入光流角度控制，=0不执行		    	
 * 返回  ：使用光流时，光流驱使的横滚角度
 * 调用  ：-
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
		//变积分系数的求取	error_i_x为变积分后的积分值不同于dx_cm_accuml
		//变积分区间的系数公式：f_ei=(A-|e(k)|+B)/A,A为区间范围，B为区间下限
		if(dx_cm_accuml<=Optflow.irange_min&&dx_cm_accuml>=-Optflow.irange_min)
			error_i_x = dx_cm_accuml;													//系数f_ei为1
		else if(dx_cm_accuml>Optflow.irange_min&&
				dx_cm_accuml<=Optflow.irange_max)
			error_i_x = (Optflow.f_ei-(dx_cm_accuml/Optflow.irange))*dx_cm_accuml;
		else if(dx_cm_accuml<-Optflow.irange_min&&
				dx_cm_accuml>=-Optflow.irange_max)	
			error_i_x = (Optflow.f_ei+(dx_cm_accuml/Optflow.irange))*dx_cm_accuml;	//dx_cm_accuml<0,取绝对值变号
		else if(dx_cm_accuml>Optflow.irange_max)
			error_i_x = Optflow.irange_max * Optflow.irange_max / Optflow.irange;	//视变积分的极点处为最值，此后保持这个值																//系数f_ei为0
		else if(dx_cm_accuml<-Optflow.irange_max)
			error_i_x = -Optflow.irange_max * Optflow.irange_max / Optflow.irange;	//																
		else
			error_i_x = 0;															//系数f_ei为0
		
		p_term = Optflow.Kp*error_x;			//误差的PID运算，得到控制期望角度
		d_term = Optflow.Kd*error_rate_x;
		i_term = Optflow.Ki*(-error_i_x);
		d_roll = p_term+d_term+i_term;
		
//		if(d_roll<-2.0) d_roll = -2.0;
//		if(d_roll>2.0) d_roll = 2.0;

		new_roll += d_roll;
	}
	else
	{
		dx_cm_accuml = 0;							//外界角度控制接入，停止光流定位控制
		new_roll = 0;								//输出为0
	}
	
	if(new_roll>7) 	new_roll = 7;	//限制允许光流PID输出的最大控制角度
	if(new_roll<-7)	new_roll = -7;
	
	return new_roll;
}

float error_i_y;
/********************************************************************************
 * 函数名：Add_Optflow_Pitch()
 * 描述  ：
 * 输入  ：execute =1执行加入光流角度控制，=0不执行		    	
 * 返回  ：使用光流时，光流驱使的俯仰角度
 * 调用  ：-
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
		//变积分系数的求取	error_i_y为变积分后的积分值不同于dy_cm_accuml
		//变积分区间的系数公式：f_ei=(A-|e(k)|+B)/A,A为区间范围，B为区间下限
		if(dy_cm_accuml<=Optflow.irange_min&&dy_cm_accuml>=-Optflow.irange_min)
			error_i_y = dy_cm_accuml;													//系数f_ei为1
		else if(dy_cm_accuml>Optflow.irange_min&&
				dy_cm_accuml<=Optflow.irange_max)
			error_i_y = (Optflow.f_ei-(dy_cm_accuml/Optflow.irange))*dy_cm_accuml;
		else if(dy_cm_accuml<-Optflow.irange_min&&
				dy_cm_accuml>=-Optflow.irange_max)	
			error_i_y = (Optflow.f_ei-(-dy_cm_accuml/Optflow.irange))*dy_cm_accuml;	//dy_cm_accuml<0,取绝对值变号
		else if(dy_cm_accuml>Optflow.irange_max)
			error_i_y = Optflow.irange_max * Optflow.irange_max / Optflow.irange;	//视变积分的极点处为最值，此后保持这个值																//系数f_ei为0
		else if(dy_cm_accuml<-Optflow.irange_max)
			error_i_y = -Optflow.irange_max * Optflow.irange_max / Optflow.irange;	//																
		else
			error_i_y = 0;															//系数f_ei为0
		
		p_term = Optflow.Kp*error_y;			//误差的PID运算，得到控制期望角度
		d_term = Optflow.Kd*error_rate_y;
		i_term = Optflow.Ki*(-error_i_y);
		d_pitch = p_term+d_term+i_term;
		
//		if(d_pitch<-2.0) d_pitch = -2.0;
//		if(d_pitch>2.0)	d_pitch = 2.0;
		
		new_pitch += d_pitch; 					//增量累加
	}
	else
	{
		dy_cm_accuml = 0;
		new_pitch = 0;								//输出为0
	}
	
	if(new_pitch>7) 	new_pitch = 7;
	if(new_pitch<-7)	new_pitch = -7;

	return new_pitch;
}

#endif

/********************************************************************************
 * 函数名：Comput_Ctr_Yaw()
 * 描述  ：计算各种模式下航控制量
 * 输入  ：输出角的地址		    
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
float Comput_Ctr_Yaw(float input_yaw,RC_DATA RCcomand)
{	
	u8 flymode = 1;
	float yawerror = 0;
	
	switch(flymode)
	{
		case 1 :{
			if(RCcomand.THROTTLE>=1150)		//进入头部锁定
			{
				if(Saveheadholdflag==0)		//转动前保存当前值
				{
					Headhold = input_yaw;
					Headoriginal_rad = Yaw_Raw_Rad;
					Saveheadholdflag = 1;	//标记以保存
				}
				
				if(RCcomand.YAW<1420||RCcomand.YAW>1570)	
				{
					Yaw_turn = ((RCcomand.YAW-1500)/400.0);
				}
				else Yaw_turn = 0;	
				
				Headhold += Yaw_turn;
				
				if(Headhold>=360) Headhold = Headhold - 360;
				else if(Headhold<0) Headhold = Headhold + 360;		//目标航向角的范围限定在0-2pi之间
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
				Saveheadholdflag = 0;	//退出头部锁定模式时，保存标志清零，以备下次使用。
				yawerror = 0;			//偏差清零
			}
		}break;
			
		default:break;	
	}
	return 	yawerror;	//将计算得到的差值输出用于控制

}

/********************************************************************************
 * 函数名：VehicleBal_Compensate()
 * 描述  ：机体平衡补偿
 * 输入  ：u16 roll,u16 pitch   
 * 返回  ：-
 * 调用  ：-
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
 * 函数名：Add_RC_Roll()
 * 描述  ：遥控输出的角度
 * 输入  ：rc_roll
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
float Add_RC_Roll(u16 rc_roll)
{
	float roll = 0;
	if(rc_roll>1515||rc_roll<1485)
	roll = -(rc_roll+Vehiclebias.X-1500)/15.0; //横滚量差值 = 当前横量 -（遥控值-1500）*一个比例系数 (相当于目标角)15步进
	else roll= -Vehiclebias.X/15.0; 	

	return roll;
}

/********************************************************************************
 * 函数名：Add_RC_Pitch()
 * 描述  ：遥控输出的角度
 * 输入  ：rc_pitch  
 * 返回  ：-
 * 调用  ：-
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
		if(Savethrottle_fistin==0)						//开启定高第一次进入时保存当前值
		{	
			HightHoldVal = 600;							//给定就为当前高度
			Controller_target_alt = SonarHight;			//刚进入时，使高度控制器的控制目标=现实目标
			Savethrottle_fistin = 1;					//标记已保存
		}
	}
	else
	{
		if(Savethrottle_fistin==1)
		{	
			Savethrottle_fistin = 0;	//清除保存标记
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
 * 函数名：AltAcc_Controller()
 * 描述  ：
 * 输入  ：vtarget_accel  (mm/0.1s^2)
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
int16_t AltAcc_Controller(int32_t vtarget_accel)
{
//	static float vacc_error = 0;
	static float vacc_error_last = 0;
	static int32_t lasttime = 0;
	int32_t nowtime = 0;
	
	nowtime = GetSystemTime();
	if(nowtime-lasttime>100)						//ms,说明刚被运行
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

float vertical_rate_error = 0;			//单位mm
float vtarget_filter_rate = 0;
int32_t target_output;
/********************************************************************************
 * 函数名：AltRate_Controller()
 * 描述  ：	高度速度控制器 g_LVel_n.Z	单位是
 * 输入  ：vtarget_rate  (mm/s)
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
int16_t AltRate_Controller(int32_t vtarget_rate)
{
//	static float vertical_rate_error = 0;			//单位mm
//	static float vtarget_filter_rate = 0;
    int32_t vtarget_speed_delta;   					// The change in requested speed
//  int32_t target_output;  
	static int32_t lasttime = 0;
	int32_t nowtime = 0;
	
	nowtime = GetSystemTime();
	if(nowtime-lasttime>200)						//ms,说明刚被运行
	{
		vtarget_filter_rate = vtarget_rate;
		vertical_rate_error = 0;
	}
	lasttime = nowtime;
//	vertical_rate_error = vtarget_rate - g_LVel_n.Z;
	vertical_rate_error = vertical_rate_error + 0.35f * ((vtarget_rate - g_LVel_n.Z) - vertical_rate_error);	//0.20085f
	vtarget_speed_delta = 0.35f * (vtarget_rate - vtarget_filter_rate);
	vtarget_filter_rate = vtarget_filter_rate + vtarget_speed_delta;
	target_output = vtarget_speed_delta * P_RATEFILT_ALT.P;						//速度差就相当于加速度
	
	P_RATE_ALT.Pout = P_RATE_ALT.P * vertical_rate_error;			//
	
	target_output += (int32_t)P_RATE_ALT.Pout;
//	target_output = (int32_t)P_RATE_ALT.Pout;

	return target_output;
}

float desired_rate;
/********************************************************************************
 * 函数名：Alt_Root_Controller()
 * 描述  ：根据上级给定的目标高度计算此时需要达到的目标速度，并送给垂直方向的速度控制器
 * 输入  ：target_alt  (mm)
 * 返回  ：-
 * 调用  ：-
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
 * 函数名：Alt_AutoSmooth_Handle()
 * 描述  ：
 * 输入  ：target_rate  (mm)
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
int16_t Alt_AutoSmooth_Handle(u16 target_alt)
{
	int32_t alt_change;
	
	alt_change = target_alt - Controller_target_alt;
	if(alt_change<0||alt_change>0)
	{
		if(alt_change>ALT_CLIMBRATE_MAX*0.1)	Controller_target_alt += ALT_CLIMBRATE_MAX * 0.1 ;		//周期0.1s
		else if(alt_change<-ALT_DESCENTRATE_MAX*0.1)	Controller_target_alt += -ALT_DESCENTRATE_MAX*0.1 ;		//周期0.1s
		else Controller_target_alt += alt_change ;			
	}
	
	if(Controller_target_alt>SonarHight+1000)	Controller_target_alt = SonarHight+1000;
	else if(Controller_target_alt<SonarHight-1000)	Controller_target_alt = SonarHight-1000;
	
	return Alt_Root_Controller(Controller_target_alt);
}

/********************************************************************************
 * 函数名：Alt_PilotSmooth_Handle()
 * 描述  ：
 * 输入  ：target_rate  (mm)
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
int16_t Alt_PilotSmooth_Handle(int32_t target_rate)
{
//	if(target_rate>0||target_rate<0);
//	Controller_target_alt += target_rate*0.1;			//周期100ms
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
 * 函数名：Alt_Sonar_Controller()
 * 描述  ：
 * 输入  ：target_rate  (mm)
 * 返回  ：-
 * 调用  ：-
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
	//------对高度进行PID控制-------------
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
