/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：AHRS.c
 * 描	述	：航姿参考算法
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.1.150612
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.28
 * 最后编辑	：2015.6.13
**********************************************************************************************/

/****************************包含头文件*******************************************/
#include "AHRS.h"
#include "math.h"
#include "Quaternion.h"
#include "MPU.h"
#include "HMC5883.h"
#include "Attitude.h"

/****************************宏定义***********************************************/
#define FILTERZISE_ACC	10
/****************************变量声明*********************************************/

/****************************变量定义*********************************************/
ZHOUDATA GYRO_Raw,GYRO_Last,GYRO_OFFSET={23,-41,10};	//简单校准值
ZHOUDATA GYRO_Int={0,0,0};								//陀螺仪不修正积分(静态测试用)
ZHOUDATA ACC_Raw;										//原始的加速度
ZHOUDATA ACC_Last;										//最终要使用的加速度
ZHOUDATA ACC_OFFSET={0};//{60,145,-310};						//加速度静态偏移
ZHOUDATA ACC_Linear_n;									//剔除重力的线性加速度
ZHOUDATA ACC_Linear_n_mm;								//剔除重力的线性加速度,单位mm

ZHOUDATA g_LVelimu_n;									//n系上imu估计的线速度
ZHOUDATA g_LVel_n;										//n系上的线速度

s16 ACC_1G = 8192;											//ACC z轴测到的1倍重力加速度
tg_HMC5883L_FLOAT magtmp;

int FiltArray_AX[FILTERZISE_ACC] = {0};
int FiltArray_AY[FILTERZISE_ACC] = {0};
int FiltArray_AZ[FILTERZISE_ACC] = {0};

float Yaw_Last= 0,Roll_Last = 0,Pitch_Last= 0;
float Yaw_Raw,Roll_Raw,Pitch_Raw;
float Yaw_Raw_Rad,Roll_Raw_Rad,Pitch_Raw_Rad;		//弧度制
float SinRoll=0,SinPitch=0,CosRoll=0,CosPitch=0,SinYaw,CosYaw;

/****************************函数声明*********************************************/
float Mag_Yaw_Cal(tg_HMC5883L_FLOAT *mag_crect_val);

/********************************************************************************
 * 函数名：MPUGyroZeroCal()
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/
u8 MPUGyroZeroCal()
{	static u16 calcnt = 0;
	static s32 tempX = 0,tempY = 0,tempZ = 0;
	
	GYRO_Raw.X = gyro[0];						//陀螺仪原始数据
	GYRO_Raw.Y = gyro[1];
	GYRO_Raw.Z = gyro[2];
	
	if(GYRO_Raw.X>500||GYRO_Raw.X<-500			//pass掉前几个错误的数据 500为估计的零偏最大范围
	||GYRO_Raw.Y>500||GYRO_Raw.Y<-500
	||GYRO_Raw.Z>500||GYRO_Raw.Z<-500)
	return 1;
	
	tempX += GYRO_Raw.X;
	tempY += GYRO_Raw.Y;
	tempZ += GYRO_Raw.Z;
	calcnt++;
	if(calcnt==100)								//加100次求平均
	{
		GYRO_OFFSET.X = tempX/calcnt;
		GYRO_OFFSET.Y = tempY/calcnt;
		GYRO_OFFSET.Z = tempZ/calcnt;
		tempX = 0;
		tempY = 0;		
		tempZ = 0;
		calcnt = 0;								//复位参数，以备将来的调用
		return 0;
	}
	else return 1;
}



/********************************************************************************
 * 函数名：MPUAccZeroCal_GravityMeasure()
 * 描述  ：create a quaternion from Euler angles
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/
u8 MPUAccZeroCal_GravityMeasure()
{	static u16 calcnt = 0;
	static s32 tempX = 0,tempY = 0,tempZ = 0;
//	
	ACC_Raw.X = accel[0];						//陀螺仪原始数据
	ACC_Raw.Y = accel[1];
	ACC_Raw.Z = accel[2];
	
	if(ACC_Raw.X>1000||ACC_Raw.X<-1000			//pass掉前几个错误的数据 1000为估计的零偏最大范围
	||ACC_Raw.Y>1000||ACC_Raw.Y<-1000
	)
	return 1;
	
	tempX += ACC_Raw.X;
	tempY += ACC_Raw.Y;
	tempZ += ACC_Raw.Z;							//一般不矫正
	calcnt++;
	if(calcnt==100)								//加100次求平均
	{
		ACC_OFFSET.X = tempX/calcnt;
		ACC_OFFSET.Y = tempY/calcnt;
		ACC_1G		 = tempZ/calcnt;			//当前重力值
		tempX = 0;
		tempY = 0;		
		tempZ = 0;
		calcnt = 0;								//复位参数，以备将来的调用
		return 0;
	}
	else return 1;
}



#define IMU_Kp 0.8f                         // 值越大，对加速度的参考就越重 proportional gain governs rate of convergence to accelerometer/magnetometer
#define IMU_Ki 0.001f//0.0003f              // 值大了过度会有过冲，integral gain governs rate of convergence of gyroscope biases
#define IMU_halfT 0.002f//        // half the sample period???????

float exInt = 0, eyInt = 0, ezInt = 0;    	// scaled integral error
float exInt_mag = 0, eyInt_mag = 0, ezInt_mag = 0;    	// scaled integral error of mag
/********************************************************************************
 * 函数名：IMU_update()
 * 描述  ：create a quaternion from Euler angles
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/

void IMU_update(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float  wx, wy, wz; 
	float ex, ey, ez;
	
	//计算基于四元数的坐标变换阵
	Quaternion_rotation_matrix();				
	
	norm = sqrt(ax*ax + ay*ay + az*az);      	//acc?????
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;								//归一化ACC三个轴

	// estimated direction of gravity            
	wx = g_MxCnb.a.Z;							//认为导航坐标系z轴与重力平行					
	wy = g_MxCnb.b.Z;
	wz = g_MxCnb.c.Z;

	// error is sum of cross product(向量积) between reference direction of fields and direction measured by sensors
	ex = (ay*wz - az*wy) ;
	ey = (az*wx - ax*wz) ;
	ez = (ax*wy - ay*wx) ;

	exInt = exInt + ex * IMU_Ki;
	eyInt = eyInt + ey * IMU_Ki;
	ezInt = ezInt + ez * IMU_Ki;

	// adjusted gyroscope measurements
	gx = gx + IMU_Kp*ex + exInt;
	gy = gy + IMU_Kp*ey + eyInt;
	gz = gz + IMU_Kp*ez + ezInt;

	// integrate quaternion rate and normalise						   
	Quat.Q0 = Quat.Q0 + (-Quat.Q1*gx - Quat.Q2*gy - Quat.Q3*gz)*IMU_halfT;
	Quat.Q1 = Quat.Q1 + ( Quat.Q0*gx + Quat.Q2*gz - Quat.Q3*gy)*IMU_halfT;
	Quat.Q2 = Quat.Q2 + ( Quat.Q0*gy - Quat.Q1*gz + Quat.Q3*gx)*IMU_halfT;
	Quat.Q3 = Quat.Q3 + ( Quat.Q0*gz + Quat.Q1*gy - Quat.Q2*gx)*IMU_halfT;

	// normalise quaternion
	norm = sqrt(Quat.Q0*Quat.Q0 + Quat.Q1*Quat.Q1 + Quat.Q2*Quat.Q2 + Quat.Q3*Quat.Q3);

	Quat.Q0 = Quat.Q0 / norm;
	Quat.Q1 = Quat.Q1 / norm;
	Quat.Q2 = Quat.Q2 / norm;
	Quat.Q3 = Quat.Q3 / norm;
}

#define AHRS_Kp 1.0f                         // proportional gain governs rate of convergence to accelerometer/magnetometer
#define AHRS_Ki 0.001f//0.0003f              // integral gain governs rate of convergence of gyroscope biases
#define AHRS_halfT 0.002f//0.0025f         // half the sample period

#define AHRS_MAG_Kp 0.0f                      //	
#define AHRS_MAG_Ki 0.005f					          // 	
#define AHRS_MAG_halfT 0.0067f				       	//
/********************************************************************************
 * 函数名：AHRS_update()S.O.H. Madgwick
 * 描述  ：update attitudequaternion from Euler angles
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/

void AHRS_update(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz)
{
	static u8 periodcnt=0;
	float norm;
	float hx, hy, hz, bx, bz;
	float  wx, wy, wz,vx, vy, vz;//
	float ex, ey, ez,ex_mag,ey_mag,ez_mag;

	// auxiliary variables to reduce number of repeated operations
	float q0q0 = Quat.Q0*Quat.Q0;
	float q0q1 = Quat.Q0*Quat.Q1;
	float q0q2 = Quat.Q0*Quat.Q2;
	float q0q3 = Quat.Q0*Quat.Q3;
	float q1q1 = Quat.Q1*Quat.Q1;
	float q1q2 = Quat.Q1*Quat.Q2;
	float q1q3 = Quat.Q1*Quat.Q3;
	float q2q2 = Quat.Q2*Quat.Q2;
	float q2q3 = Quat.Q2*Quat.Q3;
	float q3q3 = Quat.Q3*Quat.Q3;
		
	norm = sqrt(ax*ax + ay*ay + az*az);       
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;														//归一化ACC三个轴

	norm = sqrt(mx*mx + my*my + mz*mz);          
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;														//归一化Mag三个轴

	// estimated direction of gravity and flux (w and v)
	wx = 2*(q1q3 - q0q2);											//估计重力在机体坐标系中的矢量 因为在R系中，重力矢量（0,0,g）,所以
	wy = 2*(q0q1 + q2q3);
	wz = q0q0 - q1q1 - q2q2 + q3q3;

	//从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值）
	// compute reference direction of flux
	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2); 

	/*计算地理坐标系下的磁场矢量bxyz（参考值）。
	*因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），所以by=0，bx=某值
	*但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
	*我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
	*磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
	*因为by=0，所以就简化成(bx*bx)  = ((hx*hx) + (hy*hy))。可算出bx。
	*/
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;

	//我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
	vx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	vy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	vz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

	// error is sum of cross product(向量积) between reference direction of fields and direction measured by sensors
	ex = (ay*wz - az*wy) + (my*vz - mz*vy);
	ey = (az*wx - ax*wz) + (mz*vx - mx*vz);
	ez = (ax*wy - ay*wx) + (mx*vy - my*vx);

	exInt = exInt + ex * AHRS_Ki;
	eyInt = eyInt + ey * AHRS_Ki;
	ezInt = ezInt + ez * AHRS_Ki;

	// adjusted gyroscope measurements
	gx = gx + AHRS_Kp*ex + exInt ;
	gy = gy + AHRS_Kp*ey + eyInt ;
	gz = gz + AHRS_Kp*ez + ezInt ;

	// integrate quaternion rate and normalise						   
	Quat.Q0 = Quat.Q0 + (-Quat.Q1*gx - Quat.Q2*gy - Quat.Q3*gz)*AHRS_halfT;
	Quat.Q1 = Quat.Q1 + ( Quat.Q0*gx + Quat.Q2*gz - Quat.Q3*gy)*AHRS_halfT;
	Quat.Q2 = Quat.Q2 + ( Quat.Q0*gy - Quat.Q1*gz + Quat.Q3*gx)*AHRS_halfT;
	Quat.Q3 = Quat.Q3 + ( Quat.Q0*gz + Quat.Q1*gy - Quat.Q2*gx)*AHRS_halfT;

#if 0
	periodcnt++;
	if(periodcnt==3)
	{
		periodcnt = 0;
		
//		norm = sqrt(mx*mx + my*my + mz*mz);          
//		mx = mx / norm;
//		my = my / norm;
//		mz = mz / norm;														//归一化Mag三个轴
//		
//		//从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值）
//		// compute reference direction of flux
//		hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
//		hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//		hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2); 

//		/*计算地理坐标系下的磁场矢量bxyz（参考值）。
//		*因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），所以by=0，bx=某值
//		*但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
//		*我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
//		*磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
//		*因为by=0，所以就简化成(bx*bx)  = ((hx*hx) + (hy*hy))。可算出bx。
//		*/
//		bx = sqrt((hx*hx) + (hy*hy));
//		bz = hz;

//		//我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
//		vx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//		vy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//		vz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

		ex_mag = (my*vz - mz*vy);
		ey_mag = (mz*vx - mx*vz);
		ez_mag = (mx*vy - my*vx);
		
		exInt_mag = exInt_mag + ex_mag * AHRS_MAG_Ki;
		eyInt_mag = eyInt_mag + ey_mag * AHRS_MAG_Ki;
		ezInt_mag = ezInt_mag + ez_mag * AHRS_MAG_Ki;

	 // adjusted gyroscope measurements
		gx = gx + AHRS_MAG_Kp*ex_mag + exInt_mag;
		gy = gy + AHRS_MAG_Kp*ey_mag + exInt_mag;
		gz = gz + AHRS_MAG_Kp*ez_mag + exInt_mag;
		
		// integrate quaternion rate and normalise						   
		Quat.Q0 = Quat.Q0 + (-Quat.Q1*gx - Quat.Q2*gy - Quat.Q3*gz)*AHRS_MAG_halfT;
		Quat.Q1 = Quat.Q1 + ( Quat.Q0*gx + Quat.Q2*gz - Quat.Q3*gy)*AHRS_MAG_halfT;
		Quat.Q2 = Quat.Q2 + ( Quat.Q0*gy - Quat.Q1*gz + Quat.Q3*gx)*AHRS_MAG_halfT;
		Quat.Q3 = Quat.Q3 + ( Quat.Q0*gz + Quat.Q1*gy - Quat.Q2*gx)*AHRS_MAG_halfT;
	}
#endif
	// normalise quaternion
	norm = sqrt(Quat.Q0*Quat.Q0 + Quat.Q1*Quat.Q1 + Quat.Q2*Quat.Q2 + Quat.Q3*Quat.Q3);

	Quat.Q0 = Quat.Q0 / norm;
	Quat.Q1 = Quat.Q1 / norm;
	Quat.Q2 = Quat.Q2 / norm;
	Quat.Q3 = Quat.Q3 / norm;
	
}



/********************************************************************************
 * 函数名：AHRS_SpeedUpdate()
 * 描述  ：实现未补偿的速度更新,注意暂未对速度更新进行划桨旋转补偿
 * 输入  ：-	    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/

void AHRS_SpeedUpdate()
{
	ACC_Linear_n.X = ACC_Last.X*g_MxCnb.a.X + ACC_Last.Y*g_MxCnb.b.X + ACC_Last.Z*g_MxCnb.c.X;
	ACC_Linear_n.Y = ACC_Last.X*g_MxCnb.a.Y + ACC_Last.Y*g_MxCnb.b.Y + ACC_Last.Z*g_MxCnb.c.Y;
	ACC_Linear_n.Z = ACC_Last.X*g_MxCnb.a.Z + ACC_Last.Y*g_MxCnb.b.Z + ACC_Last.Z*g_MxCnb.c.Z - ACC_1G;	//坐标变换后还要减去重力加速度

	ACC_Linear_n_mm.Z = ACC_Linear_n.Z*Acc_G*1000;
//	g_LVelimu_n.X += ACC_Linear_n.X*Acc_G*T_AHRS_UPDATE;
//	g_LVelimu_n.Y += ACC_Linear_n.Y*Acc_G*T_AHRS_UPDATE;
//	g_LVelimu_n.Z += ACC_Linear_n.Z*Acc_G*T_AHRS_UPDATE;
}
/********************************************************************************
 * 函数名：AHRS_Attitude()
 * 描述  ：航资参考系统，仅以IMU实现
 * 输入  ：-	    	
 * 返回  ：-
 * 调用  ：-
 ********************************************************************************/

void AHRS_Attitude()
{
	u8 i;
	int tempval1 = 0,tempval2 = 0,tempval3 = 0;
//	float sin2x,sin2y,abs_Roll_Last,abs_Pitch_Last;
	static u8 filpoint = 0;			//滤波器指针

										//非DMP数据处理
//---陀螺仪----------------------------------------------------------------------------------------------------
	GYRO_Raw.X = gyro[0]-GYRO_OFFSET.X;						//陀螺仪原始数据
	GYRO_Raw.Y = gyro[1]-GYRO_OFFSET.Y;
	GYRO_Raw.Z = gyro[2]-GYRO_OFFSET.Z;
	
	if(GYRO_Raw.X<(-5000)||GYRO_Raw.X>(5000)) GYRO_Raw.X = GYRO_Last.X; //如果读数有误，则使用原来值
	if(GYRO_Raw.Y<(-5000)||GYRO_Raw.Y>(5000)) GYRO_Raw.Y = GYRO_Last.Y; 
	if(GYRO_Raw.Z<(-5000)||GYRO_Raw.Z>(5000)) GYRO_Raw.Z = GYRO_Last.Z; 
	
	GYRO_Last.X = GYRO_Raw.X;
	GYRO_Last.Y = GYRO_Raw.Y;
	GYRO_Last.Z = GYRO_Raw.Z;	

//---加速度----------------------------------------------------------------------------------------------------
	ACC_Raw.X = accel[0]-ACC_OFFSET.X;			//陀螺仪原始数据
	ACC_Raw.Y = accel[1]-ACC_OFFSET.Y;
	ACC_Raw.Z = accel[2];//-ACC_OFFSET.Z;			//

	FiltArray_AX[filpoint] = (int)ACC_Raw.X;	//每采集一个，放入队列中
	FiltArray_AY[filpoint] = (int)ACC_Raw.Y;
	FiltArray_AZ[filpoint] = (int)ACC_Raw.Z;

	for(i=0;i<FILTERZISE_ACC;i++)
	{
		tempval1 += FiltArray_AX[i];
		tempval2 += FiltArray_AY[i];
		tempval3 += FiltArray_AZ[i];
	}
	ACC_Last.X = tempval1/FILTERZISE_ACC;
	ACC_Last.Y = tempval2/FILTERZISE_ACC;
	ACC_Last.Z = tempval3/FILTERZISE_ACC;			//求和求平均
	filpoint++;
	if(filpoint==FILTERZISE_ACC) filpoint=0;

//---磁阻------------------------------------------------------------------------------------------------------------
	magtmp.hx = -hmc5883l.hx;									//进行符号和数据类型转换
	magtmp.hy = -hmc5883l.hy;									//磁阻传感器的的坐标系X,Y刚好与机体坐标系相反
	magtmp.hz = hmc5883l.hz;
	
//---四元数刷新姿态----------------------------------------------------------------------------------------------	
	IMU_update(GYRO_Last.X*Gyro_Gr,GYRO_Last.Y*Gyro_Gr,GYRO_Last.Z*Gyro_Gr,
				ACC_Last.X,ACC_Last.Y,ACC_Last.Z);
	
//	AHRS_update(GYRO_Last.X*Gyro_Gr,GYRO_Last.Y*Gyro_Gr,GYRO_Last.Z*Gyro_Gr,
//						 ACC_Last.X,ACC_Last.Y,ACC_Last.Z,magtmp.hx,magtmp.hy,magtmp.hz);
	
	//---横滚、俯仰角度----------------------------------------------------------------------------------------------	
	/*
	// roll这里Roll和Pitch有些书上说的正好相反	这里理清：ACC_X对应Roll 
	*/	
	Roll_Raw_Rad  = asin(-2 * Quat.Q1 * Quat.Q3 + 2 * Quat.Q0* Quat.Q2); 
	Pitch_Raw_Rad = atan2(2 * Quat.Q2 * Quat.Q3 + 2 * Quat.Q0 * Quat.Q1, -2 * Quat.Q1 * Quat.Q1 - 2 * Quat.Q2* Quat.Q2 + 1); // ptich
//	Yaw_Raw_Rad   = atan2(2 * Quat.Q1 * Quat.Q2 + 2 * Quat.Q0 * Quat.Q3, -2 * Quat.Q2*Quat.Q2 - 2 * Quat.Q3* Quat.Q3 + 1);

//--更新相关三角函数值-------------------------------------------------------
	CosPitch = cos(Pitch_Raw_Rad);
	SinPitch = sin(Pitch_Raw_Rad);
	CosRoll = cos(Roll_Raw_Rad);
	SinRoll = sin(Roll_Raw_Rad);

	Yaw_Raw_Rad = Mag_Yaw_Cal(&Magdecp);							//单位弧度
	
	Yaw_Raw		= Yaw_Raw_Rad*57.3f;
	Roll_Raw	= Roll_Raw_Rad*57.3f;
	Pitch_Raw	= Pitch_Raw_Rad*57.3f;
	

	if(Roll_Raw<=(-180)||Roll_Raw>=(180))	Roll_Raw = Roll_Last; 
	if(Pitch_Raw<=(-180)||Pitch_Raw>=(180))	Pitch_Raw = Pitch_Last; 
	
	Roll_Last = Roll_Raw;
	Pitch_Last = Pitch_Raw; 
	Yaw_Last = Yaw_Raw;
	
	Control_proc_att.roll = Roll_Last;
	Control_proc_att.pitch= Pitch_Last;
	
	AHRS_SpeedUpdate();
}



/********************************************************************************
 * 函数名：Mag_Yaw_Cal()
 * 描述  ：磁阻传感器倾角补偿，并计算由磁阻获得的航向角
 * 输入  ：保存计算结果的mag结构体地址	    	
 * 返回  ：磁阻航向角
 * 调用  ：-
 ********************************************************************************/
float Mag_Yaw_Cal(tg_HMC5883L_FLOAT *mag_crect_val)
{
	float sin_roll_mag,cos_roll_mag,cos_pitch_mag,sin_pitch_mag;
	
	sin_roll_mag = -SinRoll;			//加负号因为FCU转接的5883坐标系与6050的刚好相反
	cos_roll_mag = CosRoll;
	sin_pitch_mag= -SinPitch;
	cos_pitch_mag= CosPitch;
	
	//利用姿态转移矩阵将磁阻X,Y轴校正到水平面，由于5883的x轴与y轴与机体所定义的坐标系的
	//x轴、y轴相反，所以以下等式的pitch和roll与书中公式中的二者是对调的。
	mag_crect_val->hx = hmc5883l.hx*cos_roll_mag + hmc5883l.hz*sin_roll_mag;			//已验证	
	mag_crect_val->hy = hmc5883l.hx*sin_pitch_mag*sin_roll_mag + hmc5883l.hy*cos_pitch_mag - hmc5883l.hz*cos_roll_mag*sin_pitch_mag;
	
	HMC5883L_Cal(mag_crect_val);			//(&hmc5883l);
	return mag_crect_val->m_yaw;			//弧度
}

#define HP_Par 0.8f
#define LP_Par 0.2f
#define i_Par  0.03f
float errorh_int = 0;
/********************************************************************************
 * 函数名：AHRS_AltitudeVel()
 * 描述  ：导航垂直速度
 * 输入  ：    	
 * 返回  ：
 * 调用  ：-
 ********************************************************************************/
float AHRS_AltitudeVel(float imuacc_z,float sonarspd)
{
	static float altvel = 0;
	float tempval = Acc_G*1000*T_ALT_UPDATE;
	float errorh = g_LVel_n.Z - sonarspd;
	float z_rate_n;
	
	errorh_int += i_Par*errorh;						//加入积分修正加速度误差
//	errorh_int = 0;
	if(errorh_int>100) errorh_int = 100;
	if(errorh_int<-100)errorh_int = -100;
	z_rate_n = HP_Par*(g_LVel_n.Z + imuacc_z*tempval) + LP_Par*(sonarspd-errorh_int);
	return z_rate_n;
}

//#define Alt_Kp	0.85f  //0.05f  //1.25f//0.85f
//#define Alt_Ki	0.025f //0.015f //0.05f//0.025f
//float errorhint = 0;
//void AHRS_AltitudeVel(float imuacc_z,float sonarspd)
//{
////	static float altvel = 0;
//	static float tempval = Acc_G*100;			//tempval = Acc_G*1000*T_ALT_UPDATE
//	float errorh = g_LVel_n.Z - sonarspd;
//	
//	imuacc_z = imuacc_z/10;							//不知哪一步单位换算问题，这里需要除以10
//	errorhint += Alt_Ki*errorh;						//加入积分修正加速度误差
//	imuacc_z -=( errorhint + Alt_Kp*errorh);
//	g_LVel_n.Z += imuacc_z*tempval;
//}

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

