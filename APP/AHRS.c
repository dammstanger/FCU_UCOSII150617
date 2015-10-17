/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��AHRS.c
 * ��	��	�����˲ο��㷨
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.1.150612
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.28
 * ���༭	��2015.6.13
**********************************************************************************************/

/****************************����ͷ�ļ�*******************************************/
#include "AHRS.h"
#include "math.h"
#include "Quaternion.h"
#include "MPU.h"
#include "HMC5883.h"
#include "Attitude.h"

/****************************�궨��***********************************************/
#define FILTERZISE_ACC	10
/****************************��������*********************************************/

/****************************��������*********************************************/
ZHOUDATA GYRO_Raw,GYRO_Last,GYRO_OFFSET={23,-41,10};	//��У׼ֵ
ZHOUDATA GYRO_Int={0,0,0};								//�����ǲ���������(��̬������)
ZHOUDATA ACC_Raw;										//ԭʼ�ļ��ٶ�
ZHOUDATA ACC_Last;										//����Ҫʹ�õļ��ٶ�
ZHOUDATA ACC_OFFSET={0};//{60,145,-310};						//���ٶȾ�̬ƫ��
ZHOUDATA ACC_Linear_n;									//�޳����������Լ��ٶ�
ZHOUDATA ACC_Linear_n_mm;								//�޳����������Լ��ٶ�,��λmm

ZHOUDATA g_LVelimu_n;									//nϵ��imu���Ƶ����ٶ�
ZHOUDATA g_LVel_n;										//nϵ�ϵ����ٶ�

s16 ACC_1G = 8192;											//ACC z��⵽��1���������ٶ�
tg_HMC5883L_FLOAT magtmp;

int FiltArray_AX[FILTERZISE_ACC] = {0};
int FiltArray_AY[FILTERZISE_ACC] = {0};
int FiltArray_AZ[FILTERZISE_ACC] = {0};

float Yaw_Last= 0,Roll_Last = 0,Pitch_Last= 0;
float Yaw_Raw,Roll_Raw,Pitch_Raw;
float Yaw_Raw_Rad,Roll_Raw_Rad,Pitch_Raw_Rad;		//������
float SinRoll=0,SinPitch=0,CosRoll=0,CosPitch=0,SinYaw,CosYaw;

/****************************��������*********************************************/
float Mag_Yaw_Cal(tg_HMC5883L_FLOAT *mag_crect_val);

/********************************************************************************
 * ��������MPUGyroZeroCal()
 * ����  ��
 * ����  ��-		    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/
u8 MPUGyroZeroCal()
{	static u16 calcnt = 0;
	static s32 tempX = 0,tempY = 0,tempZ = 0;
	
	GYRO_Raw.X = gyro[0];						//������ԭʼ����
	GYRO_Raw.Y = gyro[1];
	GYRO_Raw.Z = gyro[2];
	
	if(GYRO_Raw.X>500||GYRO_Raw.X<-500			//pass��ǰ������������� 500Ϊ���Ƶ���ƫ���Χ
	||GYRO_Raw.Y>500||GYRO_Raw.Y<-500
	||GYRO_Raw.Z>500||GYRO_Raw.Z<-500)
	return 1;
	
	tempX += GYRO_Raw.X;
	tempY += GYRO_Raw.Y;
	tempZ += GYRO_Raw.Z;
	calcnt++;
	if(calcnt==100)								//��100����ƽ��
	{
		GYRO_OFFSET.X = tempX/calcnt;
		GYRO_OFFSET.Y = tempY/calcnt;
		GYRO_OFFSET.Z = tempZ/calcnt;
		tempX = 0;
		tempY = 0;		
		tempZ = 0;
		calcnt = 0;								//��λ�������Ա������ĵ���
		return 0;
	}
	else return 1;
}



/********************************************************************************
 * ��������MPUAccZeroCal_GravityMeasure()
 * ����  ��create a quaternion from Euler angles
 * ����  ��-		    	
 * ����  ��inttype
 * ����  ��-
 ********************************************************************************/
u8 MPUAccZeroCal_GravityMeasure()
{	static u16 calcnt = 0;
	static s32 tempX = 0,tempY = 0,tempZ = 0;
//	
	ACC_Raw.X = accel[0];						//������ԭʼ����
	ACC_Raw.Y = accel[1];
	ACC_Raw.Z = accel[2];
	
	if(ACC_Raw.X>1000||ACC_Raw.X<-1000			//pass��ǰ������������� 1000Ϊ���Ƶ���ƫ���Χ
	||ACC_Raw.Y>1000||ACC_Raw.Y<-1000
	)
	return 1;
	
	tempX += ACC_Raw.X;
	tempY += ACC_Raw.Y;
	tempZ += ACC_Raw.Z;							//һ�㲻����
	calcnt++;
	if(calcnt==100)								//��100����ƽ��
	{
		ACC_OFFSET.X = tempX/calcnt;
		ACC_OFFSET.Y = tempY/calcnt;
		ACC_1G		 = tempZ/calcnt;			//��ǰ����ֵ
		tempX = 0;
		tempY = 0;		
		tempZ = 0;
		calcnt = 0;								//��λ�������Ա������ĵ���
		return 0;
	}
	else return 1;
}



#define IMU_Kp 0.8f                         // ֵԽ�󣬶Լ��ٶȵĲο���Խ�� proportional gain governs rate of convergence to accelerometer/magnetometer
#define IMU_Ki 0.001f//0.0003f              // ֵ���˹��Ȼ��й��壬integral gain governs rate of convergence of gyroscope biases
#define IMU_halfT 0.002f//        // half the sample period???????

float exInt = 0, eyInt = 0, ezInt = 0;    	// scaled integral error
float exInt_mag = 0, eyInt_mag = 0, ezInt_mag = 0;    	// scaled integral error of mag
/********************************************************************************
 * ��������IMU_update()
 * ����  ��create a quaternion from Euler angles
 * ����  ��-		    	
 * ����  ��inttype
 * ����  ��-
 ********************************************************************************/

void IMU_update(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float  wx, wy, wz; 
	float ex, ey, ez;
	
	//���������Ԫ��������任��
	Quaternion_rotation_matrix();				
	
	norm = sqrt(ax*ax + ay*ay + az*az);      	//acc?????
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;								//��һ��ACC������

	// estimated direction of gravity            
	wx = g_MxCnb.a.Z;							//��Ϊ��������ϵz��������ƽ��					
	wy = g_MxCnb.b.Z;
	wz = g_MxCnb.c.Z;

	// error is sum of cross product(������) between reference direction of fields and direction measured by sensors
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
 * ��������AHRS_update()S.O.H. Madgwick
 * ����  ��update attitudequaternion from Euler angles
 * ����  ��-		    	
 * ����  ��inttype
 * ����  ��-
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
	az = az / norm;														//��һ��ACC������

	norm = sqrt(mx*mx + my*my + mz*mz);          
	mx = mx / norm;
	my = my / norm;
	mz = mz / norm;														//��һ��Mag������

	// estimated direction of gravity and flux (w and v)
	wx = 2*(q1q3 - q0q2);											//���������ڻ�������ϵ�е�ʸ�� ��Ϊ��Rϵ�У�����ʸ����0,0,g��,����
	wy = 2*(q0q1 + q2q3);
	wz = q0q0 - q1q1 - q2q2 + q3q3;

	//�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ��
	// compute reference direction of flux
	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2); 

	/*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
	*��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣�������by=0��bx=ĳֵ
	*������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
	*�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
	*�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
	*��Ϊby=0�����Ծͼ򻯳�(bx*bx)  = ((hx*hx) + (hy*hy))�������bx��
	*/
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz;

	//���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
	vx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	vy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	vz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

	// error is sum of cross product(������) between reference direction of fields and direction measured by sensors
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
//		mz = mz / norm;														//��һ��Mag������
//		
//		//�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ��
//		// compute reference direction of flux
//		hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
//		hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//		hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2); 

//		/*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
//		*��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣�������by=0��bx=ĳֵ
//		*������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
//		*�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
//		*�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
//		*��Ϊby=0�����Ծͼ򻯳�(bx*bx)  = ((hx*hx) + (hy*hy))�������bx��
//		*/
//		bx = sqrt((hx*hx) + (hy*hy));
//		bz = hz;

//		//���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
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
 * ��������AHRS_SpeedUpdate()
 * ����  ��ʵ��δ�������ٶȸ���,ע����δ���ٶȸ��½��л�����ת����
 * ����  ��-	    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/

void AHRS_SpeedUpdate()
{
	ACC_Linear_n.X = ACC_Last.X*g_MxCnb.a.X + ACC_Last.Y*g_MxCnb.b.X + ACC_Last.Z*g_MxCnb.c.X;
	ACC_Linear_n.Y = ACC_Last.X*g_MxCnb.a.Y + ACC_Last.Y*g_MxCnb.b.Y + ACC_Last.Z*g_MxCnb.c.Y;
	ACC_Linear_n.Z = ACC_Last.X*g_MxCnb.a.Z + ACC_Last.Y*g_MxCnb.b.Z + ACC_Last.Z*g_MxCnb.c.Z - ACC_1G;	//����任��Ҫ��ȥ�������ٶ�

	ACC_Linear_n_mm.Z = ACC_Linear_n.Z*Acc_G*1000;
//	g_LVelimu_n.X += ACC_Linear_n.X*Acc_G*T_AHRS_UPDATE;
//	g_LVelimu_n.Y += ACC_Linear_n.Y*Acc_G*T_AHRS_UPDATE;
//	g_LVelimu_n.Z += ACC_Linear_n.Z*Acc_G*T_AHRS_UPDATE;
}
/********************************************************************************
 * ��������AHRS_Attitude()
 * ����  �����ʲο�ϵͳ������IMUʵ��
 * ����  ��-	    	
 * ����  ��-
 * ����  ��-
 ********************************************************************************/

void AHRS_Attitude()
{
	u8 i;
	int tempval1 = 0,tempval2 = 0,tempval3 = 0;
//	float sin2x,sin2y,abs_Roll_Last,abs_Pitch_Last;
	static u8 filpoint = 0;			//�˲���ָ��

										//��DMP���ݴ���
//---������----------------------------------------------------------------------------------------------------
	GYRO_Raw.X = gyro[0]-GYRO_OFFSET.X;						//������ԭʼ����
	GYRO_Raw.Y = gyro[1]-GYRO_OFFSET.Y;
	GYRO_Raw.Z = gyro[2]-GYRO_OFFSET.Z;
	
	if(GYRO_Raw.X<(-5000)||GYRO_Raw.X>(5000)) GYRO_Raw.X = GYRO_Last.X; //�������������ʹ��ԭ��ֵ
	if(GYRO_Raw.Y<(-5000)||GYRO_Raw.Y>(5000)) GYRO_Raw.Y = GYRO_Last.Y; 
	if(GYRO_Raw.Z<(-5000)||GYRO_Raw.Z>(5000)) GYRO_Raw.Z = GYRO_Last.Z; 
	
	GYRO_Last.X = GYRO_Raw.X;
	GYRO_Last.Y = GYRO_Raw.Y;
	GYRO_Last.Z = GYRO_Raw.Z;	

//---���ٶ�----------------------------------------------------------------------------------------------------
	ACC_Raw.X = accel[0]-ACC_OFFSET.X;			//������ԭʼ����
	ACC_Raw.Y = accel[1]-ACC_OFFSET.Y;
	ACC_Raw.Z = accel[2];//-ACC_OFFSET.Z;			//

	FiltArray_AX[filpoint] = (int)ACC_Raw.X;	//ÿ�ɼ�һ�������������
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
	ACC_Last.Z = tempval3/FILTERZISE_ACC;			//�����ƽ��
	filpoint++;
	if(filpoint==FILTERZISE_ACC) filpoint=0;

//---����------------------------------------------------------------------------------------------------------------
	magtmp.hx = -hmc5883l.hx;									//���з��ź���������ת��
	magtmp.hy = -hmc5883l.hy;									//���贫�����ĵ�����ϵX,Y�պ����������ϵ�෴
	magtmp.hz = hmc5883l.hz;
	
//---��Ԫ��ˢ����̬----------------------------------------------------------------------------------------------	
	IMU_update(GYRO_Last.X*Gyro_Gr,GYRO_Last.Y*Gyro_Gr,GYRO_Last.Z*Gyro_Gr,
				ACC_Last.X,ACC_Last.Y,ACC_Last.Z);
	
//	AHRS_update(GYRO_Last.X*Gyro_Gr,GYRO_Last.Y*Gyro_Gr,GYRO_Last.Z*Gyro_Gr,
//						 ACC_Last.X,ACC_Last.Y,ACC_Last.Z,magtmp.hx,magtmp.hy,magtmp.hz);
	
	//---����������Ƕ�----------------------------------------------------------------------------------------------	
	/*
	// roll����Roll��Pitch��Щ����˵�������෴	�������壺ACC_X��ӦRoll 
	*/	
	Roll_Raw_Rad  = asin(-2 * Quat.Q1 * Quat.Q3 + 2 * Quat.Q0* Quat.Q2); 
	Pitch_Raw_Rad = atan2(2 * Quat.Q2 * Quat.Q3 + 2 * Quat.Q0 * Quat.Q1, -2 * Quat.Q1 * Quat.Q1 - 2 * Quat.Q2* Quat.Q2 + 1); // ptich
//	Yaw_Raw_Rad   = atan2(2 * Quat.Q1 * Quat.Q2 + 2 * Quat.Q0 * Quat.Q3, -2 * Quat.Q2*Quat.Q2 - 2 * Quat.Q3* Quat.Q3 + 1);

//--����������Ǻ���ֵ-------------------------------------------------------
	CosPitch = cos(Pitch_Raw_Rad);
	SinPitch = sin(Pitch_Raw_Rad);
	CosRoll = cos(Roll_Raw_Rad);
	SinRoll = sin(Roll_Raw_Rad);

	Yaw_Raw_Rad = Mag_Yaw_Cal(&Magdecp);							//��λ����
	
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
 * ��������Mag_Yaw_Cal()
 * ����  �����贫������ǲ������������ɴ����õĺ����
 * ����  �������������mag�ṹ���ַ	    	
 * ����  �����躽���
 * ����  ��-
 ********************************************************************************/
float Mag_Yaw_Cal(tg_HMC5883L_FLOAT *mag_crect_val)
{
	float sin_roll_mag,cos_roll_mag,cos_pitch_mag,sin_pitch_mag;
	
	sin_roll_mag = -SinRoll;			//�Ӹ�����ΪFCUת�ӵ�5883����ϵ��6050�ĸպ��෴
	cos_roll_mag = CosRoll;
	sin_pitch_mag= -SinPitch;
	cos_pitch_mag= CosPitch;
	
	//������̬ת�ƾ��󽫴���X,Y��У����ˮƽ�棬����5883��x����y������������������ϵ��
	//x�ᡢy���෴���������µ�ʽ��pitch��roll�����й�ʽ�еĶ����ǶԵ��ġ�
	mag_crect_val->hx = hmc5883l.hx*cos_roll_mag + hmc5883l.hz*sin_roll_mag;			//����֤	
	mag_crect_val->hy = hmc5883l.hx*sin_pitch_mag*sin_roll_mag + hmc5883l.hy*cos_pitch_mag - hmc5883l.hz*cos_roll_mag*sin_pitch_mag;
	
	HMC5883L_Cal(mag_crect_val);			//(&hmc5883l);
	return mag_crect_val->m_yaw;			//����
}

#define HP_Par 0.8f
#define LP_Par 0.2f
#define i_Par  0.03f
float errorh_int = 0;
/********************************************************************************
 * ��������AHRS_AltitudeVel()
 * ����  ��������ֱ�ٶ�
 * ����  ��    	
 * ����  ��
 * ����  ��-
 ********************************************************************************/
float AHRS_AltitudeVel(float imuacc_z,float sonarspd)
{
	static float altvel = 0;
	float tempval = Acc_G*1000*T_ALT_UPDATE;
	float errorh = g_LVel_n.Z - sonarspd;
	float z_rate_n;
	
	errorh_int += i_Par*errorh;						//��������������ٶ����
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
//	imuacc_z = imuacc_z/10;							//��֪��һ����λ�������⣬������Ҫ����10
//	errorhint += Alt_Ki*errorh;						//��������������ٶ����
//	imuacc_z -=( errorhint + Alt_Kp*errorh);
//	g_LVel_n.Z += imuacc_z*tempval;
//}

/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

