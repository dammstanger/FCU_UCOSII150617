/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Optical_Flow.c
 * ��	��	�������������
 *                   
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150508
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2015.1.2
 * ���༭	��2015.5.8
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#include "ADNS3080.h"
#include "Optical_Flow.h"
#include "usart.h"
#include <math.h>
#include "RCdata.h"


AP_TRANS_PAK AP_Pakbuf;

OPTICALFLOW_PAR Optflow;

float	exp_change_x, exp_change_y;    // expected change in x, y coordinates
s8		dx,dy;
int16_t dx_ref = 0,dy_ref = 0;
u8 		g_intercnt=0;
/********************************
������OPFLOW_Para_Init
���ܣ�����������������ʼ��
���룺-
�����-
�޸ģ�20141228
***********************************/
void OPFLOW_Para_Init()
{
	Optflow.num_pixels = ADNS3080_PIXELS_X;
	Optflow.scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
	Optflow.field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;

	// multiply this number by altitude and pixel change to get horizontal
	// move (in same units as altitude)
	//    conv_factor = ((1.0f / (float)(num_pixels * scaler))
	//                   * 2.0f * tanf(field_of_view / 2.0f));	
	// ��������0.00615
	//������ͳ�Ƶõ���
//	Optflow.conv_factor = 0.0146;
//	Optflow.conv_factor = 0.02376;				//ʵ�����ݣ�400cpi �ƶ�100CM��Ӧ���߶�91.5cm����Ӧ46����λ
//	Optflow.conv_factor = 0.00594;				//1600cpi  0.02376/4
	Optflow.conv_factor = 0.00486;				//ʵ������ 1600cpi �ƶ�20CM��Ӧ���߶�71cm����Ӧ53-58����λ 0.00531-0.00486
	
    Optflow.radians_to_pixels = (Optflow.num_pixels * Optflow.scaler) / Optflow.field_of_view;
	// ��������162.99
	
	Optflow.change_x=0;
	Optflow.change_y=0;            	// actual change in x, y coordinates
	Optflow.dx_cm=0;
	Optflow.dy_cm=0;                    // x,y position in cm
	Optflow.x_add_cm=0;
	Optflow.y_add_cm=0;
#if POSRATECTL
	Optflow.Kp = 2.3;				//3.5;//0.05;
	Optflow.Kd = 0;					//1.0;
	Optflow.Ki = 0.072;
#else	
	Optflow.Kp = 0.09;				//0.090
	Optflow.Kd = 2.5;				//2.5
	Optflow.Ki = 0.001;				//0.00015
#endif	
	Optflow.irange_min = 10;			//���������10
	Optflow.irange = 20;				//��Χ20
	Optflow.irange_max = (Optflow.irange_min+Optflow.irange)/2.0;
	Optflow.f_ei = (Optflow.irange_min+Optflow.irange)/Optflow.irange;		//���ȼ���ϵ����һ����
}

void OPFLOW_InstalOrientation(u8 orientation,s8 x,s8 y)
{
	switch(orientation)
	{
		case INSTALORIEN_0_DEGREE :{
			dx=x;
			dy=y;
		}break;
		case INSTALORIEN_90_DEGREE :{
			dx=y;
			dy=-x;
		}break;
		case INSTALORIEN_180_DEGREE :{
			dx=-x;
			dy=-y;
		}break;
		case INSTALORIEN_270_DEGREE :{
			dx=-y;
			dy=x;
		}break;
	}
}
	
#if OPFLOW_SENSOR_OFFBOARD_USART_EN
/********************************
������OPFLOW_DataUpdate
���ܣ����¹�������������
���룺-
�����-
�޸ģ�20141224
***********************************/
void OPFLOW_DataUpdate()
{
	//�ڲɼ�֮����ת��Ϊ���������ϵһ��
	OPFLOW_InstalOrientation(INSTALORIEN_90_DEGREE,AP_Pakbuf.DX_REG,AP_Pakbuf.DY_REG);
	surface_quality = AP_Pakbuf.QUAL_REG;
}

#endif


/********************************
������Update_Position
���ܣ�λ�ø���
���룺roll_rad,pitch_rad�����ȣ�,sin_yaw,cos_yaw,altitude(����)
�����-
λ�������ʼλ�õ���Ծ��롣
�޸ģ�20141115
***********************************/
void Update_Position(float roll_rad, float pitch_rad,float sin_yaw, float cos_yaw, s16 altitude)
{
	static float _last_roll_rad;
	static float _last_pitch_rad;
	static s16 _last_altitude;
	
    float diff_roll_rad     = (roll_rad  - _last_roll_rad);
    float diff_pitch_rad    = (pitch_rad - _last_pitch_rad);
	float avg_altitude  = (altitude + _last_altitude)>>1;//*0.5f;
	
    // only update position if surface quality is good and angle is not
    // over 45 degrees
    if( surface_quality >= 10 && fabsf(roll_rad) <= OPTICALFLOW_ANGLE_LIMIT_MAX
     && fabsf(pitch_rad) <= OPTICALFLOW_ANGLE_LIMIT_MAX ) {
//	altitude = max(altitude, 0);
		 
	// calculate expected x,y diff due to roll and pitch change
	exp_change_x = diff_roll_rad * Optflow.radians_to_pixels;
	exp_change_y = -diff_pitch_rad * Optflow.radians_to_pixels;			//�и���----

	// real estimated raw change from mouse
	Optflow.change_x = dx + exp_change_x;
	Optflow.change_y = dy + exp_change_y;


//	��Ӱ��λ��ת��Ϊʵ��λ�Ƶ�λcm
	Optflow.dx_cm = Optflow.change_x * avg_altitude * Optflow.conv_factor;

	Optflow.dy_cm = Optflow.change_y * avg_altitude * Optflow.conv_factor;
		 
	// convert x/y movements into lon/lat movement
//	vlon = Optflow.dx_cm * cos_yaw + Optflow.y_cm * sin_yaw;
//	vlat = Optflow.dy_cm * cos_yaw - Optflow.x_cm * sin_yaw;

	dx_ref = Optflow.dx_cm;
	dy_ref = Optflow.dy_cm;
	
	Optflow.x_add_cm += Optflow.dx_cm;
	Optflow.y_add_cm += Optflow.dy_cm;
    }
	else {
		Optflow.dx_cm = 0;
		Optflow.dy_cm = 0;
		if(Rc_Data.CH6>=1800)
			g_intercnt++;
	}

    _last_altitude = altitude;
    _last_roll_rad = roll_rad;
    _last_pitch_rad = pitch_rad;
}













