/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Sonar.c
 * 描	述	：对SR04测得的距离进行处理
 *                    
 * 实验平台	：FCUV1.0
 * 硬件连接	：
 * 版 	本	：V1.1
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.25
 * 最后编辑	：2015.6.13
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************/

/****************************包含头文件*******************************************/
#include "Sonar.h"
#include "SR04.h"
#include "SysTick.h"

/****************************宏定义***********************************************/
#define FILTERZISE_HIGH	10

/****************************变量声明*********************************************/

/****************************变量定义*********************************************/
SONAR_TRANS_PAK SR_Pakbuf;

u16  FiltArray_H[FILTERZISE_HIGH] ={0};

u16 SonarHight_Raw=0,SonarHight_avr,SonarHight_Last=0;
u16 SonarHight = 0;						//声呐所探测的高度值
s16 SonarHightvel = 0;					//高度变化率有符号
s16 SonarHightvel_add = 0;




#if	SONAR_ONBOARD
/******************************************************************************
/ 函数功能:高度数据kalman滤波
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
static float Q_val = 0.0022;//0.0002
static float R_val = 0.04;//0.469;
static float x_lval = 3;
static float p_lval = 1;
u16 KalmanFilter(u16 x_Measure)
{
	float x_tempval,p_tempval,Kg,x_nval,p_nval;
	
	x_tempval = x_lval;
	p_tempval = p_lval+Q_val;
	Kg = p_tempval/(p_tempval+R_val);

	x_nval = x_tempval+Kg*(x_Measure-x_tempval);
	p_nval = (1-Kg)*p_tempval;
	
	x_lval = x_nval;
	p_lval = p_nval;
	return (u16)x_lval;
}


/******************************************************************************
/ 函数功能:声呐数据处理
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void SonarDataProcess()					//T=60ms
{
	u8 i;
	u16 temp_h2=0;//,SonarHight_avr;//,
	static u8 filpoint_h = 0,cnt = 0,Nospeedcnt = 0,contincnt = 0;			//滤波器指针
	
	SonarHight_Raw = SR04_HightCal();
	//--------------------------简单的去掉极值点------------------------------------	
	FiltArray_H[filpoint_h] = SonarHight_Raw;	//每采集一个，放入队列中
	for(i=0;i<FILTERZISE_HIGH;i++)
	{
		temp_h2 += FiltArray_H[i];
	}
	SonarHight_avr = temp_h2/FILTERZISE_HIGH;			//求和求平均
	
	filpoint_h++;	
	if(filpoint_h==FILTERZISE_HIGH) filpoint_h =0;
	
	if(SonarHight_Raw<SonarHight_avr>>1) SonarHight_Raw = SonarHight_Last;//若测量值小之前平均值的一半则等于上一次
	else if(SonarHight_Raw>(SonarHight_avr>>1)*3) SonarHight_Raw = SonarHight_Last;//等于上
	
	//--------------------------------------------------------------	
	SonarHight = KalmanFilter(SonarHight_Raw);
	SonarHightvel = (SonarHight - SonarHight_Last)*10;				//乘以10变为mm/s 
	SonarHight_Last = SonarHight;
}





#else 

/********************************
函数：Sonar_DataUpdate
功能：更新声呐传感器数据
输入：-
输出：-
修改：20141225
***********************************/
void Sonar_DataUpdate()
{
	SonarHight = (u16)(SR_Pakbuf.Hight_H<<8|SR_Pakbuf.Hight_L);
	SonarHightvel = (s16)(SR_Pakbuf.Hight_Vel_H<<8|SR_Pakbuf.Hight_Vel_L)*10;			//乘以10变为mm/s 
}


#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

