/******************** (C) COPYRIGHT 2014 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Sonar.c
 * ��	��	����SR04��õľ�����д���
 *                    
 * ʵ��ƽ̨	��FCUV1.0
 * Ӳ������	��
 * �� 	��	��V1.1
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.25
 * ���༭	��2015.6.13
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************/

/****************************����ͷ�ļ�*******************************************/
#include "Sonar.h"
#include "SR04.h"
#include "SysTick.h"

/****************************�궨��***********************************************/
#define FILTERZISE_HIGH	10

/****************************��������*********************************************/

/****************************��������*********************************************/
SONAR_TRANS_PAK SR_Pakbuf;

u16  FiltArray_H[FILTERZISE_HIGH] ={0};

u16 SonarHight_Raw=0,SonarHight_avr,SonarHight_Last=0;
u16 SonarHight = 0;						//������̽��ĸ߶�ֵ
s16 SonarHightvel = 0;					//�߶ȱ仯���з���
s16 SonarHightvel_add = 0;




#if	SONAR_ONBOARD
/******************************************************************************
/ ��������:�߶�����kalman�˲�
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
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
/ ��������:�������ݴ���
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void SonarDataProcess()					//T=60ms
{
	u8 i;
	u16 temp_h2=0;//,SonarHight_avr;//,
	static u8 filpoint_h = 0,cnt = 0,Nospeedcnt = 0,contincnt = 0;			//�˲���ָ��
	
	SonarHight_Raw = SR04_HightCal();
	//--------------------------�򵥵�ȥ����ֵ��------------------------------------	
	FiltArray_H[filpoint_h] = SonarHight_Raw;	//ÿ�ɼ�һ�������������
	for(i=0;i<FILTERZISE_HIGH;i++)
	{
		temp_h2 += FiltArray_H[i];
	}
	SonarHight_avr = temp_h2/FILTERZISE_HIGH;			//�����ƽ��
	
	filpoint_h++;	
	if(filpoint_h==FILTERZISE_HIGH) filpoint_h =0;
	
	if(SonarHight_Raw<SonarHight_avr>>1) SonarHight_Raw = SonarHight_Last;//������ֵС֮ǰƽ��ֵ��һ���������һ��
	else if(SonarHight_Raw>(SonarHight_avr>>1)*3) SonarHight_Raw = SonarHight_Last;//������
	
	//--------------------------------------------------------------	
	SonarHight = KalmanFilter(SonarHight_Raw);
	SonarHightvel = (SonarHight - SonarHight_Last)*10;				//����10��Ϊmm/s 
	SonarHight_Last = SonarHight;
}





#else 

/********************************
������Sonar_DataUpdate
���ܣ��������Ŵ���������
���룺-
�����-
�޸ģ�20141225
***********************************/
void Sonar_DataUpdate()
{
	SonarHight = (u16)(SR_Pakbuf.Hight_H<<8|SR_Pakbuf.Hight_L);
	SonarHightvel = (s16)(SR_Pakbuf.Hight_Vel_H<<8|SR_Pakbuf.Hight_Vel_L)*10;			//����10��Ϊmm/s 
}


#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

