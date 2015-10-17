/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��ComProtocol.c
 * ��	��	���ⲿ�жϳ�ʼ�����жϷ���
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.1506.3
 * ������ϵ	��FCU_UCOSII150603_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.4.11
 * ���༭	��2015.6.10
 **-------------------------------------------------------------------------------

**********************************************************************************************/



/****************************����ͷ�ļ�*******************************************/
#include "ComProtocol.h"
#include "Control.h"
#include "DataProcess.h"


uint8_t BUF_R[12] = {0};
uint8_t BUF_T[10] = {0};
uint8_t RevFinish = 0;
__fly_reg FlyReg;
__fly_reg *Reg = &FlyReg;


unsigned char RC_Handle(void)
{
	NRF24L01_RxPacket(BUF_R);
	
	if (IsFlyAskPkg())
	{
		FlyAskPkgHandle();
//		FlyAckPkgSend();
		return 1;
	}
	else if (IsImuAskPkg())
	{
		ImuAskPkgHandle();
//		ImuAckPkgSend();
		return 1;
	}
	else if (IsMotoAskPkg())
	{
		MotoAskPkgHandle();
//		MotoAckPkgSend();
		return 1;
	}
	else if (IsPIDParAskPkg())
	{
		PIDParAskPkgHandle();
		return 1;
	}
	else if (IsRegWrAskPkg())
	{
		if (RegWrAskPkgIsWrite())
		{
			RegWritePkgHandle();
		}
		else if (RegWrAskPkgIsRead())
		{
			RegReadPkgHandle();
		}
		else
		{
			return 0;
		}
//		RegWrAckPkgSend();				//���ж��з���Ӧ���ռ�ý϶��ʱ�䣬��ʱ����
		return 1;
	}
	else
	{
		return 0;
	}
}




void SendDebugDat_Hunter_2401(int dat1,int dat2,int dat3,int dat4)//����ƥ�䴮�����˵�����
{
	s16 i;
	i=(s16)dat1;
	
	BUF_T[0] = 0x55;
	BUF_T[1] = (u8)(i>>8);
	BUF_T[2] = (u8)(i&0xff);
	
	i=(s16)dat2;
	BUF_T[3] = (u8)(i>>8);
	BUF_T[4] = (u8)(i&0xff);
	
	i=(s16)dat3;
	BUF_T[5] = (u8)(i>>8);
	BUF_T[6] = (u8)(i&0xff);
	
	i=(s16)dat4;
	BUF_T[7] = (u8)(i>>8);
	BUF_T[8] = (u8)(i&0xff);
	BUF_T[9] = 0xaa;
	
	NRF24L01_Tx();					
	NRF24L01_TxPacket(BUF_T,10);
	NRF24L01_Rx();					

		
}

void SendDebugDat_LabVIEW_2401(u8 datype,int dat1,int dat2,int dat3,int dat4)//����ƥ��labviwe������
{
	s16 i;
	i=(s16)dat1;
	BUF_T[0] = datype;
	BUF_T[1] = (u8)(i>>8);
	BUF_T[2] = (u8)(i&0xff);
	
	i=(s16)dat2;
	BUF_T[3] = (u8)(i>>8);
	BUF_T[4] = (u8)(i&0xff);
	
	i=(s16)dat3;
	BUF_T[5] = (u8)(i>>8);
	BUF_T[6] = (u8)(i&0xff);
	
	i=(s16)dat4;
	BUF_T[7] = (u8)(i>>8);
	BUF_T[8] = (u8)(i&0xff);
	BUF_T[9] = 0xaa;
	
	NRF24L01_Tx();					
	NRF24L01_TxPacket(BUF_T,10);
	NRF24L01_Rx();	
}

bool _24L01PIDParData()
{
	bool retval = FALSE;
	retval = PIDDebugData(&FlyReg);
	return retval;
}

u8 PIDDebugData(__fly_reg *debdat)
{	
	u8 retval = 0;
	if(FlightMode==UNARMED)
	{
		switch(debdat->PIDPar_CTLTYPE_REG)
		{
			case PID_CTLTYPE_RPA :{
				PID_ROL.P = debdat->PIDPar_P_REG/1000.0;				//P�����1000������Ŵ�һ��
				PID_PIT.P = PID_ROL.P;
				PID_ROL.I = debdat->PIDPar_I_REG/10000.0;
				PID_PIT.I = PID_ROL.I;
				PID_ROL.D = debdat->PIDPar_D_REG/10000.0;
				PID_PIT.D = PID_ROL.D;
				PID_ROL.Imax = debdat->PIDPar_IMAX_REG/10.0;
				PID_PIT.Imax = PID_ROL.Imax;
				retval = 0x22;
			}break;
			case PID_CTLTYPE_YAW :{
				PID_YAW.P = debdat->PIDPar_P_REG/1000.0;
				PID_YAW.I = debdat->PIDPar_I_REG/10000.0;
				PID_YAW.D = debdat->PIDPar_D_REG/10000.0;
				PID_YAW.Imax = debdat->PIDPar_IMAX_REG/10.0;
				retval = 0x44;
			}break;
			case PID_CTLTYPE_ALT :{
				PID_HIGH.P = debdat->PIDPar_P_REG/1000.0;
				PID_HIGH.I = debdat->PIDPar_I_REG/10000.0;
				PID_HIGH.D = debdat->PIDPar_D_REG/10000.0;
				PID_HIGH.Imax = debdat->PIDPar_IMAX_REG/10.0;	
				retval = 0x55;				
			}break;
			case PID_CTLTYPE_POS :{
				PID_POS_OF.P = debdat->PIDPar_P_REG/1000.0;
				PID_POS_OF.I = debdat->PIDPar_I_REG/10000.0;
				PID_POS_OF.D = debdat->PIDPar_D_REG/10000.0;
				PID_POS_OF.Imax = debdat->PIDPar_IMAX_REG/10.0;
				retval = 0x66;
			}break;
			case PID_CTLTYPE_RPSTB :{
				PID_STABLE.P = debdat->PIDPar_P_REG/1000.0;
				PID_STABLE.I = debdat->PIDPar_I_REG/10000.0;
				PID_STABLE.D = debdat->PIDPar_D_REG/10000.0;
				PID_STABLE.Imax = debdat->PIDPar_IMAX_REG/10.0;
				retval = 0x11;
			}break;
			case PID_CTLTYPE_ALTRATE :{
				P_RATE_ALT.P = debdat->PIDPar_P_REG/1000.0;
				P_RATEFILT_ALT.P = debdat->PIDPar_I_REG/1000.0;
				retval = 0x77;
			}break;
			case PID_CTLTYPE_ALTACC :{
				PID_ACC_ALT.P = debdat->PIDPar_P_REG/1000.0;
				PID_ACC_ALT.I = debdat->PIDPar_I_REG/10000.0;
				PID_ACC_ALT.D = debdat->PIDPar_D_REG/10000.0;
				PID_ACC_ALT.Imax = debdat->PIDPar_IMAX_REG/10.0;
				retval = 0x88;
			}break;
			default : break;	
		}
		return retval;
	}
	return retval;
}
