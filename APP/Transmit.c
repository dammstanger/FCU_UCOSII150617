/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：Transmit.c
 * 描	述	：通过串口上传或下载数据_发送命令端
 *                   
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.1.150418
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.12.24
 * 最后编辑	：2015.4.18
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/


/****************************包含头文件*******************************************/
#include "Transmit.h"
#include "Optical_Flow.h"
#include "Sonar.h"
/****************************宏定义***********************************************/

/****************************变量声明*********************************************/

/****************************变量定义*********************************************/
uint8_t Recvbuf[12] = {0};
uint8_t Sendbuf[12] = {0};

uint8_t Trans_RevPakFin = 0;
uint8_t Trans_RevPakErr = 0;
AP_TRANS_PAK *Pakbuf_Optf = &AP_Pakbuf;
SONAR_TRANS_PAK *Pakbuf_Sonar = &SR_Pakbuf;

/****************************函数声明*********************************************/

/********************************************************************************
 * 函数名：main()
 * 描述  ：主函数
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/
void PakRev_OverTimeDeal()
{
	if(Trans_RevPakFin!=0)
	{
		;
	}
		
}

/********************************************************************************
 * 函数名：main()
 * 描述  ：主函数
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/
u8 PakRev_BufHandle(u8 dat)
{
	static u8 check = 0,i= 2;
	u8 firstbyte_flag = 0;
	if(check==2)
	{
		Recvbuf[i] = dat;
		i++;
		if(i==PAG_REVSIZE)
		{
			i = 0;
			check = 0;
			Trans_RevPakFin = 1;
		}
	}
	if((dat==0x55)&&(check==0))		check = 1;
	if((dat==0xaa)&&(check==1)) 	{check = 2;i = 2;firstbyte_flag = 1;}
	return firstbyte_flag;
}

/********************************************************************************
 * 函数名：PakRev_DMAHandle()
 * 描述  ：专用于DMA的包处理
 * 输入  ：-		    	
 * 返回  ：inttype
 * 调用  ：-
 ********************************************************************************/
bool PakRev_DMAHandle()
{
	if(Recvbuf[0]==0x55&&Recvbuf[1]==0xAA)
	{
		Trans_RevPakFin = 1;
		return TRUE;
	}
	else
	{
		Trans_RevPakErr = 1;
		return FALSE;
	}
	
}

/********************************************************************************
 * 函数名：
 * 描述  ：
 * 输入  ：-		    	
 * 返回  ：- 
 * 调用  ：外部调用
 ********************************************************************************/
u8 Pak_Handle(void)
{
	
	if(Is_AP_RTDATA_Ack_Pkg())
	{	
		AP_RTDATA_Cmd_Handle();
		AP_RTDATA_Cmd_ACK();
		return 1;
	}
	else if(Is_RTDATA_Ack_Pkg())
	{
		RTDATA_Cmd_Handle();
		RTDATA_Cmd_ACK();
		return 1;
	}
	else if(Is_TEST_Ack_Pkg())
	{
		TEST_Cmd_Handle();
		TEST_Cmd_ACK();
		return 1;
	}
	else if(Is_AP_BRIGHT_Ack_Pkg())
	{
		AP_BRIGHT_Cmd_Handle();
		AP_BRIGHT_Cmd_ACK();
		return 1;
	}
	else return 0;
}


/******************* (C) COPYRIGHT 2015 DammStanger *****END OF FILE************/

