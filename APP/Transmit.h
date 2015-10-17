/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��Transmit.h
 * ��	��	��ͨ�������ϴ�����������_���������
 *                   
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150418
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.12.24
 * ���༭	��2015.4.18
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/

#ifndef __TRANSMIT_H_
#define __TRANSMIT_H_

/****************************����ͷ�ļ�*******************************************/
#include "stm32f10x.h"

#include "usart.h"

/****************************�궨��***********************************************/

#define TransPkgSend()			\
{								\
	Sendbuf[0] = 0x55;			\
	Sendbuf[1] = 0xaa;			\
	USART1_SendStr(Sendbuf,12);	\
}

#define PAG_SEDSIZE				12
#define PAG_REVSIZE				12

//----------��ͷƫ�Ƶ�ַ----------------
#define PAG_HEAD_ADDR_H			0x00
#define PAG_HEAD_ADDR_L			0X01
//----Э���׼ֵ
#define PAG_HEAD_H				0x55
#define PAG_HEAD_L				0xAA

#define Is_Valid_Cmd_Pkg()		((*(Recvbuf+PAG_HEAD_ADDR_H)==PAG_HEAD_H) && (*(Recvbuf+PAG_HEAD_ADDR_L)==PAG_HEAD_L))		


//----------ָ��ƫ�Ƶ�ַ----------------
#define PAG_CMD_ADDR_H			0x02
#define PAG_CMD_ADDR_L			0x03
//----ָ������
//
#define PAG_AP_RTDATA_CMD		0x1000
#define PAG_AP_RTDATA_CMD_H		0x10
#define PAG_AP_RTDATA_CMD_L		0x00

#define PAG_AP_RTDATA_ACK		0x1100
#define PAG_AP_RTDATA_ACK_H		0x11
#define PAG_AP_RTDATA_ACK_L		0x00
//
#define PAG_AP_BRIGHT_CMD		0x2000
#define PAG_AP_BRIGHT_CMD_H		0x20
#define PAG_AP_BRIGHT_CMD_L		0x00
//
#define PAG_AP_BRIGHT_ACK		0x2100
#define PAG_AP_BRIGHT_ACK_H		0x21
#define PAG_AP_BRIGHT_ACK_L		0x00

#define PAG_AP_TEST_CMD			0x3000
#define PAG_AP_TEST_CMD_H		0x30
#define PAG_AP_TEST_CMD_L		0x00
//
#define PAG_AP_TEST_ACK			0x3100
#define PAG_AP_TEST_ACK_H		0x31
#define PAG_AP_TEST_ACK_L		0x00

#define PAG_RTDATA_CMD			0x8000
#define PAG_RTDATA_CMD_H		0x80
#define PAG_RTDATA_CMD_L		0x00

#define PAG_RTDATA_ACK			0x8100
#define PAG_RTDATA_ACK_H		0x81
#define PAG_RTDATA_ACK_L		0x00


//FLASH��д
#define PAG_FLASH_WR_CMD		0x5000
#define PAG_FLASH_WR_CMD_H		0x50
#define PAG_FLASH_WR_CMD_L		0x00
//FLASH��д
#define PAG_FLASH_WR_ACK		0x5100
#define PAG_FLASH_WR_ACK_H		0x51
#define PAG_FLASH_WR_ACK_L		0x00


//----------ʵʱ����ƫ�Ƶ�ַ----------------
#define PAG_DX_ADDR				0X04
#define PAG_DY_ADDR				0x05
#define PAG_QUAL_ADDR			0x06
#define PAG_HIGHT_ADDR_H		0x07
#define PAG_HIGHT_ADDR_L		0x08
#define PAG_HIGHT_VEL_ADDR_H	0x09
#define PAG_HIGHT_VEL_ADDR_L	0x0a


#define Is_RTDATA_Cmd_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_RTDATA_CMD_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_RTDATA_CMD_L))		
#define Is_RTDATA_Ack_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_RTDATA_ACK_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_RTDATA_ACK_L))		

#define RTDATA_Cmd_Pkg_Send()						\
{													\
	*(Sendbuf+PAG_CMD_ADDR_H) = PAG_RTDATA_CMD_H;	\
	*(Sendbuf+PAG_CMD_ADDR_L) = PAG_RTDATA_CMD_L;	\
	TransPkgSend();									\
}

#define RTDATA_Cmd_Handle()											\
{																	\
	Pakbuf_Optf->DX_REG = *(Recvbuf+PAG_DX_ADDR);				\
	Pakbuf_Optf->DY_REG = *(Recvbuf+PAG_DY_ADDR);				\
	Pakbuf_Optf->QUAL_REG = *(Recvbuf+PAG_QUAL_ADDR);			\
	Pakbuf_Sonar->Hight_H = *(Recvbuf+PAG_HIGHT_ADDR_H);			\
	Pakbuf_Sonar->Hight_L = *(Recvbuf+PAG_HIGHT_ADDR_L);			\
	Pakbuf_Sonar->Hight_Vel_H = *(Recvbuf+PAG_HIGHT_VEL_ADDR_H);	\
	Pakbuf_Sonar->Hight_Vel_L = *(Recvbuf+PAG_HIGHT_VEL_ADDR_L);	\
}

#define RTDATA_Cmd_ACK()								\
{														\
}

//-----------------------------------------------------------------------------------------

//----------��������ƫ�Ƶ�ַ----------------
#define PAG_AP_DX_ADDR				0X04
#define PAG_AP_DY_ADDR				0x05
#define PAG_AP_QUAL_ADDR			0x06
#define PAG_AP_BRIGHT_ADDR_H		0x07		//����
#define PAG_AP_BRIGHT_ADDR_L		0x08
//#define PAG_AP_QUAL_ADDR			0x09
//#define PAG_AP_QUAL_ADDR			0x0a

#define AP_RTDATA_Cmd_Pkg_Send()						\
{														\
	*(Sendbuf+PAG_CMD_ADDR_H) = PAG_AP_RTDATA_CMD_H;	\
	*(Sendbuf+PAG_CMD_ADDR_L) = PAG_AP_RTDATA_CMD_L;	\
	TransPkgSend();											\
}


#define Is_AP_RTDATA_Cmd_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_AP_RTDATA_CMD_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_AP_RTDATA_CMD_L))		
#define Is_AP_RTDATA_Ack_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_AP_RTDATA_ACK_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_AP_RTDATA_ACK_L))		

#define AP_RTDATA_Cmd_Handle()							\
{														\
	Pakbuf_Optf->DX_REG = *(Recvbuf+PAG_AP_DX_ADDR);			\
	Pakbuf_Optf->DY_REG = *(Recvbuf+PAG_AP_DY_ADDR);			\
	Pakbuf_Optf->QUAL_REG = *(Recvbuf+PAG_AP_QUAL_ADDR);		\
}

#define AP_RTDATA_Cmd_ACK()								\
{														\
}

#define AP_BRIGHT_Cmd_Pkg_Send()						\
{														\
	*(Sendbuf+PAG_CMD_ADDR_H) = PAG_AP_BRIGHT_CMD_H;	\
	*(Sendbuf+PAG_CMD_ADDR_L) = PAG_AP_BRIGHT_CMD_L;	\
	TransPkgSend();											\
}

#define Is_AP_BRIGHT_Cmd_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_AP_BRIGHT_CMD_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_AP_BRIGHT_CMD_L))		
#define Is_AP_BRIGHT_Ack_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_AP_BRIGHT_ACK_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_AP_BRIGHT_ACK_L))		

#define AP_BRIGHT_Cmd_Handle()							\
{														\
}

#define AP_BRIGHT_Cmd_ACK()		\
{								\
}



//----------ʵʱ����ƫ�Ƶ�ַ----------------
#define PAG_TS_DX_ADDR				0X04
#define PAG_TS_DY_ADDR				0x05
#define PAG_TS_QUAL_ADDR			0x06
#define PAG_TS_ADDX_ADDR_H			0x07
#define PAG_TS_ADDX_ADDR_L			0x08
#define PAG_TS_ADDY_ADDR_H			0x09
#define PAG_TS_ADDY_ADDR_L			0x0a
//#define PAG_TS__					0x0b

#define AP_TEST_Cmd_Pkg_Send()						\
{													\
	*(Sendbuf+PAG_CMD_ADDR_H) = PAG_AP_TEST_CMD_H;	\
	*(Sendbuf+PAG_CMD_ADDR_L) = PAG_AP_TEST_CMD_L;	\
	TransPkgSend();									\
}


#define Is_TEST_Cmd_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_AP_TEST_CMD_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_AP_TEST_CMD_L))		
#define Is_TEST_Ack_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_AP_TEST_ACK_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_AP_TEST_ACK_L))		

#define TEST_Cmd_Handle()										\
{																\
	Pakbuf_Optf->DX_REG = *(Recvbuf+PAG_TS_DX_ADDR);			\
	Pakbuf_Optf->DY_REG = *(Recvbuf+PAG_TS_DY_ADDR);			\
	Pakbuf_Optf->QUAL_REG = *(Recvbuf+PAG_TS_QUAL_ADDR);		\
	Pakbuf_Optf->ADD_DX = ((s16)*(Recvbuf+PAG_TS_ADDX_ADDR_H)<<8)+*(Recvbuf+PAG_TS_ADDX_ADDR_L);		\
	Pakbuf_Optf->ADD_DY = ((s16)*(Recvbuf+PAG_TS_ADDY_ADDR_H)<<8)+*(Recvbuf+PAG_TS_ADDY_ADDR_L);		\
}

#define TEST_Cmd_ACK()												\
{																	\
}





//----------Flash��дƫ�Ƶ�ַ----------------
#define PAG_FLASH_W_R_ADDR			0x04
#define PAG_FLASH_REG_ADDR			0x05
#define PAG_FLASH_WR5_ADDR			0x06
#define PAG_FLASH_WR6_ADDR			0x07
#define PAG_FLASH_WR1_ADDR			0X08
#define PAG_FLASH_WR2_ADDR			0x09

#define Is_Flash_Cmd_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_FLASH_WR_CMD_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_FLASH_WR_CMD_L))		
#define Is_Flash_Ack_Pkg()		((*(Recvbuf+PAG_CMD_ADDR_H)==PAG_FLASH_WR_ACK_H) && (*(Recvbuf+PAG_CMD_ADDR_L)==PAG_FLASH_WR_ACK_L))		

#define FLASH_Cmd_Handle()								\
{														\
}

#define FLASH_Cmd_ACK()								\
{													\
}

/****************************�ṹ�嶨��*******************************************/

/****************************��������*********************************************/
extern uint8_t Sendbuf[12];
extern uint8_t Recvbuf[12];
extern uint8_t Trans_RevPakFin;
extern uint8_t Trans_RevPakErr;

/****************************��������*********************************************/
void PakRev_OverTimeDeal(void);
u8 PakRev_BufHandle(u8 dat);
u8 Pak_Handle(void);
bool PakRev_DMAHandle(void);


#endif
/******************* (C) COPYRIGHT 2014 DammStanger *****END OF FILE************/

