/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���		��HMC5883.c
 * ��	��		��STM32 HMC5883L ����
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150409
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾		��ST3.0.0
 * ����ʱ��	��2014.8.31
 * ���༭	��2015.4.9
 * ��			ע��
 **-------------------------------------------------------------------------------
  --------->y		HMC5883L ����ϵ
	|
  |
  |
/\/
  x 
 
 
 
     N 
 NW  |  NE
     |  
W----------E
     |
 SW  |  SE
     S
 
 * ��	��		��Damm Stanger
 * ��	��		��dammstanger@qq.com
**********************************************************************************************/


#include "HMC5883.h"
#include "SysTick.h"
#include "I2C_1.h"
#include <math.h>    	//Keil library 
#include "led.h"
#include	<ucos_ii.h>  		//uC/OS-IIϵͳ����ͷ�ļ�


tg_HMC5883L_TYPE hmc5883l;
tg_HMC5883L_FLOAT Magdecp;

tg_HMC5883L_TYPE HMCoffset = {-148,102,-115,0};						//���ƫ��
////��������
float HMC5883L_GAIN_X = 1.064534;
float HMC5883L_GAIN_Y = 1.096332;
float HMC5883L_GAIN_Z = 1.042416;


void HMC5883L_WriteByte(uint8_t REG_Address,uint8_t REG_data)
{
	OS_CPU_SR cpu_sr=0;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//�ٽ��� 
	IIC1_WriteByte(HMC5883_SlaveAddress,REG_Address,REG_data);
	OS_EXIT_CRITICAL();								//�˳��ٽ���	
#else
	IIC1_WriteByte(HMC5883_SlaveAddress,REG_Address,REG_data);
#endif	
}

//**************************************
//��5883��ȡһ���ֽ���
//**************************************
u8 HMC5883L_ReadByte(uint8_t REG_Address)
{	
	u8 readval = 0x55;
	OS_CPU_SR cpu_sr=0;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//�ٽ��� 
	readval = IIC1_ReadByte(HMC5883_SlaveAddress,REG_Address);
	OS_EXIT_CRITICAL();								//�˳��ٽ���	
#else
	readval = IIC1_ReadByte(HMC5883_SlaveAddress,REG_Address);
#endif	
	return readval;
}

void HMC5883L_ReadBytes(u8* pBuffer,u8 readAddr, u16 NumByteToRead)
{
	OS_CPU_SR cpu_sr=0;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//�ٽ��� 
	IIC1_MultRead(HMC5883_SlaveAddress,pBuffer,readAddr,NumByteToRead);
	OS_EXIT_CRITICAL();								//�˳��ٽ���	
#else
	IIC1_MultRead(HMC5883_SlaveAddress,pBuffer,readAddr,NumByteToRead);
#endif	
}

/******************************************************************************
/ ��������:��ʼ��HMC5883
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
u8  HMC5883L_Init(void)
{
	u8 Add[3];//����ʶ��
	HMC5883L_ReadBytes(Add,HMC5883L_IRA,3);	
	printf("HMC5883_ID=%x\r\n",Add[0]);
	printf("HMC5883_ID=%x\r\n",Add[1]);
	printf("HMC5883_ID=%x\r\n",Add[2]);
	if(Add[0]==0x48&&Add[1]==0x34&&Add[2]==0x33)
	{
		HMC5883L_WriteByte(HMC5883L_REGA,0x78);		//ÿ�����8��ƽ���������� 0x14->30Hz  0x18->75Hz(continue mode max) ��������
		HMC5883L_WriteByte(HMC5883L_REGB,0x20);		//�������� Ĭ��0x20	1090/Ga
		HMC5883L_WriteByte(HMC5883L_MODE,0x00);		//��������ģʽ
		return TRUE;
	}
	else return FALSE;
}

/******************************************************************************
/ ��������:��ȡHMC5883������
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void HMC5883L_Read(tg_HMC5883L_TYPE * ptResult)
{
    uint8_t tmp[6];
    int32_t s32Val;
    
    HMC5883L_WriteByte(HMC5883L_REGA,0x14);   //30Hz
    HMC5883L_WriteByte(HMC5883L_MODE,0x01);   //���β���ģʽ
    Delay_ms(10);
    
    tmp[0]=HMC5883L_ReadByte(HMC5883L_HX_H);//OUT_X_L_A
    tmp[1]=HMC5883L_ReadByte(HMC5883L_HX_L);//OUT_X_H_A
    
    tmp[2]=HMC5883L_ReadByte(HMC5883L_HZ_H);//OUT_Z_L_A
    tmp[3]=HMC5883L_ReadByte(HMC5883L_HZ_L);//OUT_Z_H_A
    
    tmp[4]=HMC5883L_ReadByte(HMC5883L_HY_H);//OUT_Y_L_A
    tmp[5]=HMC5883L_ReadByte(HMC5883L_HY_L);//OUT_Y_H_A

    ptResult->hx  = (int16_t)((tmp[0] << 8) | tmp[1])+HMCoffset.hx;
    s32Val = (int16_t)((tmp[4] << 8) | tmp[5])+HMCoffset.hy;    
    s32Val = (s32Val*HMC5883L_GAIN_Y)/10000;
    ptResult->hy    = (int16_t)s32Val;
    ptResult->hz    = (int16_t)((tmp[2] << 8) | tmp[3]);
}

/******************************************************************************
/ ��������:����HMC5883��ʼת��(����������-�ж�-��ȡ���ݵĳ���)
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (����)
******************************************************************************/
void HMC5883L_Start(void)
{
   HMC5883L_WriteByte(HMC5883L_REGA,0x14);   //30Hz
   HMC5883L_WriteByte(HMC5883L_MODE,0x00);   //��������ģʽ
}

/******************************************************************************
/ ��������:��ȡHMC5883������(����������-�ж�-��ȡ���ݵĳ���)
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (��ѯ)
******************************************************************************/
void HMC5883L_MultRead(tg_HMC5883L_TYPE * ptResult)
{
	OS_CPU_SR cpu_sr=0;
  uint8_t tmp[6];
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//�ٽ��� 
    IIC1_MultRead(HMC5883_SlaveAddress,tmp,HMC5883L_HX_H,6);   //�����������������  
	OS_EXIT_CRITICAL();								//�˳��ٽ���	
#else
    IIC1_MultRead(HMC5883_SlaveAddress,tmp,HMC5883L_HX_H,6);   //�����������������  
#endif
	
	//��������(����x������y�����)	 ΪʲôMWC���ȳ��������ټ��أ�����������֤�������ǶԵ�
	ptResult->hx  = ((int16_t)((tmp[0] << 8) | tmp[1])+HMCoffset.hx)*HMC5883L_GAIN_X;
	ptResult->hy  = ((int16_t)((tmp[4] << 8) | tmp[5])+HMCoffset.hy)*HMC5883L_GAIN_Y;    
	ptResult->hz  = ((int16_t)((tmp[2] << 8) | tmp[3])+HMCoffset.hz)*HMC5883L_GAIN_Z;    

} 

/******************************************************************************
/ ��������:HMC5883������У׼
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��: // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
			// The new gain setting is effective from the second measurement and on.
******************************************************************************/
void HMC5883L_GainCalibrate()
{	uint8_t tmp[6],i;
//	int32_t s32Val;
	s32 total_X=0,total_Y=0,total_Z=0;						//4�ֽڲ��������
	s16 hx=0,hy=0,hz=0;
	Delay_ms(50);
	//--------------------------------------------------------------------------	
	HMC5883L_WriteByte(HMC5883L_REGA,0x60+HMC_POS_BIAS);   	//8��ƽ���������,������ƫ
															//����Ĭ��1090/Ga
	HMC5883L_WriteByte(HMC5883L_MODE,0x01);   				//��һ����ģʽ
	Delay_ms(10);
	//----set positive bias--------------------------------------------------------------
	
	for(i=0;i<10;i++)
	{
		HMC5883L_WriteByte(HMC5883L_MODE,1);
		Delay_ms(10);
		tmp[0]=HMC5883L_ReadByte(HMC5883L_HX_H);//OUT_X_L_A
		tmp[1]=HMC5883L_ReadByte(HMC5883L_HX_L);//OUT_X_H_A

		tmp[2]=HMC5883L_ReadByte(HMC5883L_HZ_H);//OUT_Z_L_A
		tmp[3]=HMC5883L_ReadByte(HMC5883L_HZ_L);//OUT_Z_H_A

		tmp[4]=HMC5883L_ReadByte(HMC5883L_HY_H);//OUT_Y_L_A
		tmp[5]=HMC5883L_ReadByte(HMC5883L_HY_L);//OUT_Y_H_A
		
		hx = (s16)((tmp[0] << 8) | tmp[1]);
		hy = (s16)((tmp[2] << 8) | tmp[3]);
		hz = (s16)((tmp[4] << 8) | tmp[5]);
		printf("HMC5883L:\thx: %4d,\thy: %4d,\thz: %4d\n\r",
		hx, hy, hz);		
		
		total_X += (s32)((tmp[0] << 8) | tmp[1]);
		total_Z += (s32)((tmp[2] << 8) | tmp[3]);
		total_Y += (s32)((tmp[4] << 8) | tmp[5]);
		printf("HMC5883L:\tT1x: %5d,\tT1y: %5d,\tT1z: %5d\n\r",
            total_X, total_Y, total_Z);
	}
	//----set negative bias--------------------------------------------------------------
	HMC5883L_WriteByte(HMC5883L_REGA,0x10+HMC_NEG_BIAS);   	//������ƫ
	for(i=0;i<10;i++)
	{
		HMC5883L_WriteByte(HMC5883L_MODE,1);
		Delay_ms(10);
		tmp[0]=HMC5883L_ReadByte(HMC5883L_HX_H);//OUT_X_L_A
		tmp[1]=HMC5883L_ReadByte(HMC5883L_HX_L);//OUT_X_H_A

		tmp[2]=HMC5883L_ReadByte(HMC5883L_HZ_H);//OUT_Z_L_A
		tmp[3]=HMC5883L_ReadByte(HMC5883L_HZ_L);//OUT_Z_H_A

		tmp[4]=HMC5883L_ReadByte(HMC5883L_HY_H);//OUT_Y_L_A
		tmp[5]=HMC5883L_ReadByte(HMC5883L_HY_L);//OUT_Y_H_A
		
		hx = (s16)((tmp[0] << 8) | tmp[1]);		//��תΪ16λ�ı�����Ȼֱ���ۼӻ����
		hy = (s16)((tmp[2] << 8) | tmp[3]);
		hz = (s16)((tmp[4] << 8) | tmp[5]);
		printf("HMC5883L:\thx: %4d,\thy: %4d,\thz: %4d\n\r",
		hx, hy, hz);
		
		total_X -= hx;						//
		total_Z -= hy;
		total_Y -= hz;
		printf("HMC5883L:\tT2x: %5d,\tT2y: %5d,\tT2z: %5d\n\r",
		total_X, total_Y, total_Z);
	}
	
	HMC5883L_GAIN_X = fabs(1090*HMC58X3L_X_SELF_TEST_GAUSS*2*10/total_X);
	HMC5883L_GAIN_Y = fabs(1090*HMC58X3L_Y_SELF_TEST_GAUSS*2*10/total_Y);
	HMC5883L_GAIN_Z = fabs(1090*HMC58X3L_Z_SELF_TEST_GAUSS*2*10/total_Z);
    printf("HMC5883L:\tGAIN_X: %4f,\tGAIN_Y: %4f,\tGAIN_Z: %4f\n\r",HMC5883L_GAIN_X,HMC5883L_GAIN_Y,HMC5883L_GAIN_Z);

}

/******************************************************************************
/ ��������:HMC5883��ƫУ׼
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��: // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
			// The new gain setting is effective from the second measurement and on.
******************************************************************************/
void HMC5883L_BiasCalibrate()
{
	uint8_t tmp[6];
	u16 i;
	int16_t x0,y0,z0;
	tg_HMC5883L_TYPE HMCmax = {-32768,-32768,-32768,0};			//������ֵ
	tg_HMC5883L_TYPE HMCmin = {32767,32767,32767,0};			//�����Сֵ

	for(i=0;i<1000;i++)								//��ʱ��Լ5s ��5s�����ˮƽ��ת
	{
		HMC5883L_WriteByte(HMC5883L_REGA,0x68);   	//ƽ����8 ,��������
		HMC5883L_WriteByte(HMC5883L_MODE,0x01);   	//��һ����ģʽ
		Delay_ms(10);
		tmp[0]=HMC5883L_ReadByte(HMC5883L_HX_H);//OUT_X_L_A
		tmp[1]=HMC5883L_ReadByte(HMC5883L_HX_L);//OUT_X_H_A

		tmp[2]=HMC5883L_ReadByte(HMC5883L_HZ_H);//OUT_Z_L_A
		tmp[3]=HMC5883L_ReadByte(HMC5883L_HZ_L);//OUT_Z_H_A

		tmp[4]=HMC5883L_ReadByte(HMC5883L_HY_H);//OUT_Y_L_A
		tmp[5]=HMC5883L_ReadByte(HMC5883L_HY_L);//OUT_Y_H_A
		
		x0 = (int16_t)((tmp[0] << 8) | tmp[1]);
		y0 = (int16_t)((tmp[4] << 8) | tmp[5]);    
		z0 = (int16_t)((tmp[2] << 8) | tmp[3]);	
		if(x0>HMCmax.hx) HMCmax.hx = x0;
		if(y0>HMCmax.hy) HMCmax.hy = y0;
		if(z0>HMCmax.hz) HMCmax.hz = z0;
		if(x0<HMCmin.hx) HMCmin.hx = x0;
		if(y0<HMCmin.hy) HMCmin.hy = y0;
		if(z0<HMCmin.hz) HMCmin.hz = z0;
	}
	HMCoffset.hx = -((HMCmax.hx+HMCmin.hx)>>1);
	HMCoffset.hy = -((HMCmax.hy+HMCmin.hy)>>1);
	HMCoffset.hz = -((HMCmax.hz+HMCmin.hz)>>1);
}

/******************************************************************************
/ ��������:˫��򵥼���
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void HMC5883L_Cal(tg_HMC5883L_FLOAT * ptResult)
{
    float x,y;
    float angle;
    
    x = ptResult->hx;
    y = ptResult->hy;
    //�������

	if(x>0x7fff)x-=0xffff;
    if(y>0x7fff)y-=0xffff;
    //LED1_ON();
//    angle= atan2(y,x) * (180 / 3.14159265) + 180;   //160us����ʱ��
    angle= atan2(y,x) + 3.141593;   	//����
    ptResult->m_yaw = (float)angle;     // �õ�����
}

