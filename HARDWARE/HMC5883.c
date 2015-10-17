/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名		：HMC5883.c
 * 描	述		：STM32 HMC5883L 驱动
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150409
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本		：ST3.0.0
 * 创建时间	：2014.8.31
 * 最后编辑	：2015.4.9
 * 备			注：
 **-------------------------------------------------------------------------------
  --------->y		HMC5883L 坐标系
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
 
 * 作	者		：Damm Stanger
 * 邮	箱		：dammstanger@qq.com
**********************************************************************************************/


#include "HMC5883.h"
#include "SysTick.h"
#include "I2C_1.h"
#include <math.h>    	//Keil library 
#include "led.h"
#include	<ucos_ii.h>  		//uC/OS-II系统函数头文件


tg_HMC5883L_TYPE hmc5883l;
tg_HMC5883L_FLOAT Magdecp;

tg_HMC5883L_TYPE HMCoffset = {-148,102,-115,0};						//存放偏移
////比例因子
float HMC5883L_GAIN_X = 1.064534;
float HMC5883L_GAIN_Y = 1.096332;
float HMC5883L_GAIN_Z = 1.042416;


void HMC5883L_WriteByte(uint8_t REG_Address,uint8_t REG_data)
{
	OS_CPU_SR cpu_sr=0;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//临界区 
	IIC1_WriteByte(HMC5883_SlaveAddress,REG_Address,REG_data);
	OS_EXIT_CRITICAL();								//退出临界区	
#else
	IIC1_WriteByte(HMC5883_SlaveAddress,REG_Address,REG_data);
#endif	
}

//**************************************
//从5883读取一个字节数
//**************************************
u8 HMC5883L_ReadByte(uint8_t REG_Address)
{	
	u8 readval = 0x55;
	OS_CPU_SR cpu_sr=0;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//临界区 
	readval = IIC1_ReadByte(HMC5883_SlaveAddress,REG_Address);
	OS_EXIT_CRITICAL();								//退出临界区	
#else
	readval = IIC1_ReadByte(HMC5883_SlaveAddress,REG_Address);
#endif	
	return readval;
}

void HMC5883L_ReadBytes(u8* pBuffer,u8 readAddr, u16 NumByteToRead)
{
	OS_CPU_SR cpu_sr=0;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//临界区 
	IIC1_MultRead(HMC5883_SlaveAddress,pBuffer,readAddr,NumByteToRead);
	OS_EXIT_CRITICAL();								//退出临界区	
#else
	IIC1_MultRead(HMC5883_SlaveAddress,pBuffer,readAddr,NumByteToRead);
#endif	
}

/******************************************************************************
/ 函数功能:初始化HMC5883
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
u8  HMC5883L_Init(void)
{
	u8 Add[3];//器件识别
	HMC5883L_ReadBytes(Add,HMC5883L_IRA,3);	
	printf("HMC5883_ID=%x\r\n",Add[0]);
	printf("HMC5883_ID=%x\r\n",Add[1]);
	printf("HMC5883_ID=%x\r\n",Add[2]);
	if(Add[0]==0x48&&Add[1]==0x34&&Add[2]==0x33)
	{
		HMC5883L_WriteByte(HMC5883L_REGA,0x78);		//每次输出8次平均测量周期 0x14->30Hz  0x18->75Hz(continue mode max) 正常测量
		HMC5883L_WriteByte(HMC5883L_REGB,0x20);		//增益设置 默认0x20	1090/Ga
		HMC5883L_WriteByte(HMC5883L_MODE,0x00);		//连续测量模式
		return TRUE;
	}
	else return FALSE;
}

/******************************************************************************
/ 函数功能:读取HMC5883的数据
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void HMC5883L_Read(tg_HMC5883L_TYPE * ptResult)
{
    uint8_t tmp[6];
    int32_t s32Val;
    
    HMC5883L_WriteByte(HMC5883L_REGA,0x14);   //30Hz
    HMC5883L_WriteByte(HMC5883L_MODE,0x01);   //单次测量模式
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
/ 函数功能:启动HMC5883开始转换(适用于启动-中断-读取数据的程序)
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:启动-中断-查询 (启动)
******************************************************************************/
void HMC5883L_Start(void)
{
   HMC5883L_WriteByte(HMC5883L_REGA,0x14);   //30Hz
   HMC5883L_WriteByte(HMC5883L_MODE,0x00);   //连续测量模式
}

/******************************************************************************
/ 函数功能:读取HMC5883的数据(适用于启动-中断-读取数据的程序)
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:启动-中断-查询 (查询)
******************************************************************************/
void HMC5883L_MultRead(tg_HMC5883L_TYPE * ptResult)
{
	OS_CPU_SR cpu_sr=0;
  uint8_t tmp[6];
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//临界区 
    IIC1_MultRead(HMC5883_SlaveAddress,tmp,HMC5883L_HX_H,6);   //多读读出传感器数据  
	OS_EXIT_CRITICAL();								//退出临界区	
#else
    IIC1_MultRead(HMC5883_SlaveAddress,tmp,HMC5883L_HX_H,6);   //多读读出传感器数据  
#endif
	
	//修正数据(根据x轴修正y轴输出)	 为什么MWC中先乘上增益再减呢？？？试验验证，以下是对的
	ptResult->hx  = ((int16_t)((tmp[0] << 8) | tmp[1])+HMCoffset.hx)*HMC5883L_GAIN_X;
	ptResult->hy  = ((int16_t)((tmp[4] << 8) | tmp[5])+HMCoffset.hy)*HMC5883L_GAIN_Y;    
	ptResult->hz  = ((int16_t)((tmp[2] << 8) | tmp[3])+HMCoffset.hz)*HMC5883L_GAIN_Z;    

} 

/******************************************************************************
/ 函数功能:HMC5883轴间比例校准
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明: // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
			// The new gain setting is effective from the second measurement and on.
******************************************************************************/
void HMC5883L_GainCalibrate()
{	uint8_t tmp[6],i;
//	int32_t s32Val;
	s32 total_X=0,total_Y=0,total_Z=0;						//4字节不会溢出了
	s16 hx=0,hy=0,hz=0;
	Delay_ms(50);
	//--------------------------------------------------------------------------	
	HMC5883L_WriteByte(HMC5883L_REGA,0x60+HMC_POS_BIAS);   	//8次平均采样输出,启动正偏
															//增益默认1090/Ga
	HMC5883L_WriteByte(HMC5883L_MODE,0x01);   				//单一测量模式
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
	HMC5883L_WriteByte(HMC5883L_REGA,0x10+HMC_NEG_BIAS);   	//启动负偏
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
		
		hx = (s16)((tmp[0] << 8) | tmp[1]);		//先转为16位的变量不然直接累加会出错
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
/ 函数功能:HMC5883零偏校准
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明: // Note that the  very first measurement after a gain change maintains the same gain as the previous setting. 
			// The new gain setting is effective from the second measurement and on.
******************************************************************************/
void HMC5883L_BiasCalibrate()
{
	uint8_t tmp[6];
	u16 i;
	int16_t x0,y0,z0;
	tg_HMC5883L_TYPE HMCmax = {-32768,-32768,-32768,0};			//存放最大值
	tg_HMC5883L_TYPE HMCmin = {32767,32767,32767,0};			//存放最小值

	for(i=0;i<1000;i++)								//耗时大约5s 在5s内完成水平旋转
	{
		HMC5883L_WriteByte(HMC5883L_REGA,0x68);   	//平均数8 ,正常测量
		HMC5883L_WriteByte(HMC5883L_MODE,0x01);   	//单一测量模式
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
/ 函数功能:双轴简单计算
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void HMC5883L_Cal(tg_HMC5883L_FLOAT * ptResult)
{
    float x,y;
    float angle;
    
    x = ptResult->hx;
    y = ptResult->hy;
    //求出区间

	if(x>0x7fff)x-=0xffff;
    if(y>0x7fff)y-=0xffff;
    //LED1_ON();
//    angle= atan2(y,x) * (180 / 3.14159265) + 180;   //160us计算时间
    angle= atan2(y,x) + 3.141593;   	//弧度
    ptResult->m_yaw = (float)angle;     // 得到方向
}

