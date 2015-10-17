/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------文件信息----------------------------------------------------------
 * 文件名	：MPU.c
 * 描	述	：MPU6050的操作
 *                    
 * 实验平台	：FCU V1.0
 * 硬件连接	：
 * 版 	本	：V1.0.150407
 * 从属关系	：FCU_UCOSII150602_UCOSII2.91
 * 库版本	：ST3.0.0
 * 创建时间	：2014.3.29
 * 最后编辑	：2015.4.7
 **-------------------------------------------------------------------------------

 * 作	者	：Damm Stanger
 * 邮	箱	：dammstanger@qq.com
**********************************************************************************************/
#include "SysTick.h"
#include <ucos_ii.h>  		//uC/OS-II系统函数头文件
#include "MPU.h"
#include "usart.h"
#include "led.h"
#ifdef MPU6050_DMP_DRIVER
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#endif

short gyro[3], accel[3], tempt,sensors;

#ifdef	MPU6050_DMP_DRIVER

unsigned long sensor_timestamp;
unsigned char more;
long quat[4];
 
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};



/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static  unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
//    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
	if(result == 0) printf("self-test is executed succesfully ......\n");	//added by Damm Stanger
	if (result == 0x7) 
//	if(result == 0x03)	//Curently we don't have compass sensor so we don't need to test it Damm Stanger
		{
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\n");
    }
	else
	{
		printf("bias has not been modified ......\n");
	}

}





int MPU_DMP_Init()
{ 	int result = 0;
	//mpu_init();
	  printf("mpu initialization complete......\n ");
	  //mpu_set_sensor
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  {
	  	 printf("mpu_set_sensor complete ......\n");
	  }
	  else
	  {
	  	 printf("mpu_set_sensor come across error ......\n");
		 result |=0x0001;
	  }
	  //mpu_configure_fifo
	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  {
	  	 printf("mpu_configure_fifo complete ......\n");
	  }
	  else
	  {
	  	 printf("mpu_configure_fifo come across error ......\n");
		 result |=0x0002;
	  }
	  //mpu_set_sample_rate
	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
	  {
	  	 printf("mpu_set_sample_rate complete ......\n");
	  }
	  else
	  {
	  	 printf("mpu_set_sample_rate error ......\n");
		 result |=0x0004;
	  }
	  //dmp_load_motion_driver_firmvare
	  if(!dmp_load_motion_driver_firmware())
	  {
	  	printf("dmp_load_motion_driver_firmware complete ......\n");
	  }
	  else
	  {
	  	printf("dmp_load_motion_driver_firmware come across error ......\n");
		 result |=0x0008;
	  }
	  //dmp_set_orientation
	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
	  {
	  	 printf("dmp_set_orientation complete ......\n");
	  }
	  else
	  {
	  	 printf("dmp_set_orientation come across error ......\n");
		 result |=0x0010;
	  }
	  //dmp_enable_feature
	  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | //DMP_FEATURE_SEND_RAW_GYRO |	// DMP_FEATURE_ANDROID_ORIENT |  //By Damm Stagner
	        DMP_FEATURE_GYRO_CAL))
	  {
	  	 printf("dmp_enable_feature complete ......\n");
	  }
	  else
	  {
	  	 printf("dmp_enable_feature come across error ......\n");
		 result |=0x0020;
	  }
	  //dmp_set_fifo_rate
	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
	  {
	  	 printf("dmp_set_fifo_rate complete ......\n");
	  }
	  else
	  {
	  	 printf("dmp_set_fifo_rate come across error ......\n");
		 result |=0x0040;
	  }
	  run_self_test();
	  if(!mpu_set_dmp_state(1))
	  {
	  	 printf("mpu_set_dmp_state complete ......\n");
	  }
	  else
	  {
	  	 printf("mpu_set_dmp_state come across error ......\n");
		 result |=0x0080;
	  }
	  return result;
}






/**********************************************************************************/

#else

//**************************************
//向I2C设备写入一个字节数据
//**************************************
void MPU6050_WriteI2C(u8 REG_Address,u8 REG_data)
{
	OS_CPU_SR cpu_sr=0;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//临界区 
	IIC1_WriteByte(MPU6050_DEFAULT_ADDRESS,REG_Address,REG_data);
	OS_EXIT_CRITICAL();								//退出临界区	
#else
	IIC1_WriteByte(MPU6050_DEFAULT_ADDRESS,REG_Address,REG_data);
#endif	
}


//**************************************
//从I2C设备读取一个字节数
//**************************************
u8 MPU6050_ReadI2C(u8 REG_Address)
{	
	OS_CPU_SR cpu_sr=0;
	u8 readval = 0x55;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//临界区 
	readval = IIC1_ReadByte(MPU6050_DEFAULT_ADDRESS,REG_Address);
	OS_EXIT_CRITICAL();								//退出临界区	
#else
	readval = IIC1_ReadByte(MPU6050_DEFAULT_ADDRESS,REG_Address);
#endif	
	return readval;
}


/**
* @brief  Reads a block of data from the MPU6050.
* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
* @param  readAddr : MPU6050's internal address to read from.
* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
* @return None
*/

void MPU6050_BufferRead(u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
	OS_CPU_SR cpu_sr=0;
#ifdef HARDWARE_I2C
	OS_ENTER_CRITICAL();						//临界区 
	IIC1_MultRead(MPU6050_DEFAULT_ADDRESS,pBuffer,readAddr,NumByteToRead);
	OS_EXIT_CRITICAL();								//退出临界区	
#else
	IIC1_MultRead(MPU6050_DEFAULT_ADDRESS,pBuffer,readAddr,NumByteToRead);
	#endif	
}



//**************************************
//合成数据
//**************************************
s16 GetData(u8 REG_Address)
{
	char H,L;
	H=MPU6050_ReadI2C(REG_Address);
	L=MPU6050_ReadI2C(REG_Address+1);
	return (s16)(H<<8)+L;   //合成数据
}



//读取3个轴的数据
//x,y,z:读取到的数据
void MPU6050_RD_XYZ(short *mpug,short *mpua,short *temp)
{
	u8 buf[14];
	MPU6050_BufferRead(buf,MPU6050_RA_ACCEL_XOUT_H,14);
//	IIC1_MultRead(MPU6050_DEFAULT_ADDRESS,buf,MPU6050_RA_ACCEL_XOUT_H,14);

	mpua[0]=(((u16)buf[0]<<8)+buf[1]); 
	mpua[1]=(((u16)buf[2]<<8)+buf[3]); 
	mpua[2]=(((u16)buf[4]<<8)+buf[5]); 
	*temp =(((u16)buf[6]<<8)+buf[7]);
	mpug[0]=(((u16)buf[8]<<8)+buf[9]); 
	mpug[1]=(((u16)buf[10]<<8)+buf[11]); 
	mpug[2]=(((u16)buf[12]<<8)+buf[13]);
}

/*
 * 函数名：MPU6050_NoneDMPInit
 * 描述  ：I2C 外设(EEPROM)初始化
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
bool MPU6050_NoneDMPInit()
{
	u8 Add;//器件地址	

//	Add=MPU6050_ReadI2C(MPU6050_RA_WHO_AM_I);
	MPU6050_BufferRead(&Add,MPU6050_RA_WHO_AM_I,1);
//	printf("ID=%x\r\n",Add);
	Delay_ms(10);
	MPU6050_WriteI2C(MPU6050_RA_PWR_MGMT_1, 0x00);		//解除休眠状态 0x6b 00
	Delay_ms(10);
	MPU6050_WriteI2C(MPU6050_RA_SMPLRT_DIV, 0x03);		//Sample Rate = Gyroscope Output Rate(1kHz) / (1 + SMPLRT_DIV)=250Hz 4ms
	Delay_ms(10);
	MPU6050_WriteI2C(MPU6050_RA_CONFIG, 0x03);				//0x03 42Hz LP
	Delay_ms(10);
	MPU6050_WriteI2C(MPU6050_RA_GYRO_CONFIG, 0x18);		//FS_SEL=3 ± 2000 °/s
	Delay_ms(10);
	MPU6050_WriteI2C(MPU6050_RA_ACCEL_CONFIG, 0x08);	//AFS_SEL=1  ± 4g
	Delay_ms(10);
	MPU6050_WriteI2C(MPU6050_RA_INT_PIN_CFG, 0x92);		//I2C_BYPASS_EN IIC 直通 /中断为低电平，上拉，自动持续50us，读操作清中断标志
	Delay_ms(10);
	MPU6050_WriteI2C(MPU6050_RA_INT_ENABLE, 0x01);		//使能数据准备好中断
	Delay_ms(10);
	MPU6050_WriteI2C(MPU6050_RA_USER_CTRL, 0x00);			//MPU6050不做主机 When I2C_BYPASS_EN is equal to 1 and
																										//I2C_MST_EN (Register 106 bit[5]) is equal to 0,
																										//the host application processor will be able to 
																										//directly access the auxiliary I2C bus of the MPU-60X0	
	if(Add!=0x68) return FALSE;
	return TRUE;
	
}
	

#endif

void MPUDataRead(void)
{
#ifdef MPU6050_DMP_DRIVER
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);
#else	
		MPU6050_RD_XYZ(gyro,accel,&tempt);
#endif	
}

