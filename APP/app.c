/******************** (C) COPYRIGHT 2015 DammStanger *****************************************
**--------------�ļ���Ϣ----------------------------------------------------------
 * �ļ���	��app.c
 * ��	��	������Ӧ�ó���
 *                    
 * ʵ��ƽ̨	��FCU V1.0
 * Ӳ������	��
 * �� 	��	��V1.0.150610
 * ������ϵ	��FCU_UCOSII150602_UCOSII2.91
 * ��汾	��ST3.0.0
 * ����ʱ��	��2014.3.28
 * ���༭	��2015.6.10
 **-------------------------------------------------------------------------------

 * ��	��	��Damm Stanger
 * ��	��	��dammstanger@qq.com
**********************************************************************************************/
#include "includes.h"

OS_STK task_led_stk[TASK_LED_STK_SIZE];		//����ջ

OS_STK task_CrucialInterract_stk[TASK_CRUCIALINTERRACT_STK_SIZE];	//����ջ

OS_STK task_MPU6050_stk[TASK_MPU6050_STK_SIZE];	//����ջ

OS_STK task_Debug_stk[TASK_DEBUG_STK_SIZE];		//����ջ

OS_STK task_PWMout_stk[TASK_PWMout_STK_SIZE];	//����ջ

OS_STK task_PWMin_stk[task_PWMin_STK_SIZE];		//����ջ

OS_STK task_Sonar_stk[TASK_SONAR_STK_SIZE];		//����ջ

OS_STK task_Magnet_stk[TASK_MAGNET_STK_SIZE];	//����ջ

OS_EVENT * g_Msgsem_dbg;						//���Ե���Ϣ��
OS_EVENT * g_Msgsem_ctl_att;					//��̬���Ƶ���Ϣ��

bool mpu = FALSE;
u8	 g_Printcnt = 0;
bool DataDealOK = FALSE;
bool MagCalibOK = FALSE;
static float Roll_Raw_Rad_Op_1 = 0,Pitch_Raw_Rad_Op_1 = 0;//Roll_Raw_Rad_Op = 0,Pitch_Raw_Rad_Op = 0,

void Task_Start(void *p_arg)					//������
{
	OS_CPU_SR cpu_sr=0;
	bool result = FALSE;
    (void)p_arg;                				// 'p_arg' ��û���õ�����ֹ��������ʾ����
	
	/* creat message box*/
	g_Msgsem_dbg=OSSemCreate(0);						//�����ź���
	g_Msgsem_ctl_att=OSSemCreate(0);						//�����ź���
	Delay_ms(1000);	
	
//---��������ĳ�ʼ��----------------------------------------------------------
	PID_Para_Init();			//���ڹ���������ʼ��֮ǰ
	OPFLOW_Para_Init();

//---�����ٽ��----------------------------------------------------------------	
	OS_ENTER_CRITICAL();						//�ٽ���  
	OSTaskCreate(Task_Debug,(void *)0,		   	//��������1
	   &task_Debug_stk[TASK_DEBUG_STK_SIZE-1], TASK_DEBUG_PRIO);
	
	OSTaskCreate(Task_PWMin,(void *)0,		   	//��������2
	   &task_PWMin_stk[task_PWMin_STK_SIZE-1], TASK_PWMin_PRIO);
	
	LED1(ON);
	printf("\r\n FCU V1.0 Init... \r\n");	
	Delay_ms(1000);	
	LED1(OFF);
	
//	printf("type int have %d byes\r\n",sizeof(tset));
	do{
		result = MPU6050_NoneDMPInit();
		Delay_ms(500);	
	}while(result!=TRUE);
	printf("MPU Init Succeeded!\r\n");
	LED1(ON);
	do{
		result = HMC5883L_Init();
		Delay_ms(500);	
	}while(result!=TRUE);
	LED1(OFF);
#if OPFLOW_SENSOR_ONBOARD_SPI_EN	
	do{
		result = ADNS3080_DeviceInit();
		Delay_ms(500);	
	}while(result!=TRUE);
#endif	
	
	OSTaskCreate(Task_CrucialInterract,(void *)0,		   	//��������3
		&task_CrucialInterract_stk[TASK_CRUCIALINTERRACT_STK_SIZE-1], TASK_CRUCIALINTERRACT_PRIO);
	
	OSTaskCreate(Task_LEDState,(void *)0,		  	//��������3
		&task_led_stk[TASK_LED_STK_SIZE-1], TASK_LED_PRIO);

	OSTaskCreate(Task_MPU6050,(void *)0,		  	//��������4
		&task_MPU6050_stk[TASK_MPU6050_STK_SIZE-1], TASK_MPU6050_PRIO);
//		
	
	OSTaskCreate(Task_Magnet,(void *)0,		  		//��������5
		&task_Magnet_stk[TASK_MAGNET_STK_SIZE-1], TASK_MAGNET_PRIO);
//	

	OSTaskCreate(Task_ThrottleLoop,(void *)0,		//��������6
		&task_Sonar_stk[TASK_SONAR_STK_SIZE-1], TASK_SONAR_PRIO);
	

	OSTaskCreate(Task_PWMout,(void *)0,		  		//��������7
		&task_PWMout_stk[TASK_PWMout_STK_SIZE-1], TASK_PWMout_PRIO);
		
//-----------------------------------------------------------------------------------
	OSTaskSuspend(STARTUP_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();													//�˳��ٽ���
	
}

void Task_CrucialInterract(void *p_arg)
{
	static u16 pollcnt = 0;
	(void)p_arg;  
	while(1)
	{
		pollcnt++;
		
		if(Trans_RevPakFin)
		{
			Trans_RevPakFin = 0;
			Pak_Handle();
			OPFLOW_DataUpdate();
		}
		if(pollcnt>=10)				//Լ50ms 		20Hz
		{	pollcnt = 0;
			
			//-----------------------------------------------------
			if(Trans_RevPakErr)
			{
				Trans_RevPakErr = 0;
				USART1_EnableRevDMA();			//���������¸�DMA��������ֵ����֤�´����ݴ�����ͷ��װ��
//				USART1_DMAConfig_Sim();			//���������DMA�жϲ��������������ǰ		
			}
			//-----------------------------------------------------
			RTDATA_Cmd_Pkg_Send();
//			AP_TEST_Cmd_Pkg_Send();
	  									
			Update_Position(Roll_Raw_Rad_Op_1,Pitch_Raw_Rad_Op_1,0,1,(s16)(SonarHight/10));

//			Roll_Raw_Rad_Op = Roll_Raw_Rad_Op_1;
//			Pitch_Raw_Rad_Op = Pitch_Raw_Rad_Op_1;	
			Roll_Raw_Rad_Op_1 = Roll_Raw_Rad;
			Pitch_Raw_Rad_Op_1 = Pitch_Raw_Rad;		//���ڹ����ɼ��ͺ�һ���ڣ���ǰһ����ʱ�ĽǶ�ֵ

			OSTimeDlyHMSM(0,0,0,4);		//ԼΪ5ms
		}
		else 
			OSTimeDlyHMSM(0,0,0,5);	
	}
}

void Task_MPU6050(void *p_arg)
{	
	static bool Gyro_OffsetOK = TRUE ;			//��У׼
	static bool Acc_OffsetOK = FALSE ;
	static u8 Once=160;
	u32 currentT;
	(void)p_arg;                	
    while (1)
	{
		currentT = OSTimeGet();
//**************У������**********************************************************	
		if(Gyro_OffsetOK==FALSE)					//��ƫδУ��������У��
		{
			if(!MPUGyroZeroCal())					//��ƫУ��
			{	
				Gyro_OffsetOK = TRUE;		
			}
		}
		if(Acc_OffsetOK==FALSE)					//��ƫδУ��������У��
		{
			if(!MPUAccZeroCal_GravityMeasure())		//���ٶ���ƫУ��������������
			{	
				Acc_OffsetOK = TRUE;
			}
		}

////*******************************************************************************		
		if(Acc_OffsetOK&&Gyro_OffsetOK)
		{	
			mpu = TRUE;
			if(MagCalibOK==TRUE)
			{		
				AttDataProcess();					//��ʱ900us/5ms	
				FlyModeProcess();
				
				if(g_Printcnt++==9)
				{
					g_Printcnt = 0;
					OSSemPost(g_Msgsem_dbg);
				}
				OSSemPost(g_Msgsem_ctl_att);
				DataDealOK = TRUE;					//		

				if(Once>1)
				{
					LED3( ON );			
					Once--;
				}
				else if(Once==1) {LED3( OFF );Once=0;}  //��Once=0֮��Ͳ������LED3��ʹ��
			}
		}
		if(currentT==OSTimeGet())					//1ms�����
			OSTimeDlyHMSM(0,0,0,4);						//1ms����ɣ���ʱ4ms
		else
			OSTimeDlyHMSM(0,0,0,3);					//���ʱ��>1ms����ʱ3ms	��������������Ϊ4ms
    }
}

void Task_Magnet(void *p_arg)
{
	tg_HMC5883L_FLOAT temph;
    (void)p_arg;
	
	while(mpu==FALSE) OSTimeDlyHMSM(0,0,0,1);						//�ȴ�MPUУ����� 141202

//	HMC5883L_BiasCalibrate();				//��5883У��
//	HMC5883L_Init();
	OSTimeDlyHMSM(0,0,0,10);
	HMC5883L_MultRead(&hmc5883l);			//����һ��

	MagCalibOK = TRUE;						//У�����

	while(1)
	{		
		temph.hx = hmc5883l.hx;
		temph.hy = hmc5883l.hy;

		HMC5883L_Cal(&temph);
		hmc5883l.m_yaw = temph.m_yaw * RAD_TO_DEG;
		OSTimeDlyHMSM(0,0,0,15);				//TԼ=15ms	
	}
}

void Task_Debug(void *p_arg)
{	
	static bool _24L01onDebug = 0;
	INT8U err;
	u8 result = 0;
    (void)p_arg;
	//--------------------------------------------------------------------
	if(NRF24L01_Check()==ERROR)//��ⲻ��24L01
	{
		_24L01onDebug = 0;
		printf("didn't find 24L01");
	}
	else 
	{
		printf("24L01 have found.");
		_24L01onDebug = 1;
	}
	while(1)
	{	
		OSSemPend(g_Msgsem_dbg, 1000, &err);			//�ȴ�100 0ms		��������40ms
		if(_24L01onDebug)
		{	
			if(RevFinish)
			{
				RevFinish = 0;
				result = _24L01PIDParData();
				switch(result)
				{
					case 0x11 :SendDebugDat_LabVIEW_2401(0x11,PID_STABLE.P*1000,PID_STABLE.I*10000,PID_STABLE.D*10000,PID_STABLE.Imax*10);break;	
					case 0x22 :SendDebugDat_LabVIEW_2401(0x22,PID_ROL.P*1000,PID_ROL.I*10000,PID_ROL.D*10000,PID_ROL.Imax*10);break;	
					case 0x44 :SendDebugDat_LabVIEW_2401(0x44,PID_YAW.P*1000,PID_YAW.I*10000,PID_YAW.D*10000,PID_YAW.Imax*10);break;	
					case 0x55 :SendDebugDat_LabVIEW_2401(0x55,PID_HIGH.P*1000,PID_HIGH.I*10000,PID_HIGH.D*10000,PID_HIGH.Imax*10);break;	
					case 0x66 :SendDebugDat_LabVIEW_2401(0x66,PID_POS_OF.P*1000,PID_POS_OF.I*10000,PID_POS_OF.D*10000,PID_POS_OF.Imax*10);break;	
					case 0x77 :SendDebugDat_LabVIEW_2401(0x77,P_RATE_ALT.P*1000,P_RATEFILT_ALT.P*1000,0,0);break;	
					case 0x88 :SendDebugDat_LabVIEW_2401(0x88,PID_ACC_ALT.P*1000,PID_ACC_ALT.I*10000,PID_ACC_ALT.D*10000,PID_ACC_ALT.Imax*10);break;	
					default :  SendDebugDat_LabVIEW_2401(0x01,P_RATE_ALT.P*1000,P_RATEFILT_ALT.P*1000,PID_ACC_ALT.P*1000,PID_ACC_ALT.I*10000);break;	
				}
				ALARM(ON);	
				OSTimeDlyHMSM(0,0,0,200); 
				ALARM(OFF);					
			}
			else
//			SendDebugDat_LabVIEW_2401(0xFF,dx_cm_accuml*10,Headhold,Yaw_Last,Control_proc_att.yaw);
//			SendDebugDat_LabVIEW_2401(0xFF,dx_cm_accuml*10,dy_cm_accuml*10,Yaw_Last,Control_proc_att.yaw);
//			SendDebugDat_LabVIEW_2401(0xFF,Rc_Data.ROLL,Rc_Data.PITCH,Rc_Data.THROTTLE,Rc_Data.YAW);
//			SendDebugDat_LabVIEW_2401(0xFF,Roll_Last,Pitch_Last,ACC_Last.X,ACC_Last.Y);
			SendDebugDat_LabVIEW_2401(0xFF,Controller_target_alt,SonarHight,Control_proc_thro,g_land_finish);
//			SendDebugDat_LabVIEW_2401(0xFF,target_output,vertical_acc_mm,PID_ACC_ALT.Pout,g_LVel_n.Z);
//			SendDebugDat_LabVIEW_2401(0xFF,desired_rate,vertical_rate_error,vacc_error,PID_ACC_ALT.ALLout);
//			SendDebugDat_LabVIEW_2401(0xFF,Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4);

		}	//Moto_PWM_1 ACC_Raw,SonarHightvel,ACC_Last,ACC_OFFSET  GYRO_Raw,GYRO_Last,GYRO_OFFSET SonarHight										
		else										//���ڵ���ģʽ
		{
			SendDebugDat_Hunter(Optflow.x_add_cm,Optflow.y_add_cm,surface_quality,Rc_Data.ROLL);		//��ʱ960us/50ms
			if(RevFinish)
			{
				RevFinish = 0;
//				USARTPIDDebug(ReceiveData+2);//						
			}
		}
    }
}


void Task_ThrottleLoop(void *p_arg)
{
	float z_rate_tmp;
	(void)p_arg;
	while(1)
	{
#if SONAR_ONBOARD		
		SR04_Start();
		OSTimeDlyHMSM(0,0,0,50);		//���������������Ϊ29.4ms ��ʱ�㹻��׼������
		while(!(TIM4_Cap2STA&0x80));
		SonarDataProcess();		
#else
		Sonar_DataUpdate();
		z_rate_tmp = AHRS_AltitudeVel(ACC_Linear_n.Z,SonarHightvel);
		g_LVel_n.Z = z_rate_tmp;
#endif
		ThrottleModeProcess();
		Update_Land_Detector();
		OSTimeDlyHMSM(0,0,0,100);
 		//�Ƽ���100ms�������
	}
}

//����_���PWM����
void Task_PWMout(void *p_arg)
{
	INT8U err;
	bool enable = FALSE;
  (void)p_arg;
	while(DataDealOK!=TRUE) OSTimeDlyHMSM(0,0,0,1);
	OSTimeDlyHMSM(0,0,2,0); 
	
	TIM1_Mode_Config();
    while (1)
    {	
		OSSemPend(g_Msgsem_ctl_att, 100, &err);			//�ȴ�100ms	������׼����

		enable = SafeDeal(Control_ulti_att.roll,Control_ulti_att.pitch);	
		CONTROL(Control_ulti_att,Control_targer_altacc,Control_proc_thro,enable);
	}
}

void Task_PWMin(void *p_arg)
{
    (void)p_arg;
	TIM_ClearFlag(TIM2, TIM_FLAG_Update|TIM_FLAG_CC1|TIM_FLAG_CC2|TIM_FLAG_CC3|TIM_FLAG_CC4);						/* �������жϱ�־ */
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,ENABLE);   
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update|TIM_FLAG_CC3|TIM_FLAG_CC4);						/* �������жϱ�־ */
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4,ENABLE); 

    while (1)
    {
		RCDataProcess();
		OSTimeDlyHMSM(0,0,0,20); 
	}
}



//����1
void Task_LEDState(void *p_arg)
{
	u8 cnt = 0;
	(void)p_arg;                	
	
    while (1)
    {
		switch(FlightMode)
		{
			case UNARMED :{
				LED4(OFF);
				LED2(OFF);
			}break;
			case ARMED :{
				LED4(ON);
				LED2(OFF);
			}break;
			case ATTITUDE :{
				LED2(OFF);
			}break;
			case HEADFREE :{
				
			}break;
			case ATLHOLD :{
				LED2(ON);
			}break;
			case POSHOLD :{
				if(!(cnt%4))LEDTrg(2);	
			}break;
			case AUTO :{
				LEDTrg(2);	
			}break;
		}
		if(cnt++==10)
		{
			cnt = 0;
			LEDTrg(1);
		}
		OSTimeDlyHMSM(0,0,0,50); 

    }
}

