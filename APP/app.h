#ifndef	_APP_H_
#define	_APP_H_

/**************** �û��������� *******************/
void Task_Start(void *p_arg);
void Task_LEDState(void *p_arg);
void Task_CrucialInterract(void *p_arg);
void Task_MPU6050(void *p_arg);
void Task_Debug(void *p_arg);
void Task_PWMout(void *p_arg);
void Task_PWMin(void *p_arg);
void Task_ThrottleLoop(void *p_arg);
void Task_Magnet(void *p_arg);

#endif	//_APP_H_
