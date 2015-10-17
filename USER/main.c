#include "includes.h"
#include "Project_cfg.h"
OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE];		  //∂®“Â’ª
  
int main(void)
{
	BSP_Init();
	OSInit();
	OSTaskCreate(Task_Start,(void *)0,
	&startup_task_stk[STARTUP_TASK_STK_SIZE-1], STARTUP_TASK_PRIO);

	OSStart();
	return 0;
 }

/************END OF FILE************/


