/**************************************************************************************************/
/** @file     freertos.c
 *  @brief    Code for freertos applications
 *  @details  x
 *
 *  @section 	Opens
 * 		includes sec header
 *
 *  @note	freertos.c uses main.h as the interface file
 */
/**************************************************************************************************/

//************************************************************************************************//
//                                            INCLUDES                                            //
//************************************************************************************************//

//Standard Library Includes
#include <string.h>
#include <stdio.h>

//RTOS Includes
#include "FreeRTOS.h"
#include "task.h"

//Project Includes
#include "main.h"


//************************************************************************************************//
//                                        DEFINITIONS & TYPES                                     //
//************************************************************************************************//

//-----------------------------------------  Definitions -----------------------------------------//

//Task Definitions
#define SYSTEM_TASK_LOOP_DELAY_CTS	(800)			/* @open	define this in milliseconds		  */
#define DATA_TASK_LOOP_DELAY_CTS	(800)			/* @open	define this in milliseconds		  */
#define DISPLAY_TASK_LOOP_DELAY_CTS	(800)			/* @open	define this in milliseconds		  */
#define CONTROL_TASK_LOOP_DELAY_CTS	(800)			/* @open	define this in milliseconds		  */


//************************************************************************************************//
//                                             OS VARIABLES                                       //
//************************************************************************************************//

//--------------------------------------------- Tasks --------------------------------------------//

//Tasks
osThreadId_t sysTaskHandle;
osThreadId_t dataTaskHandle;
osThreadId_t dispTaskHandle;
osThreadId_t ctrlTaskHandle;


//Config
const osThreadAttr_t sysTask_attributes = {
  .name       = SYS_TASK_NAME,
  .stack_size = DFLT_STACK_SIZE,
  .priority   = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t dataTask_attributes = {
  .name       = DATA_TASK_NAME,
  .stack_size = DFLT_STACK_SIZE,
  .priority   = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t dispTask_attributes = {
  .name       = DISP_TASK_NAME,
  .stack_size = DFLT_STACK_SIZE,
  .priority   = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t ctrlTask_attributes = {
  .name       = CTRL_TASK_NAME,
  .stack_size = DFLT_STACK_SIZE,
  .priority   = (osPriority_t) osPriorityLow,
};


//-------------------------------------------- Timers --------------------------------------------//

//Timers
osTimerId_t osTimerHandle;

//Config
const osTimerAttr_t osTimer_attributes = {
  .name = OS_TIMER_NAME
};


//------------------------------------------- Mutexes --------------------------------------------//

//Mutexes
osMutexId_t dataMutexHandle;

//Config
const osMutexAttr_t dataMutex_attributes = {
  .name = DATA_MUTEX_NAME
};


//------------------------------------------ Semaphores -------------------------------------------//

//Semaphores
osSemaphoreId_t ctrlSemHandle;
osSemaphoreId_t cntrSemHandle;

//Config
const osSemaphoreAttr_t ctrlSem_attributes = {
  .name = CTRL_SEM_NAME
};

const osSemaphoreAttr_t cntrSem_attributes = {
  .name = CNTR_SEM_NAME
};


//-------------------------------------------- Events --------------------------------------------//

//Events
osEventFlagsId_t dataStoreHandle;

//Config
const osEventFlagsAttr_t dataStore_attributes = {
  .name = DATA_EVENT_NAME
};

//PUBLIC FUNCTIONS


/**************************************************************************************************/
/** @fcn        void sysTask_Init(void *argument)
 *  @brief      Function implementing the sysTask thread.
 *  @details    GPIO & UART demos
 *
 *  @param    [in]  (void *) argument - x
 *
 *  @section 	WDT Refresh
 *  	Update counter value to !127, the refresh window is between
 *  	!35 ms (!~728 * (!127-!80)) and !46 ms (!~728 * !64)
 */
/**************************************************************************************************/
void sysTask_Init(void *argument) {

	//Locals
#ifdef WDT_IS_WORKING
	HAL_StatusTypeDef stat = HAL_ERROR;				/* status of HAL operations for review 		  */
#endif

	//Loop
	for(;;) {

		//Notify
		_printf("Calling System Task\n\r");

		//Wiggle
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

#ifdef WDT_IS_WORKING
		//Refresh
		stat = HAL_WWDG_Refresh(&hwwdg);
		Error_Catch(stat);
#endif

		//Delay
		osDelay(SYSTEM_TASK_LOOP_DELAY_CTS);
	}
}


/**************************************************************************************************/
/** @fcn        void dataTask_Init(void *argument)
 *  @brief      Function implementing the dataTask thread.
 *  @details    Timer demo
 *
 *  @param    [in]  (void *) argument - x
 *
 *  @section 	Opens
 *  	Move print to each thread! with staggered timing
 */
/**************************************************************************************************/
void dataTask_Init(void *argument) {

	//Locals
	uint32_t timer_val  = 0;
	char buff[100]      = {0};

	//Init
	memset(&buff, 0x00, sizeof(buff));

	//Loop
	for(;;) {

		//Notify
		_printf("Calling Data Task\n\r");

		//Check
		timer_val = __HAL_TIM_GetCounter(&htim1);

		sprintf(&buff[0], "Timer: 0x%08lu\n\r\n\r", timer_val);

		//Print
		_printf(buff);

		//Delay
		osDelay(DATA_TASK_LOOP_DELAY_CTS);
	}
}


/**************************************************************************************************/
/** @fcn        void dispTask_Init(void *argument)
 *  @brief      Function implementing the dispTask thread.
 *  @details    Semaphore demo
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void dispTask_Init(void *argument) {

	//Loop
	for(;;) {

		//Notify
		_printf("Calling Disp Task\n\r");

		//Delay
		osDelay(DISPLAY_TASK_LOOP_DELAY_CTS);
	}
}


/**************************************************************************************************/
/** @fcn        void ctrlTask_Init(void *argument)
 *  @brief      Function implementing the ctrlTask thread.
 *  @details    x
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void ctrlTask_Init(void *argument) {

	//Loop
	for(;;) {

		//Notify
		_printf("Calling Control Task\n\r");

		//Delay
		osDelay(DISPLAY_TASK_LOOP_DELAY_CTS);
	}
}


/**************************************************************************************************/
/** @fcn        void osTimer_Callback(void *argument)
 *  @brief      osTimer_Callback function
 *  @details    x
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void osTimer_Callback(void *argument) {
	return;
}

