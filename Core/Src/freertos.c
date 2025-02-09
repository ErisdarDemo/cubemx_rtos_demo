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
#define DATA_TASK_LOOP_DELAY_CTS	(1000)			/* @open	define this in milliseconds		  */


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
 *  @details    x
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

		//Wiggle
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

#ifdef WDT_IS_WORKING
		//Refresh
		stat = HAL_WWDG_Refresh(&hwwdg);
		Error_Catch(stat);
#endif

		//Delay
		osDelay(1);
	}
}


/**************************************************************************************************/
/** @fcn        void dataTask_Init(void *argument)
 *  @brief      Function implementing the dataTask thread.
 *  @details    x
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void dataTask_Init(void *argument) {

	//Loop
	for(;;) {

		//Notify
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		//Delay
		osDelay(DATA_TASK_LOOP_DELAY_CTS);

		//Notify
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		//Delay
		osDelay(DATA_TASK_LOOP_DELAY_CTS);
	}
}


/**************************************************************************************************/
/** @fcn        void dispTask_Init(void *argument)
 *  @brief      Function implementing the dispTask thread.
 *  @details    x
 *
 *  @param    [in]  (void *) argument - x
 */
/**************************************************************************************************/
void dispTask_Init(void *argument) {

	//Loop
	for(;;) {
		osDelay(1);
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
		osDelay(1);
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

