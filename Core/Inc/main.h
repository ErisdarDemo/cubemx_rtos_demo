/**************************************************************************************************/
/** @file     main.h
 *  @brief    Header for main.c file
 *  @details  This file contains the common defines of the application
 *
 *  @section    Opens
 *      none current
 *
 *  @note	freertos.c uses main.h as its interface file
 */
/**************************************************************************************************/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


//************************************************************************************************//
//                                            INCLUDES                                            //
//************************************************************************************************//

//BSP Includes
#include "stm32f4xx_hal.h"

//FreeRTOS Includes
#include "cmsis_os.h"


//************************************************************************************************//
//                                        DEFINITIONS & TYPES                                     //
//************************************************************************************************//

//-----------------------------------------  Definitions -----------------------------------------//

//Pin Definitions
#define B1_Pin     			(GPIO_PIN_13)
#define B1_GPIO_Port 		(GPIOC)
#define USART_TX_Pin 		(GPIO_PIN_2)
#define USART_TX_GPIO_Port 	(GPIOA)
#define USART_RX_Pin 		(GPIO_PIN_3)
#define USART_RX_GPIO_Port 	(GPIOA)
#define LD2_Pin 			(GPIO_PIN_5)
#define LD2_GPIO_Port 		(GPIOA)
#define TMS_Pin 			(GPIO_PIN_13)
#define TMS_GPIO_Port 		(GPIOA)
#define TCK_Pin 			(GPIO_PIN_14)
#define TCK_GPIO_Port 		(GPIOA)
#define SWO_Pin 			(GPIO_PIN_3)
#define SWO_GPIO_Port 		(GPIOB)

//--------------------------------------- RTOS Definitions ---------------------------------------//

//Code Definitions
#define _nop()				asm("NOP")

//Design Specifications
#define DFLT_STACK_SIZE		(128 * 4)				/* @open 	capture meaning of each num		  */

//Task Definitions
#define SYS_TASK_NAME		"sysTask"
#define DATA_TASK_NAME		"dataTask"
#define DISP_TASK_NAME		"dispTask"
#define CTRL_TASK_NAME		"ctrlTask"

//Timer Definitions
#define OS_TIMER_NAME		"osTimer"

//Mutex Definitions
#define  DATA_MUTEX_NAME	"dataMutex"

//Semaphore Definitions
#define CTRL_SEM_NAME		"ctrlSem"
#define CTRL_MAX_CT 		(1)						/* @open  	value descrip					  */
#define CTRL_INIT_CT 		(1)						/* @open  	value descrip					  */

#define CNTR_SEM_NAME		"cntrSem"
#define CNTR_MAX_CT 		(10)					/* @open  	value descrip					  */
#define CNTR_INIT_CT 		(0)						/* @open  	value descrip					  */

//Event Definitions
#define DATA_EVENT_NAME		"dataStore"


//************************************************************************************************//
//                                       FUNCTION DECLARATIONS                                    //
//************************************************************************************************//

//Error Handling
extern void Error_Handler(void);
extern void Error_Catch(HAL_StatusTypeDef stat);

//--------------------------------------------- STM32 --------------------------------------------//
extern TIM_HandleTypeDef  htim1;
extern UART_HandleTypeDef huart2;
extern WWDG_HandleTypeDef hwwdg;

//------------------------------------------- FreeRTOS -------------------------------------------//

//Task Prototypes
extern void sysTask_Init(void *argument);
extern void dataTask_Init(void *argument);
extern void dispTask_Init(void *argument);
extern void ctrlTask_Init(void *argument);

//Timer Prototypes
extern void osTimer_Callback(void *argument);


//************************************************************************************************//
//                                             VARIABLES                                          //
//************************************************************************************************//

//Tasks
extern osThreadId_t sysTaskHandle;
extern osThreadId_t dataTaskHandle;
extern osThreadId_t dispTaskHandle;
extern osThreadId_t ctrlTaskHandle;

//Attributes
extern const osThreadAttr_t sysTask_attributes;
extern const osThreadAttr_t dataTask_attributes;
extern const osThreadAttr_t dispTask_attributes;
extern const osThreadAttr_t ctrlTask_attributes;

//Timers
extern osTimerId_t osTimerHandle;

//Mutexes
extern osMutexId_t dataMutexHandle;

//Semaphores
extern osSemaphoreId_t ctrlSemHandle;
extern osSemaphoreId_t cntrSemHandle;

//Events
extern osEventFlagsId_t dataStoreHandle;

//Config
extern const osTimerAttr_t osTimer_attributes;
extern const osMutexAttr_t dataMutex_attributes;
extern const osSemaphoreAttr_t ctrlSem_attributes;
extern const osSemaphoreAttr_t cntrSem_attributes;
extern const osEventFlagsAttr_t dataStore_attributes;


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
