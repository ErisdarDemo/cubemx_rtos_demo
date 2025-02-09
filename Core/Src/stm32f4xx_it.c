/**************************************************************************************************/
/** @file     stm32f4xx_it.c
 *  @brief    Interrupt Service Routines
 *  @details  x
 *
 *  @section 	Use
 *  	Add additional interrupt handlers to the base of this file
 */
/**************************************************************************************************/


//************************************************************************************************//
//                                            INCLUDES                                            //
//************************************************************************************************//

//Standard Library Includes
#include <stdint.h>

//SDK Includes
#include "stm32f4xx_it.h"

//RTOS Includes
#include "FreeRTOS.h"
#include "task.h"

//Project Includes
#include "main.h"


//************************************************************************************************//
//                     Cortex-M4 Processor Interruption and Exception Handlers                    //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        void NMI_Handler(void)
 *  @brief      This function handles Non maskable interrupt
 *  @details    x
 *
 *  @return   no return
 */
/**************************************************************************************************/
void NMI_Handler(void) {
	for(;;);
}


/**************************************************************************************************/
/** @fcn        void HardFault_Handler(void)
 *  @brief      This function handles Hard fault interrupt
 *  @details    x
 *
 *  @return   no return
 */
/**************************************************************************************************/
void HardFault_Handler(void) {
	for(;;);
}


/**************************************************************************************************/
/** @fcn        void MemManage_Handler(void)
 *  @brief      This function handles Memory management fault
 *  @details    x
 *
 *  @return   no return
 */
/**************************************************************************************************/
void MemManage_Handler(void) {
	for(;;);
}


/**************************************************************************************************/
/** @fcn        void BusFault_Handler(void)
 *  @brief      This function handles Pre-fetch fault, memory access fault
 *  @details    x
 *
 *  @return   no return
 */
/**************************************************************************************************/
void BusFault_Handler(void) {
	for(;;);
}


/**************************************************************************************************/
/** @fcn        void UsageFault_Handler(void)
 *  @brief      This function handles Undefined instruction or illegal state
 *  @details    x
 *
 *  @return   no return
 */
/**************************************************************************************************/
void UsageFault_Handler(void) {
	for(;;);
}


/**************************************************************************************************/
/** @fcn        void DebugMon_Handler(void)
 *  @brief      This function handles Debug monitor
 *  @details    x
 *
 *  @return   no return
 */
/**************************************************************************************************/
void DebugMon_Handler(void) {
	return;
}


/**************************************************************************************************/
/** @fcn        void SysTick_Handler(void)
 *  @brief      This function handles System tick timer
 *  @details    x
 *
 *  @return   no return
 *
 *  @section 	Opens
 *  	Drop that shit
 */
/**************************************************************************************************/
void SysTick_Handler(void) {

	//Locals
	BaseType_t stat = UINT32_MAX;

	//Update
	HAL_IncTick();

	//Check
	stat = xTaskGetSchedulerState();

	//Handle
	if(stat != taskSCHEDULER_NOT_STARTED) {
		xPortSysTickHandler();
	}

	return;
}

