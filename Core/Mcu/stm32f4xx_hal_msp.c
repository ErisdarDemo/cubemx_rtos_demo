/**************************************************************************************************/
/** @file     stm32f4xx_hal_msp.c
 *  @brief    MSP Initialization and de-Initialization codes
 *  @details  x
 */
/**************************************************************************************************/

//Standard Library Includes
#include <string.h>

//Project Includes
#include "main.h"


/**************************************************************************************************/
/** @fcn        void HAL_MspInit(void)
 *  @brief      Initializes the Global MSP
 *  @details    x
 */
/**************************************************************************************************/
void HAL_MspInit(void) {

	//Init System Components
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  //Init System Interrupts
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);  		/* PendSV_IRQn interrupt configuration 		  */

  return;
}


/**************************************************************************************************/
/** @fcn        void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
 *  @brief      TIM_Base MSP Initialization
 *  @details    This function configures the hardware resources used in this example
 *
 *  @param    [in]  (TIM_HandleTypeDef *) htim_base - TIM_Base handle pointer
 */
/**************************************************************************************************/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {

  if(htim_base->Instance == TIM1) {

	  //Enable Timer Clock
    __HAL_RCC_TIM1_CLK_ENABLE();
  }

  return;
}


/**************************************************************************************************/
/** @fcn        void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
 *  @brief      TIM_Base MSP De-Initialization
 *  @details    This function freeze the hardware resources used in this example
 *
 *  @param    [in]  (TIM_HandleTypeDef *) htim_base - TIM_Base handle pointer
 */
/**************************************************************************************************/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {

  if(htim_base->Instance == TIM1) {

    //Disable Timer Clock
    __HAL_RCC_TIM1_CLK_DISABLE();
  }

  return;
}


/**************************************************************************************************/
/** @fcn        void HAL_UART_MspInit(UART_HandleTypeDef* huart)
 *  @brief      UART MSP Initialization
 *  @details    This function configures the hardware resources used in this example
 *
 *  @param    [in]  (UART_HandleTypeDef *) huart - UART handle pointer
 *
 *  @section 	USART2 GPIO Config
 *      PA2     ------> USART2_TX
 *      PA3     ------> USART2_RX
 */
/**************************************************************************************************/
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {

	//Locals
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //Init Variables
  memset(&GPIO_InitStruct, 0x00, sizeof(GPIO_InitStruct));


  if(huart->Instance == USART2) {

    //Init Clks
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    //Init Config
    GPIO_InitStruct.Pin       = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

    //Init GPIO
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  return;
}


/**************************************************************************************************/
/** @fcn        void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
 *  @brief      UART MSP De-Initialization
 *  @details    This function freeze the hardware resources used in this example
 *
 *  @param    [in]  (UART_HandleTypeDef *) huart - UART handle pointer
 *
 *  @section 	USART2 GPIO Config
 *      PA2     ------> USART2_TX
 *      PA3     ------> USART2_RX
 */
/**************************************************************************************************/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {

  if(huart->Instance == USART2) {

    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();


    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);
  }

  return;
}


/**************************************************************************************************/
/** @fcn        void HAL_WWDG_MspInit(WWDG_HandleTypeDef* hwwdg)
 *  @brief      WWDG MSP Initialization
 *  @details    This function configures the hardware resources used in this example
 *
 *  @param    [in]  (WWDG_HandleTypeDef *) hwwdg - WWDG handle pointer
 */
/**************************************************************************************************/
void HAL_WWDG_MspInit(WWDG_HandleTypeDef* hwwdg) {

  if(hwwdg->Instance == WWDG) {

    /* Peripheral clock enable */
    __HAL_RCC_WWDG_CLK_ENABLE();

  }

  return;
}

