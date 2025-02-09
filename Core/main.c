/**************************************************************************************************/
/** @file     main.c
 *  @brief    Main program body
 *  @details  x
 *
 *  @author   Justin Reina, Firmware Engineer
 *  @created  2/8/25
 *  @last rev 2/8/25
 *
 *  @section 	Opens
 * 	   semaphores demo
 * 	   mutex demo
 * 	   event demo
 * 	   swap to c++
 */
/**************************************************************************************************/

//************************************************************************************************//
//                                              INCLUDES                                          //
//************************************************************************************************//

//Standard Library Includes
#include <stdint.h>
#include <string.h>
#include <stdio.h>


//Project Includes
#include "main.h"

//FreeRTOS Includes
#include "cmsis_os.h"


//************************************************************************************************//
//                                            DEFINITIONS                                         //
//************************************************************************************************//

//------------------------------------- Peripheral Definitions -----------------------------------//

//PLL Definitions
#define PLL_DIV_FACTOR		(16)					/* @open  	value descrip					  */
#define PLL_MULT_FACTOR		(336)					/* @open  	value descrip					  */
#define PLL_DIV_Q_FACTOR	(2)						/* @open  	value descrip					  */
#define PLL_DIV_R_FACTOR	(2)						/* @open  	value descrip					  */

//Timer Definitions
#define TIM_PRESCALER_VAL	(0)						/* @open  	value descrip					  */
#define TIM_PERIOD_VAL		(65535) //0xFFFF)				/* @open  	value descrip					  */
#define TIM_REP_VAL			(0)						/* @open  	value descrip					  */

//USART Definitions
#define UART_BAUD_RATE_BPS	(115200)				/* @open  	value descrip					  */

//Watchdog Definitions
#define WWDG_WINDOW_VAL		(64)					/* @open  	value descrip					  */
#define WWDG_CTR_VAL		(64)					/* @open  	value descrip					  */


//************************************************************************************************//
//                                            SDK VARIABLES                                       //
//************************************************************************************************//

//Periphs
TIM_HandleTypeDef  htim1;
UART_HandleTypeDef huart2;
WWDG_HandleTypeDef hwwdg;


//************************************************************************************************//
//                                   PRIVATE FUNCTION PROTOTYPES                                  //
//************************************************************************************************//

//System Prototypes
void SystemClock_Config(void);

//Cube Prototypes
static void GPIO_Init(void);
static void USART2_UART_Init(void);
static void TIM1_Init(void);
#ifdef WDT_IS_WORKING
static void WWDG_Init(void);
#endif

//************************************************************************************************//
//                                        PRIVATE ROUTINES                                        //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        int main(void)
 *  @brief      The application entry point
 *  @details    x
 *
 *  @return   (int) exit return status
 *
 *  @section 	Opens
 *  	sys_init()
 */
/**************************************************************************************************/
int main(void) {

	//********************************************************************************************//
	//                                          MCU INIT                                          //
	//********************************************************************************************//

	//Init HAL
	HAL_Init();

	//Init Clocks
	SystemClock_Config();

	//Init Peripheral
	GPIO_Init();
	USART2_UART_Init();
	TIM1_Init();
#ifdef WDT_IS_WORKING
	WWDG_Init();
#endif

	//Init Scheduler
	osKernelInitialize();


	//------------------------------------ Initialize Mutexes ------------------------------------//

	//Init Mutexes
	dataMutexHandle = osMutexNew(&dataMutex_attributes);


	//----------------------------------- Initialize Semaphores ----------------------------------//

	//Init Semaphores
	ctrlSemHandle = osSemaphoreNew(CTRL_MAX_CT, CTRL_INIT_CT, &ctrlSem_attributes);
	cntrSemHandle = osSemaphoreNew(CNTR_MAX_CT, CNTR_INIT_CT, &cntrSem_attributes);


	//------------------------------------- Initialize Timers ------------------------------------//

	//Init Timers
	osTimerHandle = osTimerNew(osTimer_Callback, osTimerPeriodic, NULL, &osTimer_attributes);


	//------------------------------------- Initialize Tasks -------------------------------------//

	//Init Tasks
	sysTaskHandle  = osThreadNew(sysTask_Init,  NULL, &sysTask_attributes);
	dataTaskHandle = osThreadNew(dataTask_Init, NULL, &dataTask_attributes);
	dispTaskHandle = osThreadNew(dispTask_Init, NULL, &dispTask_attributes);
	ctrlTaskHandle = osThreadNew(ctrlTask_Init, NULL, &ctrlTask_attributes);


	//------------------------------------ Initialize Events -------------------------------------//

	//Init Events
	dataStoreHandle = osEventFlagsNew(&dataStore_attributes);

	//------------------------------------- Initialize RTOS --------------------------------------//

	//Notify
	_printf("Operating system in initialized\n\r");

	//Start Scheduler
	osKernelStart();

	//Loop
	for(;;) {
		_nop();										/* @open ErrorHandler? should never get here  */
	}
}


/**************************************************************************************************/
/** @fcn        void SystemClock_Config(void)
 *  @brief      System Clock Configuration
 *  @details    x
 */
/**************************************************************************************************/
void SystemClock_Config(void) {

	//Locals
	HAL_StatusTypeDef stat = HAL_ERROR;				/* status of HAL operations for review 		  */
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	//Init Variables
	memset(&RCC_OscInitStruct, 0x00, sizeof(RCC_OscInitStruct));
	memset(&RCC_ClkInitStruct, 0x00, sizeof(RCC_ClkInitStruct));


	//------------------------------------------- Setup ------------------------------------------//

	//Setup VREG
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	//RCC Oscillator Config
	RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM            = PLL_DIV_FACTOR;
	RCC_OscInitStruct.PLL.PLLN            = PLL_MULT_FACTOR;
	RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ            = PLL_DIV_Q_FACTOR;
	RCC_OscInitStruct.PLL.PLLR            = PLL_DIV_R_FACTOR;

	//RCC Clock Config (CPU/AHB/APB bus clks)
	RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK
								     | RCC_CLOCKTYPE_SYSCLK
					                 | RCC_CLOCKTYPE_PCLK1
								     | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	//---------------------------------------- Initialize ----------------------------------------//

	//Init Oscillator
	stat = HAL_RCC_OscConfig(&RCC_OscInitStruct);
	Error_Catch(stat);

	//Init Clock
	stat = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
	Error_Catch(stat);

	return;
}


/**************************************************************************************************/
/** @fcn        static void TIM1_Init(void)
 *  @brief      TIM1 Initialization Function
 *  @details    x
 *
 *  @section	Opens
 *  	Variable init check!
 */
/**************************************************************************************************/
static void TIM1_Init(void) {

	//Locals
	HAL_StatusTypeDef stat = HAL_ERROR;				/* status of HAL operations for review 		  */
	TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig      = {0};

	//Init Variables
	memset(&sClockSourceConfig, 0x00, sizeof(sClockSourceConfig));
	memset(&sMasterConfig,      0x00, sizeof(sMasterConfig));


	//------------------------------------------- Setup ------------------------------------------//

	//Timer Config
	htim1.Instance                    = TIM1;
	htim1.Init.Prescaler              = TIM_PRESCALER_VAL;
	htim1.Init.CounterMode            = TIM_COUNTERMODE_UP;
	htim1.Init.Period                 = TIM_PERIOD_VAL;
	htim1.Init.ClockDivision          = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter      = TIM_REP_VAL;
	htim1.Init.AutoReloadPreload      = TIM_AUTORELOAD_PRELOAD_DISABLE;
	htim1.Channel                     = HAL_TIM_ACTIVE_CHANNEL_1;										//@open 	CATCH!

	//Source Config
	sClockSourceConfig.ClockSource    = TIM_CLOCKSOURCE_INTERNAL;

	//Sync Config
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;


	//---------------------------------------- Initialize ----------------------------------------//

	//Init Timer
	stat = HAL_TIM_Base_Init(&htim1);
	Error_Catch(stat);

	//Init Source Config
	stat = HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
	Error_Catch(stat);

	//Init Sync Config
	stat = HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
	Error_Catch(stat);

	//Enable Timer
	HAL_TIM_Base_Start(&htim1);

	return;
}


/**************************************************************************************************/
/** @fcn        static void USART2_UART_Init(void)
 *  @brief      USART2 Initialization Function
 *  @details    x
 */
/**************************************************************************************************/
static void USART2_UART_Init(void) {

	//Locals
	HAL_StatusTypeDef stat = HAL_ERROR;				/* status of HAL operations for review 		  */

	//------------------------------------------- Setup ------------------------------------------//

	//UART Config
	huart2.Instance          = USART2;
	huart2.Init.BaudRate     = UART_BAUD_RATE_BPS;
	huart2.Init.WordLength   = UART_WORDLENGTH_8B;
	huart2.Init.StopBits     = UART_STOPBITS_1;
	huart2.Init.Parity       = UART_PARITY_NONE;
	huart2.Init.Mode         = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

	//---------------------------------------- Initialize ----------------------------------------//

	//Init UART
	stat = HAL_UART_Init(&huart2);
	Error_Catch(stat);

	return;
}


/**************************************************************************************************/
/** @fcn        static void WWDG_Init(void)
 *  @brief      WWDG Initialization Function
 *  @details    x
 */
/**************************************************************************************************/
#ifdef WDT_IS_WORKING
static void WWDG_Init(void) {

	//Locals
	HAL_StatusTypeDef stat = HAL_ERROR;				/* status of HAL operations for review 		  */

	//------------------------------------------- Setup ------------------------------------------//

	//WDT Config
#ifdef BUG_WHA
	hwwdg.Instance       = WWDG;
	hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
	hwwdg.Init.Window    = WWDG_WINDOW_VAL;
	hwwdg.Init.Counter   = WWDG_CTR_VAL;
	hwwdg.Init.EWIMode   = WWDG_EWI_DISABLE;
#else
	hwwdg.Instance       = WWDG;
	hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
	hwwdg.Init.Window    = 80;
	hwwdg.Init.Counter   = 127;
#endif
	//---------------------------------------- Initialize ----------------------------------------//

	//Init WWDG
	stat = HAL_WWDG_Init(&hwwdg);
	Error_Catch(stat);

	return;
}
#endif


/**************************************************************************************************/
/** @fcn        static void GPIO_Init(void)
 *  @brief      GPIO Initialization Function
 *  @details    x
 */
/**************************************************************************************************/
static void GPIO_Init(void) {

	//Locals
	GPIO_InitTypeDef GPIO_B1_InitStruct  = {0};
	GPIO_InitTypeDef GPIO_PC3_InitStruct = {0};
	GPIO_InitTypeDef GPIO_LD2_InitStruct = {0};

	//Init Variables
	memset(&GPIO_B1_InitStruct,  0x00, sizeof(GPIO_B1_InitStruct));
	memset(&GPIO_PC3_InitStruct, 0x00, sizeof(GPIO_PC3_InitStruct));
	memset(&GPIO_LD2_InitStruct, 0x00, sizeof(GPIO_LD2_InitStruct));


	//------------------------------------------- Setup ------------------------------------------//

	//Enable GPIO Clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	//B1_Pin Config
	GPIO_B1_InitStruct.Pin  = B1_Pin;
	GPIO_B1_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_B1_InitStruct.Pull = GPIO_NOPULL;

	//PC3 Config
	GPIO_PC3_InitStruct.Pin   = GPIO_PIN_3;
	GPIO_PC3_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_PC3_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_PC3_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	//LD2_Pin Config
	GPIO_LD2_InitStruct.Pin   = LD2_Pin;
	GPIO_LD2_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_LD2_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_LD2_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;


	//---------------------------------------- Initialize ----------------------------------------//

	//Conf GPIO Output Levels
	HAL_GPIO_WritePin(GPIOC,         GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,    GPIO_PIN_RESET);

	//Init GPIO Pins
	HAL_GPIO_Init(B1_GPIO_Port,  &GPIO_B1_InitStruct);
	HAL_GPIO_Init(GPIOC,         &GPIO_PC3_InitStruct);
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_LD2_InitStruct);

	return;
}


//************************************************************************************************//
//                                         PUBLIC ROUTINES                                        //
//************************************************************************************************//

/**************************************************************************************************/
/** @fcn        void _printf(const char *str)
 *  @brief      x
 *  @details    x
 *
 *  @param  [in] (const char *) str - EOS terminated string to print
 *
 *	@pre	USART2_UART_Init()
 *
 *  @section    Opens
 *      consider using printf() replacement direct
 */
/**************************************************************************************************/
void _printf(const char *str) {

	//Transmit
	HAL_UART_Transmit(&huart2, (const uint8_t *) str, strlen(str), 1000000);

	return;
}


/**************************************************************************************************/
/** @fcn        void Error_Handler(void)
 *  @brief      This function is executed in case of error occurrence.
 *  @details    no return
 *
 *  @section    Opens
 *      move to new system.c/h
 *
 *  @note   User can add his own implementation to report the HAL error return state
 */
/**************************************************************************************************/
void Error_Handler(void) {

	//Disable
	__disable_irq();

	//Catch
	for(;;);
}


/**************************************************************************************************/
/** @fcn        void Error_Catch(HAL_StatusTypeDef stat)
 *  @brief      Respond to HAL status
 *  @warn		No return on error
 *
 *  @section    Opens
 *      move to new system.c/h
 */
/**************************************************************************************************/
 void Error_Catch(HAL_StatusTypeDef stat) {

	//Catch & Handle
	if(stat != HAL_OK) {
		Error_Handler();
	}

	return;
}
