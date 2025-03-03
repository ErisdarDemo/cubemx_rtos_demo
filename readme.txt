@brief 		CubeMX FreeRTOS
@details 	NUCLEO-F446RE (STM32F446RE)
@auth 		Justin Reina
@date 		3/2/25


@section 	Notice

	Re-organization of repository servers (GitHub: Work, GitLab: Personal) in progress and repository kept offline until ready. Taking several years of content generation and cleaning this up!

	Note that some of these repositories, e.g 'regression_test' or 'unit_test' in the new ErisdarTest organization are just placeholders, coming next!
	
@section 	Opens
	Import expanded ProductMap demo
	post
	bring it live! Copy/Temp/Post Confluence to WWW



	New ESP project
	import STM demo
	bring it live!

@section 	Procedure
	
	Open STM32CubeIDE (using 1.17.0 on Win11 Pro v24H2)

	File -> New -> STM32 Project
	
	Board Selector -> NUCLEO-F446RE 
		Name: "CubeMX_RTOS"
		Language: C++
		Copy only the necessary library files
			*sync repo! STM32Cube\Repository(#FD42)
		Periph Default: Y
	Configure IOC
	
	
@section 	IOC Configure
	GPIO:      +PC3 (Out for CN7.37)
	WWDG.Mode: Activated
	TIM1: Internal Clock source, 
	USART2: 115200 bps
	FreeRTOS: CMSIS_V2 [1]
		Premption:  T
		Tasks:      4 (sysTask, dataTask, dispTask ctrlTask, 
		Timers:		osTimer
		Semaphores:	ctrlSem(T/F), ctrSem(10)
		Mutex:		dataMutex
		Events:		dataStore
		
		
@section 	Opens
	RTOS
		Threads: x
		Queue:	 x
		Sem:	 x
		
	Consideration for FatFs


@section 	Reference
	1. https://arm-software.github.io/CMSIS_6/latest/RTOS2
	
@note	Defaults used unless otherwise specified
