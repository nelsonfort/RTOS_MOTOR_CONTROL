/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

osThreadId TaskVelRefHandle;
osThreadId TaskControlHandle;
osThreadId TaskPWMHandle;
osThreadId TaskMotorSpeedHandle;
osThreadId TaskUartSendHandle;
osMessageQId QueueUARTSendHandle;
osMutexId MutexEncoderHandle;
osSemaphoreId binSem1Handle;
osSemaphoreId BinSemPWMHandle;
/* USER CODE BEGIN PV */
osMailQDef(QueueUARTSnd, 32, UART_DATA_SEND_t);
osMailQId QueueUARTSndHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
void TaskVelRef_App(void const * argument);
void TaskControl_App(void const * argument);
void TaskPWM_App(void const * argument);
void TaskMotorSpeed_App(void const * argument);
void TaskUartSend_App(void const * argument);

/* USER CODE BEGIN PFP */
void set_PWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint32_t pulse);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float MotorA_speed = 0;
volatile IC_Sig_t MotorA_EncA;
volatile IC_Sig_t MotorA_EncB;
UART_DATA_SEND_t DataSendUart;
volatile uint32_t tickCounter;
volatile Motor_Cont_t MC; //-- Variable principal que aloja las estructuras utilizadas.
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of MutexEncoder */
  osMutexDef(MutexEncoder);
  MutexEncoderHandle = osMutexCreate(osMutex(MutexEncoder));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of binSem1 */
  osSemaphoreDef(binSem1);
  binSem1Handle = osSemaphoreCreate(osSemaphore(binSem1), 1);

  /* definition and creation of BinSemPWM */
  osSemaphoreDef(BinSemPWM);
  BinSemPWMHandle = osSemaphoreCreate(osSemaphore(BinSemPWM), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of QueueUARTSend */
  osMessageQDef(QueueUARTSend, 32, UART_DATA_SEND_t);
  QueueUARTSendHandle = osMessageCreate(osMessageQ(QueueUARTSend), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  QueueUARTSndHandle = osMailCreate(osMailQ(QueueUARTSnd), NULL);      // create mail queue
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TaskVelRef */
  osThreadDef(TaskVelRef, TaskVelRef_App, osPriorityHigh, 0, 128);
  TaskVelRefHandle = osThreadCreate(osThread(TaskVelRef), NULL);

  /* definition and creation of TaskControl */
  osThreadDef(TaskControl, TaskControl_App, osPriorityAboveNormal, 0, 128);
  TaskControlHandle = osThreadCreate(osThread(TaskControl), NULL);

  /* definition and creation of TaskPWM */
  osThreadDef(TaskPWM, TaskPWM_App, osPriorityAboveNormal, 0, 128);
  TaskPWMHandle = osThreadCreate(osThread(TaskPWM), NULL);

  /* definition and creation of TaskMotorSpeed */
  osThreadDef(TaskMotorSpeed, TaskMotorSpeed_App, osPriorityHigh, 0, 128);
  TaskMotorSpeedHandle = osThreadCreate(osThread(TaskMotorSpeed), NULL);

  /* definition and creation of TaskUartSend */
  osThreadDef(TaskUartSend, TaskUartSend_App, osPriorityBelowNormal, 0, 128);
  TaskUartSendHandle = osThreadCreate(osThread(TaskUartSend), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2400;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MotorA_INA_Pin|MotorA_INB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_DIR_Pin */
  GPIO_InitStruct.Pin = PIN_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PIN_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin MotorA_INA_Pin MotorA_INB_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|MotorA_INA_Pin|MotorA_INB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void set_PWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint32_t pulse)
{
    HAL_TIM_PWM_Stop(&timer,channel);
    TIM_OC_InitTypeDef sConfigOC;
    timer.Init.Period = period;
    HAL_TIM_PWM_Init(&timer);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&timer,&sConfigOC,channel);

    HAL_TIM_PWM_Start(&timer,channel);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(HAL_GPIO_ReadPin(PIN_DIR_GPIO_Port,PIN_DIR_Pin ) == 1)
		{
			MotorA_EncA.direction = -1;
		}
		else
		{
			MotorA_EncA.direction = 1;
		}

		if(MotorA_EncA.Is_First_Captured==0)
		{
			MotorA_EncA.IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			MotorA_EncA.Is_First_Captured = 1;
		}
		else if(MotorA_EncA.Is_First_Captured)
		{
			MotorA_EncA.IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if(MotorA_EncA.IC_Value2 > MotorA_EncA.IC_Value1)
			{
				MotorA_EncA.Period = MotorA_EncA.IC_Value2 - MotorA_EncA.IC_Value1;
			}
			else
			{
				MotorA_EncA.Period = MotorA_EncA.IC_Value2+65535 - MotorA_EncA.IC_Value1;
			}
			//MotorA_EncA.Frequency = HAL_RCC_GetPCLK2Freq()/(htim->Init.Prescaler*MotorA_EncA.Period);
			//MotorA_EncA.CalculationOK = 1;
			MotorA_EncA.Is_First_Captured = 0;

		}
		MotorA_EncA.cont++;
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if(MotorA_EncB.Is_First_Captured==0)
		{
			MotorA_EncB.IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			MotorA_EncB.Is_First_Captured = 1;
		}
		else if(MotorA_EncB.Is_First_Captured)
		{
			MotorA_EncB.IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			if(MotorA_EncB.IC_Value2 > MotorA_EncB.IC_Value1)
			{
				MotorA_EncB.Period = MotorA_EncB.IC_Value2 - MotorA_EncB.IC_Value1;
			}
			else
			{
				MotorA_EncB.Period = MotorA_EncB.IC_Value2+65535 - MotorA_EncB.IC_Value1;
			}
			// Freq = (FreqCKL/(PreScaler*Nticks))
			//MotorA_EncB.Frequency = HAL_RCC_GetPCLK2Freq()/(htim->Init.Prescaler*MotorA_EncB.Period);
			//MotorA_EncB.CalculationOK = 1;
			MotorA_EncB.Is_First_Captured = 0;

		}
		MotorA_EncB.cont++;
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(B1_Pin == GPIO_Pin)
	{
		osSemaphoreRelease(binSem1Handle);

	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_TaskVelRef_App */
/**
  * @brief  Function implementing the TaskVelRef thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TaskVelRef_App */
void TaskVelRef_App(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  MC.ready = 1;
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  for(;;)
  {
	  if(!MC.ready)
	  {
		  osSemaphoreWait(binSem1Handle, osWaitForever);
	  }
	  MC.ready=1;
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  for(uint8_t cont = 0; cont <=3 ; cont++ )
	  {
		MC.speedRef = (300+cont*50);
		osDelay(3000);
	  }
	  for(int8_t cont = 2; cont >=1 ; cont-- )
	  {
		MC.speedRef = (300+cont*50);
		osDelay(3000);
	  }
	  //osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TaskControl_App */
/**
* @brief Function implementing the TaskControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskControl_App */
void TaskControl_App(void const * argument)
{
  /* USER CODE BEGIN TaskControl_App */
	//-- Initialize structure
	MC.MotorA_speed = 0;
	MC.MotorB_speed = 0;
	for(int8_t cont = 0;cont<PID_COEF_LEN;cont++)
	{
		MC.PID_MA.x_n[cont] = 0;
		MC.PID_MA.y_n[cont] = 0;
	}
	MC.speedRef=0;
	MC.errorA=0;
	MC.errorB=0;
	//-- Seteo los coeficientes del PID diseñado.
	//-- Para Ts 1ms
	//MC.PID_MA.a[0] = 1;
	//MC.PID_MA.a[1] = 1.845;
	//MC.PID_MA.a[2] = -0.845;
	//MC.PID_MA.b[0] = 0.125;
	//MC.PID_MA.b[1] = 0.0075;
	//MC.PID_MA.b[2] = -0.117;
	//-- Para Ts 5ms
	MC.PID_MA.K = 0.5;
	MC.PID_MA.a[0] = 1;
	MC.PID_MA.a[1] = 1.296;
	MC.PID_MA.a[2] = -0.296;
	MC.PID_MA.b[0] = 0.599*MC.PID_MA.K;
	MC.PID_MA.b[1] = 0.1553*MC.PID_MA.K;
	MC.PID_MA.b[2] = -0.444*MC.PID_MA.K;
	osDelay(10);

  /* Infinite loop */
  for(;;)
  {
	  if(MC.ready)
	  {
		  osDelay(5);
		  //-- Armo la señal de error
		  MC.errorA = MC.speedRef - MC.MotorA_speed*60;

		  //-- Realizo el corrimiento de los coeficientes del PID
		  for(int8_t i = PID_COEF_LEN-2 ; i>=0 ; i--)
		  {
			  MC.PID_MA.x_n[i+1] = MC.PID_MA.x_n[i];
			  MC.PID_MA.y_n[i+1] = MC.PID_MA.y_n[i];
		  }
		  MC.PID_MA.x_n[0] = MC.errorA;
		  /*MC.PID_MA.y_n[0] = 0;
		  MC.PID_MA.y_n[0] += MC.PID_MA.b[0]*MC.PID_MA.x_n[0] ;
		  for(uint8_t i = 1 ; i<=PID_COEF_LEN-1 ; i++)
		  {
			  MC.PID_MA.y_n[0] += MC.PID_MA.b[i]*MC.PID_MA.x_n[i] + MC.PID_MA.a[i]*MC.PID_MA.y_n[i];
		  }*/
		  MC.PID_MA.y_n[0] = MC.PID_MA.b[0]*MC.PID_MA.x_n[0] +
				  	  	  	  MC.PID_MA.b[1]*MC.PID_MA.x_n[1] +
							  MC.PID_MA.b[2]*MC.PID_MA.x_n[2] +
							  MC.PID_MA.a[1]*MC.PID_MA.y_n[1] +
							  MC.PID_MA.a[2]*MC.PID_MA.y_n[2];

		  //-- Libero la tarea encargada de activar el PWM que acciona el motor..
		  osSemaphoreRelease(BinSemPWMHandle);

	  }
	  osDelay(1);
  }
  /* USER CODE END TaskControl_App */
}

/* USER CODE BEGIN Header_TaskPWM_App */
/**
* @brief Function implementing the TaskPWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskPWM_App */
void TaskPWM_App(void const * argument)
{
  /* USER CODE BEGIN TaskPWM_App */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	set_PWM(htim3, TIM_CHANNEL_1, 3600, (uint16_t) 0); // -- PWM desactivado
	HAL_GPIO_WritePin(MotorA_INA_GPIO_Port, MotorA_INA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MotorA_INB_GPIO_Port, MotorA_INB_Pin, GPIO_PIN_RESET);
	MC.pwmMotor = 0;

  /* Infinite loop */
  for(;;)
  {
	osSemaphoreWait(BinSemPWMHandle, osWaitForever);
	MC.pwmMotor = MC.PID_MA.y_n[0];
	if(MC.pwmMotor<=0)
	{
		HAL_GPIO_WritePin(MotorA_INA_GPIO_Port, MotorA_INA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(MotorA_INB_GPIO_Port, MotorA_INB_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(MotorA_INA_GPIO_Port, MotorA_INA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MotorA_INB_GPIO_Port, MotorA_INB_Pin, GPIO_PIN_SET);
	}
	set_PWM(htim3, TIM_CHANNEL_1, 3600, (uint32_t) fabs((3600*MC.pwmMotor/450)  ) ); ///7.5 --Vmax 450rpm PWM

  }
  /* USER CODE END TaskPWM_App */
}

/* USER CODE BEGIN Header_TaskMotorSpeed_App */
/**
* @brief Function implementing the TaskMotorSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskMotorSpeed_App */
void TaskMotorSpeed_App(void const * argument)
{
  /* USER CODE BEGIN TaskMotorSpeed_App */
	uint8_t contAux = 0;
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	UART_DATA_SEND_t *dataPtr;
	MotorA_EncA.direction = 1;
  /* Infinite loop */
  for(;;)
  {
	  //osMutexWait(MutexEncoderHandle, osWaitForever );
		if((contAux == MotorA_EncA.cont))
		{
			DataSendUart.MotorA_speed = 0;
			MC.MotorA_speed = 0;
			DataSendUart.time_stamp = (uint32_t) tickCounter;
			DataSendUart.Period = 0;
			DataSendUart.Frequency = 0;
		}
		else{
			// Freq = (FreqCLK/(PreScaler*Nticks))
			// Speed = Freq/Encoder_pulses (Speed in the shaft)
			DataSendUart.MotorA_speed = (float) HAL_RCC_GetPCLK2Freq()/(htim1.Init.Prescaler*MotorA_EncA.Period*ENCODER_SHAFT_CPR);
			//DataSendUart.MotorA_speed = DataSendUart.MotorA_speed;//*MotorA_EncA.direction;
			MC.MotorA_speed = DataSendUart.MotorA_speed;
			DataSendUart.time_stamp = (uint32_t) tickCounter;
			DataSendUart.Period = (uint32_t) MotorA_EncA.Period;
			DataSendUart.Frequency = (float) MotorA_EncA.Frequency;
			contAux = MotorA_EncA.cont;
		}

		//osMutexRelease(MutexEncoderHandle);
		//dataPtr = osMailAlloc(QueueUARTSndHandle, millis5);
		dataPtr = osMailAlloc(QueueUARTSndHandle, osWaitForever );
		if(dataPtr != NULL)
		{
			dataPtr->MotorA_speed = DataSendUart.MotorA_speed;
			dataPtr->time_stamp = DataSendUart.time_stamp;
			dataPtr->reference = (float) MC.speedRef;
			if(osMailPut(QueueUARTSndHandle, dataPtr) != osOK)
			{
				while(1);
			}
		}
		osDelay(1);
  }
  /* USER CODE END TaskMotorSpeed_App */
}

/* USER CODE BEGIN Header_TaskUartSend_App */
/**
* @brief Function implementing the TaskUartSend thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskUartSend_App */
void TaskUartSend_App(void const * argument)
{
  /* USER CODE BEGIN TaskUartSend_App */
  osEvent dataSEND;
  UART_DATA_SEND_t *dataPtr;
  uint8_t a=8, b=16, c=32, d=64 ;
  /* Infinite loop */
  for(;;)
  {
	  dataSEND = osMailGet(QueueUARTSndHandle, osWaitForever);
	  if(dataSEND.status == osEventMail){
		  dataPtr = dataSEND.value.p;
		  HAL_UART_Transmit(&huart2, (uint8_t *)&a, sizeof(uint8_t), osWaitForever);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&b, sizeof(uint8_t), osWaitForever);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&c, sizeof(uint8_t), osWaitForever);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&d, sizeof(uint8_t), osWaitForever);
		  HAL_UART_Transmit(&huart2, (uint8_t *)dataPtr, sizeof(UART_DATA_SEND_t), osWaitForever);
		  osMailFree(QueueUARTSndHandle, dataPtr); // Free a memory block from a mail.
	  }
  }
  /* USER CODE END TaskUartSend_App */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  //osMutexWait(MutexEncoderHandle, osWaitForever );
  tickCounter++;
  //osMutexRelease(MutexEncoderHandle);
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
