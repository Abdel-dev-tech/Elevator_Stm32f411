/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

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
 I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

osThreadId prvLightTrafficHandle;
osThreadId prvUserButtonHandle;
osThreadId LightSet1Handler;
osThreadId LightSet2Handler;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void StartLightSet1tTask(void const * argument);
void StartLightSet2Task(void const * argument);
void prvUserButtonTask(void const * argument);
void Init_OnBoard_LEDs(void);
void configure_Button(void);
void configure_Commands_Button(void);
uint8_t Read_User_Button(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
static SemaphoreHandle_t semLightControl;
static SemaphoreHandle_t semCausingDeadlock;
GPIO_PinState state, UserButtonState_PE10=0; // Floor3 //PE10
GPIO_PinState  UserButtonState_PE8; // Floor2 Haut //PE8
GPIO_PinState  UserButtonState_PB2; // Floor2 bas //PB2
GPIO_PinState  UserButtonState_PB0; // Floor2 bas //PB0
GPIO_PinState  UserButtonState_PA2; // Floor2 from Elevator
GPIO_PinState  UserButtonState_PA0; // Floor1 from Elevator
GPIO_PinState  UserButtonState_PC2; // Floor0 from Elevator

typedef enum ElevatorState_Def{
	    floor1=1,
		floor2,
		floor3,
		moving
}ElevatorState_Def;

static ElevatorState_Def ElevatorState=floor1;

/*Elevator Commands*/
typedef enum ElevatorCommands_Def{
	    floor1_Button=1,
		floor2_down_Button,
		floor2_up_Button,
		floor3_Button,
		elevator_toFloor1_Btn,
		elevator_toFloor2_Btn,
		elevator_toFloor3_Btn
}ElevatorCommands_Def;

/*Queue FIFO to store the commands for the elevator*/
QueueHandle_t xElevatorQueue;
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
//  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  Init_OnBoard_LEDs();

//  MX_I2C1_Init();
//  MX_I2S2_Init();
//  MX_I2S3_Init();
//  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  semLightControl    = xSemaphoreCreateBinary();
  semCausingDeadlock = xSemaphoreCreateBinary();
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of prvLightTraffic */

  /*Create the Elevator queue*/
  xElevatorQueue= xQueueCreate( 100, sizeof( uint8_t ) );
  if( xElevatorQueue != NULL )
  {
      /* Queue was not created and must not be used. */

		  osThreadDef(prvLightTraffic, StartDefaultTask, osPriorityNormal, 0, 128);
		  prvLightTrafficHandle = osThreadCreate(osThread(prvLightTraffic), NULL);

		  osThreadDef(prvUserButton, prvUserButtonTask, osPriorityNormal, 0, 128);
		  prvUserButtonHandle = osThreadCreate(osThread(prvUserButton), NULL);

  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin PD5 */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the prvLightTraffic thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */

  /* USER CODE BEGIN 5 */
	Init_OnBoard_LEDs();
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_6,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
  int commandNumber;
  ElevatorCommands_Def commands;
  /* Infinite loop */
  for(;;)
  {
       // The elevator is in floor 1
	  if(ElevatorState==floor1)
	  {
		  /*Turn On LED of the floor 1*/
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);

		  if(commands==floor2_down_Button ||commands==floor2_up_Button ||(commands==elevator_toFloor2_Btn)){
			  //turn on the blue led if floor2
			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
			  /*Turn Off the red LED of floor 1,2,3*/
			  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
			  	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
			  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
			  ElevatorState=moving;
			  HAL_Delay(4000);
			  //turn on the red led of floor2
			  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
			  ////BUZZER//////
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
					  HAL_Delay(500);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
					  HAL_Delay(500);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
					  HAL_Delay(500);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			  ////////////////
			  //turn off the blue led if floor2
			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
			  ElevatorState=floor2;
		  }
		  else if((commands==floor3_Button)||(commands==elevator_toFloor3_Btn)){
			  //turn on the blue led of floor3
			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET);
			  /*Turn Off the red LED of floor 1,2,3*/
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
			  ElevatorState=moving;
			  HAL_Delay(6000);
			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
			  ////BUZZER//////
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
					  HAL_Delay(500);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
					  HAL_Delay(500);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
					  HAL_Delay(500);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
					  HAL_Delay(500);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
					  HAL_Delay(500);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
			  ////////////////
			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET);
			  ElevatorState=floor3;
		  }
		  /*Wait for 3 seconds to get elevator empty*/
		  HAL_Delay(3000);
	  }
	  else if((ElevatorState==floor2)) // The elevator is in floor 2
	  {
		  /*Turn On LED of the floor 2*/
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);

		  if(commands==floor1_Button ||(commands==elevator_toFloor1_Btn)){
		  			  //turn on the blue led if floor1
		  			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		  			  /*Turn Off the red LED of floor 1,2,3*/
		  			  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		  			  	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
		  			  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
		  			  ElevatorState=moving;
		  			  HAL_Delay(4000);
		  			  //turn on the red led of floor1
		  			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
		  			  ////BUZZER//////
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		  					  HAL_Delay(500);
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		  			  ////////////////
		  			  //turn off the blue led if floor1
		  			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
		  			  ElevatorState=floor1;
		  		  }
		  		  else if((commands==floor3_Button)||(commands==elevator_toFloor3_Btn)){
		  			  //turn on the blue led of floor3
		  			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET);
		  			  /*Turn Off the red LED of floor 1,2,3*/
		  				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		  				  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
		  				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
		  			  ElevatorState=moving;
		  			  HAL_Delay(4000);
		  			  /*Turn on the blue LED**/
		  			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
		  			  ////BUZZER//////
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		  					  HAL_Delay(500);
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		  					  HAL_Delay(500);
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		  					  HAL_Delay(500);
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		  					  HAL_Delay(500);
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		  					  HAL_Delay(500);
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		  			  ////////////////
		  			  /*Turn off the blue LED**/
		  			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET);
		  			  ElevatorState=floor3;
		  		  }
		  /*Wait for 3 seconds to get elevator empty*/
		  HAL_Delay(3000);
	  }
	  else // The elevator is in floor 3
	  {
		  /*Turn On LED of the floor 3*/
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

		  if(commands==floor1_Button ||(commands==elevator_toFloor1_Btn)){
		  			  //turn on the blue led if floor1
		  			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
		  			  /*Turn Off the red LED of floor 1,2,3*/
		  			  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
		  			  	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
		  			  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
		  			  ElevatorState=moving;
		  			  HAL_Delay(6000);
		  			  //turn on the red led of floor1
		  			  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
		  			  ////BUZZER//////
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
		  					  HAL_Delay(500);
		  					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
		  			  ////////////////
		  			  //turn off the blue led if floor1
		  			  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
		  			  ElevatorState=floor1;
		  		  }
			  if(commands==floor2_down_Button ||commands==floor2_up_Button ||(commands==elevator_toFloor2_Btn)){
				  //turn on the blue led if floor2
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
				  /*Turn Off the red LED of floor 1,2,3*/
					  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
				  ElevatorState=moving;
				  HAL_Delay(4000);
				  //turn on the red led of floor2
				  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
				  ////BUZZER//////
						  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
						  HAL_Delay(500);
						  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
						  HAL_Delay(500);
						  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
						  HAL_Delay(500);
						  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
				  ////////////////
				  //turn off the blue led if floor2
				  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
				  ElevatorState=floor2;
			  }

			  /*Wait for 3 seconds to get elevator empty*/
			  HAL_Delay(3000);
	  }
	  xQueueReceive( xElevatorQueue,&(commandNumber), portMAX_DELAY );
	  commands=commandNumber;
//		/*turn on green light of the first Set*/
//		//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
//		if(UserButtonState_PE10==1)
//		{
//			//HAL_GPIO_TogglePin();
//			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
//		    UserButtonState_PE10=0;
//		}
//		else if(UserButtonState_PE8==1)
//		{
//			//HAL_GPIO_TogglePin();
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
//		    UserButtonState_PE8=0;
//		}
//		else if(UserButtonState_PB2==1)
//		{
//			//HAL_GPIO_TogglePin();
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_4);
//			UserButtonState_PB2=0;
//		}
//		else if(UserButtonState_PB0==1)
//		{
//			//HAL_GPIO_TogglePin();
//			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
//			UserButtonState_PB0=0;
//		}
//		else if(UserButtonState_PA2==1)
//		{
//			//HAL_GPIO_TogglePin();
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_10);
//			UserButtonState_PA2=0;
//		}
//		else if(UserButtonState_PA0==1)
//		{
//			//HAL_GPIO_TogglePin();
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
//			UserButtonState_PA0=0;
//		}
//		else if(UserButtonState_PC2==1)
//		{
//			//HAL_GPIO_TogglePin();
//			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
//			HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
//			UserButtonState_PC2=0;
//		}
//		else
//			osDelay(200);


  }
  /* USER CODE END 5 */
}


/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the prvLightTraffic thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void prvUserButtonTask(void const * argument)
{
  /* init code for USB_HOST */
	  configure_Commands_Button(); // call Push button GPIO pins initialization function
	  Init_OnBoard_LEDs();
  /* USER CODE BEGIN 5 */
   uint8_t userKeyState = 0;
   uint8_t userbutton=0;
  /* Infinite loop */
  for(;;)
  {

	  state = Read_User_Button(GPIOE, GPIO_PIN_10); // read state of push button and save it in "state" variable
	  if(state==1){
		  userKeyState=floor3_Button;
		  xQueueSend( xElevatorQueue,&userKeyState,( TickType_t ) 10 );/*The elevator is request from the floor 3 to go down*/
		  //UserButtonState_PE10=1;// Floor3
	  }

	  state = Read_User_Button(GPIOE, GPIO_PIN_8); // read state of push button and save it in "state" variable
	  if(state==1){
		  userKeyState=floor2_up_Button;
		  xQueueSend( xElevatorQueue,&userKeyState,( TickType_t ) 10 );/*The elevator is request from the floor 2 to go up*/
		 // UserButtonState_PE8=1;// Floor2
	  }

	  state = Read_User_Button(GPIOE, GPIO_PIN_12); // read state of push button and save it in "state" variable
	  if(state==1){
		  userKeyState=floor2_down_Button;
		  xQueueSend( xElevatorQueue,&userKeyState,( TickType_t ) 10 );/*The elevator is request from the floor 2 to go dpwn*/
		  //UserButtonState_PB2=1;// Floor1
	  }

	  state = Read_User_Button(GPIOB, GPIO_PIN_0); // read state of push button and save it in "state" variable
	  if(state==1){
		  userKeyState=floor1_Button;
		  xQueueSend( xElevatorQueue,&userKeyState,( TickType_t ) 10 );/*The elevator is request from the floor 1*/
		  // UserButtonState_PB0=1;// Floor1
	  }


	  state = Read_User_Button(GPIOA, GPIO_PIN_2); // read state of push button and save it in "state" variable
	  if(state==1){
		  userKeyState=elevator_toFloor3_Btn;
		  xQueueSend( xElevatorQueue,&userKeyState,( TickType_t ) 10 );/*The elevator is commanded to go to the floor3*/
		  //UserButtonState_PA2=1;// Elevator to floor3
	  }


	  state = Read_User_Button(GPIOA, GPIO_PIN_0); // read state of push button and save it in "state" variable
	  if(state==1){
		  userKeyState=elevator_toFloor2_Btn;
		  xQueueSend( xElevatorQueue,&userKeyState,( TickType_t ) 10 );/*The elevator is commanded to go to the floor2*/
		 //UserButtonState_PA0=1;// Elevator to floor2

	  }

	  state = Read_User_Button(GPIOC, GPIO_PIN_2); // read state of push button and save it in "state" variable
	  if(state==1){
		  userKeyState=elevator_toFloor1_Btn;
		  xQueueSend( xElevatorQueue,&userKeyState,( TickType_t ) 10 );/*The elevator is commanded to go to the floor1*/
		 		  //UserButtonState_PC2=1;// Elevator to floor1
	  }


  }
  /* USER CODE END 5 */
}


void processCommand(ElevatorState_Def state,ElevatorCommands_Def commands)
{
	switch(state)
	{
	case floor1:
		{

			break;
		}
	case floor2:
		{
			break;
		}
	case floor3:
		{
			break;
		}
	}
}
//	  userbutton = Read_User_Button(GPIOE, GPIO_PIN_10); // read state of push button and save it in "state" variable
//	  if(userbutton==1)
//	  {
//		  userKeyState++;
//		  if(userKeyState>1)
//			  userKeyState=0;
//	  }
//
//	  if(userbutton == 1)
//	  {
//		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//	  }
//	  else
//	  {
//		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//	  }

/**
  * @brief  Function implementing the prvLightTraffic thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartLightSet1tTask(void const * argument)
{
  /* init code for USB_HOST */
//  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
	  Init_OnBoard_LEDs();
	  Init_OnBoard_LEDs();
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);

  /* Infinite loop */
  for(;;)
  {
	/*turn on RED light of the first Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);

    /*Wait for the second light notification, after 1 s*/
	xSemaphoreTake(semLightControl, portMAX_DELAY);

	/*turn on Green light of the first Set and turn the red light*/
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_SET);

	 /*Wait for the second light notification, after 4 s*/
	xSemaphoreTake(semLightControl, portMAX_DELAY);

	/*turn off Green light of the second Set*/
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
	/*turn on yellow light of the first Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);

	 /*Wait for the second light notification, after 1.5 s*/
	xSemaphoreTake(semLightControl, portMAX_DELAY);

	/*turn off yellow light of the first Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
	/*turn on RED light of the first Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,GPIO_PIN_RESET);

  }
  /* USER CODE END 5 */
}



/**
  * @brief  Function implementing the prvLightTraffic thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartLightSet2Task(void const * argument)
{
  /* init code for USB_HOST */
//  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_RESET);
  /* Infinite loop */
  int i=0;
  for(;;)
  {
	  /*After the three first iterations, this task will take the semCausingDeadlock semaphore, then enter the block state
	   * while there no task that will release the semaphore, So this task will state blocked; While this task is the one master
	   * responsible for releasing the semaphore semLightControl to synchronize the StartLightSet1tTask task, then
	   * this last task will stay blocked in first xSemaphoreTake also ===> Total Deadlock*/
	  if(i==3)
		xSemaphoreTake(semCausingDeadlock, portMAX_DELAY);

	/*turn on RED light of the first Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
	osDelay(1000);
//====> Semaphore to wake up green light and turn off red light of the first light set
	xSemaphoreGive(semLightControl);

	/*keep turn on red light of the second Set*/
	osDelay(4000);

	/*turn off Green light of the second Set*/

	//====> Semaphore to wake up yellow light and turn off green light of the first light set
	 xSemaphoreGive(semLightControl);

	 /*keep turn on red light of the second Set*/
	osDelay(1500);

	//====> Semaphore to wake up red light of the first light set
	 xSemaphoreGive(semLightControl);
	 /*keep turn on red light of the second Set for one more second*/
	osDelay(1000);

	/*turn off RED light of the second Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);

	/*turn on green light of the second Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
	osDelay(4000);
	/*turn off green light of the second Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);


	/*turn on yellow light of the second Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
	osDelay(1500);
	/*turn off yellow light of the second Set*/
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);

	i++;

  }
  /* USER CODE END 5 */
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


void Init_OnBoard_LEDs(void)
{
	 __HAL_RCC_GPIOD_CLK_ENABLE();
	 __HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef BoardLEDs, USB_RED_LED, BoardLEDPC;

	BoardLEDs.Mode = GPIO_MODE_OUTPUT_PP;
	USB_RED_LED.Mode = GPIO_MODE_OUTPUT_PP;
	BoardLEDPC.Mode = GPIO_MODE_OUTPUT_PP;

	BoardLEDs.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_6;
	BoardLEDPC.Pin = GPIO_PIN_4|GPIO_PIN_0;
	USB_RED_LED.Pin = GPIO_PIN_9|GPIO_PIN_4|GPIO_PIN_6;

	HAL_GPIO_Init(GPIOD, &BoardLEDs);
	HAL_GPIO_Init(GPIOA, &USB_RED_LED);
	HAL_GPIO_Init(GPIOC, &BoardLEDPC);
}

/* Function to configure PA0 pin of as adigital input pin */
void configure_Button(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE(); //Enable clock to GPIOA
	GPIO_InitTypeDef PushButton;  // declare a variable of type struct GPIO_InitTypeDef
	PushButton.Mode = GPIO_MODE_INPUT; // set pin mode to input
	PushButton.Pin = GPIO_PIN_0;  // select pin PA0 only
	PushButton.Pull = GPIO_NOPULL; // set no internal pull-up or pull-down resistor
	HAL_GPIO_Init(GPIOA, &PushButton); //  initialize PA0 pins by passing port name and address of PushButton struct
}

void configure_Commands_Button(void)
{
	__HAL_RCC_GPIOE_CLK_ENABLE(); //Enable clock to GPIOE
	__HAL_RCC_GPIOB_CLK_ENABLE(); //Enable clock to GPIOB
	__HAL_RCC_GPIOA_CLK_ENABLE(); //Enable clock to GPIOA
	__HAL_RCC_GPIOC_CLK_ENABLE(); //Enable clock to GPIOC

	/*Button to go up in floor 2 and button to go down in floor3*/
	GPIO_InitTypeDef PushButtonE;  // declare a variable of type struct GPIO_InitTypeDef
	PushButtonE.Mode = GPIO_MODE_INPUT; // set pin mode to input
	PushButtonE.Pin = GPIO_PIN_10|GPIO_PIN_8||GPIO_PIN_12;  // select pin PE10 only
	PushButtonE.Pull = GPIO_NOPULL; // set no internal pull-up
	HAL_GPIO_Init(GPIOE, &PushButtonE); //  initialize PE10 pins by passing port name and address of PushButton struct

	/*Button to go down in floor 2*/
	GPIO_InitTypeDef PushButtonB;  // declare a variable of type struct GPIO_InitTypeDef
	PushButtonB.Mode = GPIO_MODE_INPUT; // set pin mode to input
	PushButtonB.Pin =  GPIO_PIN_2|GPIO_PIN_0;  // select pin PB0 and PB2
	PushButtonB.Pull = GPIO_NOPULL; // set no internal pull-up
	HAL_GPIO_Init(GPIOB, &PushButtonB); //  initialize PE10 pins by passing port name and address of PushButton struct

	/*Button to choose the floor from the elevator floor2 and 3r*/
	GPIO_InitTypeDef PushButtonA;  // declare a variable of type struct GPIO_InitTypeDef
	PushButtonA.Mode = GPIO_MODE_INPUT; // set pin mode to input
	PushButtonA.Pin =  GPIO_PIN_2|GPIO_PIN_0;  // select pin PB0 and PB2
	PushButtonA.Pull = GPIO_NOPULL; // set no internal pull-up
	HAL_GPIO_Init(GPIOA, &PushButtonA); //  initialize PE10 pins by passing port name and address of PushButton struct

	/*Button to choose the floor 1 from the elevator*/
	GPIO_InitTypeDef PushButtonC;  // declare a variable of type struct GPIO_InitTypeDef
	PushButtonC.Mode = GPIO_MODE_INPUT; // set pin mode to input
	PushButtonC.Pin =  GPIO_PIN_2;  // select pin PB0 and PB2
	PushButtonC.Pull = GPIO_NOPULL; // set no internal pull-up
	HAL_GPIO_Init(GPIOC, &PushButtonC); //  initialize PE10 pins by passing port name and address of PushButton struct
}

uint8_t Read_User_Button(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	uint32_t tick,ElapsedTime;//
	if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))
	{
		 tick=HAL_GetTick();
		 while((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)==GPIO_PIN_SET) && (HAL_GetTick()-tick<=200));
		 ElapsedTime=HAL_GetTick()-tick;
		 if(ElapsedTime>100)
			 return 1;
		 else
			 return 0;

	}
	else
		return 0;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
