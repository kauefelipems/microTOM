/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include "state_machine.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Machine Constants
#define N_CHANNELS 16
#define FREQUENCY 10000
#define	AMPLITUDE 1.5
#define MIN_GAIN 0
#define MAX_GAIN 1.1

//UART Constants
#define UART_TIMEOUT 1000 //ms
#define ADC_BUFF_SIZE 513 //Data Buffer
#define COMM_BUFF_SIZE 5  //Command Buffer
#define DIAG_BUFF_SIZE 8 //Diagnostic Buffer
#define TERMINATOR 2573       //CR/LF Terminator in 16 bits

// State Machine Constant
#define ENTRY_STATE 0 	    /*defines entry state (allows for further change without
								modifying the main())*/

// CHANNEL SELECTION
#define SDATA_PORT GPIOA
#define SCLK_PORT GPIOB
#define PCLK_PORT GPIOE

typedef enum
{
  SWITCH_BEGIN = 0U,
  SWITCH_TRANSFER,
  SWITCH_LOAD,
  SWITCH_END
}Switch_State;

//Values of BSRR for the switching system
#define SWITCH_DATA_ON (uint32_t)(((uint32_t)GPIO_PIN_0))
#define SWITCH_DATA_OFF (uint32_t)(((uint32_t)GPIO_PIN_0) << 16)
#define SWITCH_SIZE 256

//PCLK Clock Length
#define PCLK_DELAY 1 //[clock cycles]

// Channel Stabilization Time
#define CHANNEL_STABILIZATION_TIME 72 // [clock cycles]

// PGA
// Initial PGA Voltage (255 is 1.1V and 24 is 0.1035V)
#define PGA_MIN_VOLTAGE 24 // [bits]
#define PGA_MAX_VOLTAGE 255 // [bits]


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//GET and SET the Timer Repetition Counter Register


//DAC Macros
#define __DAC_VOLTAGE2BIT(__VOLTAGE__) (uint16_t)__VOLTAGE__*4095/(765)


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

// Private Callback
void Switch_Cplt_Callback(DMA_HandleTypeDef *hdma);

// State Functions BEGIN
Actions_TypeDef comm_wait_state(void);
Actions_TypeDef g_sel_state(void);
Actions_TypeDef ch_sel_state(void);
Actions_TypeDef meas_state(void);
Actions_TypeDef error_state(void);
Actions_TypeDef diag_state(void);
Actions_TypeDef exc_state(void);

// State Functions END

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Interface Commands
uint8_t comm[COMM_BUFF_SIZE]; 									//command

//Machine Values
uint8_t gain_val;
uint8_t channels_value[4] = {0,0,0,0};

//Voltage Measurements
uint16_t adc_buffer[ADC_BUFF_SIZE]; 									//ADC Measurements
unsigned char meas_uart[2*ADC_BUFF_SIZE]; 								//UART Buffer

//State Function Pointer (need to be synchronized with States_TypeDef)
State_FunctionsTypeDef state_func_ptr[] = {comm_wait_state, g_sel_state, ch_sel_state, meas_state, error_state, diag_state, exc_state};

//Channel Selection
uint32_t ch_data[256];
uint16_t ch_counter = 0;
Switch_State sw_flag = SWITCH_BEGIN;

//Current Time
uint32_t entry_time = 0;

//Current PGA DAC Value
uint32_t dac_value = __DAC_VOLTAGE2BIT(PGA_MIN_VOLTAGE);

//Flow Locks
HAL_LockTypeDef uart_cplt_lck = HAL_LOCKED; //UART Receiving Complete Transfer

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
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Set State Machine Variables:
  States_TypeDef state = ENTRY_STATE;
  Actions_TypeDef action;
  State_FunctionsTypeDef state_func;

  //ADC Calibration Function
  if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
	  Error_Handler();

  //GAIN_DAC Start and set value to 0.1 V
	if((HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value) != HAL_OK) ||
		  (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) != HAL_OK)
	  Error_Handler();


  //UART Terminator
	adc_buffer[ADC_BUFF_SIZE - 1] = TERMINATOR;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	state_func = state_func_ptr[state];
	action = state_func();
	state = ST_nextstate(state, action);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 35;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 17;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 35;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 359;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 999;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 71;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 35;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_ODD;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDATA_GPIO_Port, SDATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PCLK_GPIO_Port, PCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SDATA_Pin */
  GPIO_InitStruct.Pin = SDATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SDATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PCLK_Pin */
  GPIO_InitStruct.Pin = PCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PCLK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* State Functions BEGIN-----------------------------------------------------------------*/
/**
  * @brief Wait for Serial Command from the Interface
  * @retval Next Action: go_g, go_ch, go_meas, repeat, fail
  */
Actions_TypeDef comm_wait_state(void){

	HAL_StatusTypeDef uart_state = HAL_OK;

	if (uart_cplt_lck == HAL_UNLOCKED){	//DMA complete transfer

		uart_cplt_lck = HAL_LOCKED;

		switch (comm[0]){
			case 'G': return go_g;
			case 'S': return go_ch;
			case 'M': return go_meas;
			case 'D': return go_diag;
			case 'E': return go_ex;
			default: return repeat;
		}
	}

	else {

		uart_state = HAL_UART_Receive_DMA(&huart3, comm, COMM_BUFF_SIZE); //Start receiving

		if((uart_state == HAL_ERROR)){		//If UART ERROR
				Error_Handler();
		}

		return repeat;
	}
}

/**
  * @brief Change Gain of the PGA
  * @retval Next Action: OK, repeat, fail
  */
Actions_TypeDef g_sel_state(void){

	gain_val = comm[1];

	dac_value = __DAC_VOLTAGE2BIT(gain_val);

	//Set DAC to the desired GAIN value
	if((HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value) != HAL_OK) ||
		  (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) != HAL_OK)
		Error_Handler();

	return ok;

}

/**
  * @brief Change Measurement Channel
  * @retval Next Action: OK, repeat, fail
  */
Actions_TypeDef ch_sel_state(void){

	//Channel selection initialization

	channels_value[0] = comm[1];
	channels_value[1] = comm[2];
	channels_value[2] = comm[3];
	channels_value[3] = comm[4];

	switch (sw_flag) {

		case SWITCH_BEGIN:

			HAL_GPIO_WritePin(SDATA_PORT, GPIO_PIN_0, GPIO_PIN_RESET); //reset data pin
			HAL_GPIO_WritePin(PCLK_PORT, GPIO_PIN_0, GPIO_PIN_SET); //reset the parallel clock pin

			//build the ch_data buffer (each channel connects to channel + n*16 switch)
			for(ch_counter = 0; ch_counter < 256; ch_counter++){
				ch_data[ch_counter] = SWITCH_DATA_OFF;

				if((ch_counter == comm[1] - 1)||(ch_counter == comm[2] + 15)||
						(ch_counter == comm[3] + 31)||(ch_counter == comm[4] + 47))
				{
					ch_data[ch_counter] = SWITCH_DATA_ON;
				}
			}

			ch_counter = 0; //reset channel counter
			sw_flag = SWITCH_TRANSFER;

			__HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);

			htim1.hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback = Switch_Cplt_Callback;

			if(HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)ch_data, (uint32_t)&(SDATA_PORT->BSRR), (uint32_t)SWITCH_SIZE) != HAL_OK)
				Error_Handler(); //starts DMA

			__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE); //enable DMA update

			if(HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
				Error_Handler(); //Starts Timer

		case SWITCH_TRANSFER:
			break;

		case SWITCH_LOAD:

		  if(HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2) != HAL_OK)
			  Error_Handler();

			HAL_GPIO_WritePin(PCLK_PORT, GPIO_PIN_0, GPIO_PIN_RESET); //enable parallel clock

			__HAL_TIM_SET_COUNTER(&htim1, 0);

			HAL_TIM_Base_Start(&htim4);

			sw_flag = SWITCH_END;

		case SWITCH_END:

			if(__HAL_TIM_GET_COUNTER(&htim4) >= PCLK_DELAY) //check if timer reached the PCLK length
			{
				HAL_GPIO_WritePin(PCLK_PORT, GPIO_PIN_0, GPIO_PIN_SET); //enable parallel clock
				HAL_TIM_Base_Stop(&htim4);
				sw_flag = SWITCH_BEGIN;
				return ok;
			}

	}
	return repeat;
}

/**
  * @brief Starts and Manages One Measurement
  * @retval Next Action: OK, repeat, fail
  */
Actions_TypeDef meas_state(void){

	//ADC Start (TRANSFER TO volt_vector USING DMA)
	if(HAL_ADC_Start_DMA(&hadc3, (uint32_t*) adc_buffer, ADC_BUFF_SIZE - 1) == HAL_OK)
		HAL_TIM_Base_Start(&htim4);					//If not busy, starts the timer

	if (__HAL_TIM_GET_COUNTER(&htim4) >= CHANNEL_STABILIZATION_TIME){ //Wait switching stabilization time

		//Start TIM3 to trigger the ADC
		if(HAL_TIM_Base_Start(&htim3) != HAL_OK)
		  Error_Handler();

		HAL_TIM_Base_Stop(&htim4); //stops counter
		return ok;
	}
	else
		return repeat;


}

/**
  * @brief Starts/Stops Excitation Signal (Square Wave, 10 kHz)
  * @retval Next Action: OK, repeat, fail
  */
Actions_TypeDef exc_state(void){

	if (comm[1] != 0) //turn ON
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2); //Starts PWM for Current Source (10 kHz)

	else				//turn OFF
		HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2); //Stops PWM for Current Source

	return ok;
}

/**
  * @brief Handles and Informs Errors
  * @retval Next Action - OK, repeat
  */
Actions_TypeDef error_state(void){
	Error_Handler();
	return ok;
}

/**
  * @brief Transmits the Status of the Machine (GAIN, CHANNELS and INPUT)
  * @retval Next Action - OK
  */
Actions_TypeDef diag_state(void){

	char *diag_message = malloc(DIAG_BUFF_SIZE * sizeof(char));
	snprintf(diag_message, DIAG_BUFF_SIZE, "%c%c%c%c%c%c%c",
			'G', gain_val,
			'S', channels_value[0], channels_value[1], channels_value[2], channels_value[3]);

	//Sends Diagnostics to Host
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*) diag_message, DIAG_BUFF_SIZE);

	return ok;
}

/* State Functions END-----------------------------------------------------------------*/

/* Callback Functions BEGIN-----------------------------------------------------------------*/

//Stops TIM3 once the DMA transfer is completed (this is called by the ADC DMA IRQ Handler)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	//Stops ADC Measurement and Timer
	if(HAL_TIM_Base_Stop(&htim3) != HAL_OK)
		Error_Handler();

	if(HAL_ADC_Stop_DMA(&hadc3) != HAL_OK)
		Error_Handler();

	//Stops Input Signal
	//if(HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2) != HAL_OK)
		//Error_Handler();

	entry_time = 0;

	//Sends Data to Host
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*) adc_buffer, 2*ADC_BUFF_SIZE);

}

//UART Receive callback (called by the UART DMA IRQ Handler)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uart_cplt_lck = HAL_UNLOCKED;
}


void Switch_Cplt_Callback(DMA_HandleTypeDef *hdma)
{
	sw_flag = SWITCH_LOAD;
}

/* Callback Functions END-----------------------------------------------------------------*/

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
