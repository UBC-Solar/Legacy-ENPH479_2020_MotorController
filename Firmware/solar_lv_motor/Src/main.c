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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "regular_conversion_manager.h"
#include "math.h"
#include "mc_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Motor/vehicle parameters
#define NMAX 4000.0 //Max motor velocity, [RPM]
#define IRMSMAX 4.0 //Max motor RMS phase current, [A]
#define IMAXP 1.0 //Max motor current (discharge), [% decimal (0.0-1.0)]
#define IREGENMAXP 1.0 //Max regen current, [% decimal (0.0-1.0)]
#define IREGENMAX 8.0 //Max battery (DC) regen current, [A]
#define WHEELRAD 0.004 //Wheel radius, [m]
#define CARMASS 5.0 //Vehicle mass, [kg]
#define VBUSMIN 6.0 //Minimum acceptable DC voltage, [V]

//Controller parameters
#define ISENSORGAIN 0.077 //Current sensor gain, [V/A]
#define MOTORSTARTTIME 1000 //Time to allow motor to start spinning upon restart [ms]
#define SPEEDRAMPTIME 150 //Ramp time for speed control command, can be 0 but may cause current surges[ms]
#define TORQUERAMPTIME 150 ////Ramp time for torque control command, can be 0 but may cause current surges[ms]

//Firmware parameters
#define NTOL 10.0 //Motor velocity logic tolerance, [RPM]
#define ITOLP 0.10 //Motor current logic tolerance, [% decimal (0.0-1.0)]
#define TIMERLIM 2000 //Watchdog timeout limit, [ms]

//Constants
#define PI 3.14

//Calculated constants
#define S16ACONVFACTOR 65536.0*ISENSORGAIN/3.3 //(1529) Conversion factor for A-to-s16A, [s16A/A]
#define S16ACONVFACTORINV 3.3/(65536.0*ISENSORGAIN) //(0.0006539) Conversion factor for s16A-to-A, [A/s16A]
#define IQLIM 1.414*IRMSMAX //(0.9898) Max Iq current = peak phase current, [A]
#define IQMAXS16A IQLIM*S16ACONVFACTOR //(1513.576) Max Iq current in s16A units, [s16A]
#define WMAX NMAX*PI/30.0 //Max motor velocity, [rad/s]
#define WTOL NTOL*PI/30.0 //Motor velocity logic tolerance, [rad/s]
#define VMAX WMAX*WHEELRAD //Max vehicle velocity, [m/s]
#define VTOL WTOL*WHEELRAD //Vehicle velocity logic tolerance, [m/s]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ************************* GLOBAL VARIABLES ************************* //
// ***** Global usage ***** //
uint16_t state = 0; // State tracker
uint16_t currentTime = 0; //Watchdog timer for CAN timeout
uint16_t lastTime = 0; //Watchdog timer for CAN timeout
char msg_debug[327]; // Serial debug msg

// ***** CAN variables ***** //
union {
	float testCAN_IN_float;
	uint32_t testCAN_IN_int;
}
testCAN_IN;

union {
	float CAN_IN_current_float;
	uint32_t CAN_IN_current_int;
}
CAN_IN_current;

union {
	float CAN_IN_velocity_float;
	uint32_t CAN_IN_velocity_int;
}
CAN_IN_velocity;

union {
	float CAN_OUT_busVoltage_float;
	uint32_t CAN_OUT_busVoltage_int;
}
CAN_OUT_busVoltage;

union {
	float CAN_OUT_busCurrent_float;
	uint32_t CAN_OUT_busCurrent_int;
}
CAN_OUT_busCurrent;

union {
	float CAN_OUT_mtrVelocity_float;
	uint32_t CAN_OUT_mtrVelocity_int;
}
CAN_OUT_mtrVelocity;

union {
	float CAN_OUT_carVelocity_float;
	uint32_t CAN_OUT_carVelocity_int;
}
CAN_OUT_carVelocity;

union {
	float CAN_OUT_phaseCurrent_float;
	uint32_t CAN_OUT_phaseCurrent_int;
}
CAN_OUT_phaseCurrent;

union {
	float CAN_OUT_mtrTemp_float;
	uint32_t CAN_OUT_mtrTemp_int;
}
CAN_OUT_mtrTemp;

union {
	float CAN_OUT_FETTemp_float;
	uint32_t CAN_OUT_FETTemp_int;
}
CAN_OUT_FETTemp;

// ***** Manual ADC variables ***** //
RegConv_t Pot1Conv;
uint8_t Pot1Handle;
uint16_t pot1_value;

RegConv_t Pot2Conv;
uint8_t Pot2Handle;
uint16_t pot2_value;

RegConv_t DCCurrConv;
uint8_t DCCurrHandle;
uint16_t DCCurr_value;

RegConv_t ThermAHConv;
uint8_t ThermAHHandle;

RegConv_t ThermALConv;
uint8_t ThermALHandle;
uint16_t thermAL_temp;

RegConv_t ThermBHConv;
uint8_t ThermBHHandle;
uint16_t thermBH_temp;

RegConv_t ThermBLConv;
uint8_t ThermBLHandle;
uint16_t thermBL_temp;

RegConv_t ThermCHConv;
uint8_t ThermCHHandle;
uint16_t thermCH_temp;

RegConv_t ThermCLConv;
uint8_t ThermCLHandle;
uint16_t thermCL_temp;

RegConv_t ThermMTRConv;
uint8_t ThermMTRHandle;

// ***** Helper variables ***** //

// ************************* END GLOBAL VARIABLES ************************* //
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

// ************************* STATE PROTOTYPES ************************* //

// 0xx states: Startup
void state000(void);
void state001(void);
void state002(void);
void state003(void);
void state099(void);

// 1xx states: Reading values
void state100(void);
void state101(void);
void state102(void);
void state103(void);
void state104(void);
void state105(void);
void state106(void);
void state107(void);
void state108(void);
void state109(void);
void state110(void);
void state111(void);

// 2xx states: Reading input
void state200(void);
void state201(void);
void state202(void);
void state203(void);
void state204(void);
void state298(void);

// 3xx states: Writing drive commands
void state300(void);
void state301(void);
void state302(void);
void state303(void);
void state304(void);

// 4xx states: Checking faults
void state400(void);
void state401(void);
void state402(void);
void state403(void);
void state404(void);
void state405(void);
void state406(void);

// 5xx states: Fan control
void state500(void);
void state501(void);
void state502(void);
void state503(void);

// 6xx states: CAN sending
void state600(void);
void state601(void);
void state602(void);
void state603(void);

// 7xx states: Faults
void state700(void);
void state701(void);
void state702(void);
void state703(void);
void state704(void);
void state705(void);
void state706(void);
void state707(void);
// ************************* END STATE PROTOYPES ************************* //

// ************************* HELPER PROTOTYPES ************************* //
float convertTempVal(uint16_t thermXX_value);
void printState();
void printNum(int num);
// ************************* END HELPER PROTOTYPES ************************* //
/* USER CODE END PFP */

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
	Pot1Conv.regADC = ADC1; /* to be modify to match your ADC */
	Pot1Conv.channel = ADC_CHANNEL_8;/* to be modify to match your ADC channel */
	Pot1Conv.samplingTime = ADC_SAMPLETIME_3CYCLES; /* to be modify to match your sampling time */
	Pot1Handle = RCM_RegisterRegConv (&Pot1Conv);

	Pot2Conv.regADC = ADC1; /* to be modify to match your ADC */
	Pot2Conv.channel = ADC_CHANNEL_0;/* to be modify to match your ADC channel */
	Pot2Conv.samplingTime = ADC_SAMPLETIME_3CYCLES; /* to be modify to match your sampling time */
	Pot2Handle = RCM_RegisterRegConv (&Pot2Conv);

	ThermAHConv.regADC = ADC1;
	ThermAHConv.channel = ADC_CHANNEL_12;
	ThermAHConv.samplingTime = ADC_SAMPLETIME_3CYCLES;
	ThermAHHandle = RCM_RegisterRegConv (&ThermAHConv);

	ThermALConv.regADC = ADC1;
	ThermALConv.channel = ADC_CHANNEL_13;
	ThermALConv.samplingTime = ADC_SAMPLETIME_3CYCLES;
	ThermALHandle = RCM_RegisterRegConv (&ThermALConv);

	ThermBHConv.regADC = ADC1;
	ThermBHConv.channel = ADC_CHANNEL_11;
	ThermBHConv.samplingTime = ADC_SAMPLETIME_3CYCLES;
	ThermBHHandle = RCM_RegisterRegConv (&ThermBHConv);

	ThermBLConv.regADC = ADC1;
	ThermBLConv.channel = ADC_CHANNEL_10;
	ThermBLConv.samplingTime = ADC_SAMPLETIME_3CYCLES;
	ThermBLHandle = RCM_RegisterRegConv (&ThermBLConv);

	//TODO - enable for V2 hardware, fix pin assignments
//	ThermCHConv.regADC = ADC1;
//	ThermCHConv.channel = ADC_CHANNEL_12;
//	ThermCHConv.samplingTime = ADC_SAMPLETIME_3CYCLES;
//	ThermCHHandle = RCM_RegisterRegConv (&ThermCHConv);
//
//	ThermCLConv.regADC = ADC1;
//	ThermCLConv.channel = ADC_CHANNEL_12;
//	ThermCLConv.samplingTime = ADC_SAMPLETIME_3CYCLES;
//	ThermCLHandle = RCM_RegisterRegConv (&ThermCLConv);

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_MotorControl_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  state000();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (state)
	  {
	  	  case 1: state001(); break;
  	  	  case 2: state002(); break;
  	  	  case 3: state003(); break;
  	  	  case 99: state099(); break;

	  	  case 100: state100(); break;
	  	  case 101: state101(); break;
	  	  case 102: state102(); break;
	  	  case 103: state103(); break;
	  	  case 104: state104(); break;
	  	  case 105: state105(); break;
	  	  case 106: state106(); break;
	  	  case 107: state107(); break;
	  	  case 108: state108(); break;
	  	  case 109: state109(); break;
	  	  case 110: state110(); break;
	  	  case 111: state111(); break;

	  	  case 200: state200(); break;
	  	  case 201: state201(); break;
	  	  case 202: state202(); break;
	  	  case 203: state203(); break;
	  	  case 204: state204(); break;
	  	  case 298: state298(); break;

	  	  case 300: state300(); break;
	  	  case 301: state301(); break;
	  	  case 302: state302(); break;
	  	  case 303: state303(); break;
	  	  case 304: state304(); break;

	  	  case 400: state400(); break;
	  	  case 401: state401(); break;
	  	  case 402: state402(); break;
	  	  case 403: state403(); break;
	  	  case 404: state404(); break;
	  	  case 405: state405(); break;
	  	  case 406: state406(); break;

	  	  case 500: state500(); break;
	  	  case 501: state501(); break;
	  	  case 502: state502(); break;
	  	  case 503: state503(); break;

	  	  case 600: state600(); break;
	  	  case 601: state601(); break;
	  	  case 602: state602(); break;
	  	  case 603: state603(); break;

	  	  case 700: state700(); break;
	  	  case 701: state701(); break;
	  	  case 702: state702(); break;
	  	  case 703: state703(); break;
	  	  case 704: state704(); break;
	  	  case 705: state705(); break;
	  	  case 706: state706(); break;
	  	  case 707: state707(); break;
	  }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM1_UP_TIM10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_HallSensor_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = M1_HALL_TIM_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
//  sConfig.IC1Filter = M1_HALL_IC_FILTER;
  sConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DRV_DIS_GPIO_Port, DRV_DIS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLT_OUT_GPIO_Port, FLT_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_OUT_GPIO_Port, GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DRV_DIS_Pin */
  GPIO_InitStruct.Pin = DRV_DIS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DRV_DIS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MTR_OC_Pin HV_OV_Pin */
  GPIO_InitStruct.Pin = MTR_OC_Pin|HV_OV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FET_OT_Pin */
  GPIO_InitStruct.Pin = FET_OT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FET_OT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MTR_OT_Pin */
  GPIO_InitStruct.Pin = MTR_OT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MTR_OT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLT_OUT_Pin */
  GPIO_InitStruct.Pin = FLT_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLT_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_OUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// ************************* STATE FUNCTIONS *************************//

//***** 1xx states: Startup *****//
/**
 * @brief Startup entrance state
 */
void state000(void)
{
	MC_StopMotor1(); //Make sure motor is stopped at startup
	state = 1; //FSM mode
	//state = 99; //Workbench mode
}

/**
 * @brief Check starting values
 */
void state001(void)
{
	state = 2; //Assume no fault; prepare to go to next state
	HAL_Delay(250); //Temporary delay to account for startup transients (DC voltage spike)

	//HV OV
	if (HAL_GPIO_ReadPin(HV_OV_GPIO_Port, HV_OV_Pin) == 1)
	{
		sprintf(msg_debug, "001: DC BUS OV\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		state = 703;
	}

	//MTR OT
	else if (HAL_GPIO_ReadPin(MTR_OT_GPIO_Port, MTR_OT_Pin) == 1)
	{
		sprintf(msg_debug, "001: MOTOR OT\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		state = 701;
	}

	//TODO - MTR OC

	//FET OT
	else if (HAL_GPIO_ReadPin(FET_OT_GPIO_Port, FET_OT_Pin) == 1)
	{
		sprintf(msg_debug, "001: FET OT\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		state = 704;
	}

	//MC FLT
	else if (MC_GetSTMStateMotor1() == 10 || MC_GetSTMStateMotor1() == 11)
	{
		sprintf(msg_debug, "001: MOTOR CONTROL FAULT\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		state = 706;
	}
}

/**
 * @brief Check if DC bus is active
 */
void state002(void)
{
	CAN_OUT_busVoltage.CAN_OUT_busVoltage_float = ((float) PQD_MotorPowMeasM1.pVBS->AvBusVoltage_d) * ((float) PQD_MotorPowMeasM1.pVBS->ConversionFactor) / 65536.0;
	if (CAN_OUT_busVoltage.CAN_OUT_busVoltage_float < VBUSMIN)
	{
		//Wait for DC bus to activate
		sprintf(msg_debug, "Waiting for DC voltage\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
	}
	else
	{
		sprintf(msg_debug, "DC bus activated\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		//Start watchdog when HV is started
		lastTime = HAL_GetTick();
		state = 100;
	}
}

/**
 * @brief EMPTY
 */
void state003(void)
{
	printState();
}

/**
 * @brief Workbench mode
 */
void state099(void)
{
	printState();
}

//***** 1xx states: Reading current values *****//
/**
 * @brief Value reading entrance state
 */
void state100(void)
{
	state = 101;
}

/**
 * @brief Read DC voltage
 */
void state101(void)
{
//	uint16_t voltageConversionFactor = PQD_MotorPowMeasM1.pVBS->ConversionFactor;
//	uint16_t busVoltage_d = PQD_MotorPowMeasM1.pVBS->AvBusVoltage_d;
//	float busVoltage_V = ((float) PQD_MotorPowMeasM1.pVBS->AvBusVoltage_d) * ((float) PQD_MotorPowMeasM1.pVBS->ConversionFactor) / 65536.0;

	CAN_OUT_busVoltage.CAN_OUT_busVoltage_float = ((float) PQD_MotorPowMeasM1.pVBS->AvBusVoltage_d) * ((float) PQD_MotorPowMeasM1.pVBS->ConversionFactor) / 65536.0;

	if (CAN_OUT_busVoltage.CAN_OUT_busVoltage_float < VBUSMIN)
	{
		//DC bus disconnected: soft fault
		sprintf(msg_debug, "DC voltage missing\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

		state = 707;
	}
	else
	{
		state = 102;
	}
}

/**
 * @brief Read DC current
 */
void state102(void)
{
	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{
		//if Idle, then program a new conversion request
		RCM_RequestUserConv(DCCurrHandle);
	}
	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{
		//if Done, then read the captured value
		CAN_OUT_busCurrent.CAN_OUT_busCurrent_float = ((RCM_GetUserConv() * 3.3 / 65535.0) - 1.65) / ISENSORGAIN;
		state = 103;
	}
}

/**
 * @brief Read motor velocity (RPM) & vehicle velocity (m/s)
 */
void state103(void)
{
	CAN_OUT_mtrVelocity.CAN_OUT_mtrVelocity_float = (float) 6*fabs(MC_GetMecSpeedAverageMotor1()); //Multiply by 6 to convert from dHz to RPM;
	CAN_OUT_carVelocity.CAN_OUT_carVelocity_float = CAN_OUT_mtrVelocity.CAN_OUT_mtrVelocity_float * WHEELRAD * PI / 30.0; //[m/s]
	state = 104;
}

/**
 * @brief Read phase current
 */
void state104(void)
{
	CAN_OUT_phaseCurrent.CAN_OUT_phaseCurrent_float = ((float) MC_GetPhaseCurrentAmplitudeMotor1()) * S16ACONVFACTORINV;
	state = 105;
}

/**
 * @brief Read motor temperature
 */
void state105(void)
{
	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{
		//if Idle, then program a new conversion request
		RCM_RequestUserConv(ThermMTRHandle);
	}
	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{
		//if Done, then read the captured value
		CAN_OUT_mtrTemp.CAN_OUT_mtrTemp_float = convertTempVal(RCM_GetUserConv());
		state = 106;
	}
}

/**
 * @brief Read FET temperature AH
 */
void state106(void)
{
	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{
		//if Idle, then program a new conversion request
		RCM_RequestUserConv(ThermAHHandle);
	}
	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{
		//if Done, then read the captured value
		CAN_OUT_FETTemp.CAN_OUT_FETTemp_float = convertTempVal(RCM_GetUserConv());
		state = 107;
	}
}

/**
 * @brief Read FET temperature AL
 */
void state107(void)
{
	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{
		//if Idle, then program a new conversion request
		RCM_RequestUserConv(ThermALHandle);
	}
	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{
		//if Done, then read the captured value
		thermAL_temp = convertTempVal(RCM_GetUserConv());
		//If greater than current FETTemp_float, replace (only store max FET temp)
		if (thermAL_temp > CAN_OUT_FETTemp.CAN_OUT_FETTemp_float) CAN_OUT_FETTemp.CAN_OUT_FETTemp_float = thermAL_temp;
		state = 108;
	}
}

/**
 * @brief Read FET temperature BH
 */
void state108(void)
{
	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{
		//if Idle, then program a new conversion request
		RCM_RequestUserConv(ThermBHHandle);
	}
	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{
		//if Done, then read the captured value
		thermBH_temp = convertTempVal(RCM_GetUserConv());
		if (thermBH_temp > CAN_OUT_FETTemp.CAN_OUT_FETTemp_float) CAN_OUT_FETTemp.CAN_OUT_FETTemp_float = thermBH_temp;
		state = 109;
	}
}

/**
 * @brief Read FET temperature BL
 */
void state109(void)
{
	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{
		//if Idle, then program a new conversion request
		RCM_RequestUserConv(ThermBLHandle);
	}
	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{
		//if Done, then read the captured value
		thermBL_temp = convertTempVal(RCM_GetUserConv());
		if (thermBL_temp > CAN_OUT_FETTemp.CAN_OUT_FETTemp_float) CAN_OUT_FETTemp.CAN_OUT_FETTemp_float = thermBL_temp;
		state = 110;
	}
}

/**
 * @brief Read FET temperature CH
 */
void state110(void)
{
	//TODO - enable for V2 hardware
//	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
//	{
//		//if Idle, then program a new conversion request
//		RCM_RequestUserConv(ThermCHHandle);
//	}
//	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
//	{
//		//if Done, then read the captured value
//		thermCH_temp = convertTempVal(RCM_GetUserConv());
//		if (thermCH_temp > CAN_OUT_FETTemp.CAN_OUT_FETTemp_float) CAN_OUT_FETTemp.CAN_OUT_FETTemp_float = thermCH_temp;
//		state = 111;
//	}
	state = 111;
}

/**
 * @brief Read FET temperature CL
 */
void state111(void)
{
	//TODO - enable for V2 hardware
//	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
//	{
//		//if Idle, then program a new conversion request
//		RCM_RequestUserConv(ThermCLHandle);
//	}
//	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
//	{
//		//if Done, then read the captured value
//		thermCL_temp = convertTempVal(RCM_GetUserConv());
//		if (thermCL_temp > CAN_OUT_FETTemp.CAN_OUT_FETTemp_float) CAN_OUT_FETTemp.CAN_OUT_FETTemp_float = thermCL_temp;
//		state = 200;
//	}
	state = 200;
}

//***** 2xx states: Reading user inputs *****//
/**
 * @brief Input reading entrance state
 */
void state200(void)
{
	state = 201;
}

/**
 * @brief Check watchdog - TODO
 */
void state201(void)
{
	currentTime = HAL_GetTick();
//	sprintf(msg_debug, "dt: %hu\r\n", currentTime - lastTime);
//	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

	if (currentTime - lastTime > TIMERLIM)
	{
		//Timeout error
		state = 705;
	}
	else
	{
		lastTime = currentTime;
		state = 202;
	}
}

/**
 * @brief Read CAN msg - TEMP: read pot 2
 */
void state202(void)
{
	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{
		//if Idle, then program a new conversion request
		RCM_RequestUserConv(Pot2Handle);
	}
	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{
		//if Done, then read the captured value
		pot2_value = RCM_GetUserConv();
		//Scale to [%]
		CAN_IN_current.CAN_IN_current_float = ((float) pot2_value) / 45535.0;

		sprintf(msg_debug, "CAN_IN_current float (percent): %hu\r\n", (int) (100.0*CAN_IN_current.CAN_IN_current_float));
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

		state = 298;
	}
}

/**
 * @brief Increment time
 */
void state203(void)
{
	state = 204;
}

/**
 * @brief Decide drive command
 */
void state204(void)
{
	//Default: torque control
	state = 301;

	//If I > Imax
	if (CAN_IN_current.CAN_IN_current_float > IMAXP + ITOLP)
	{
		//DNE
		sprintf(msg_debug, "State DNE\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

		CAN_IN_current.CAN_IN_current_float = 0.0;
		CAN_IN_velocity.CAN_IN_velocity_float = 0.0;
		//state = 798;
	}
	// If I = 0
	else if (CAN_IN_current.CAN_IN_current_float <= ITOLP)
	{
		//Torque control - do nothing
	}
	// If I = Imax && V <= Vmax
	else if (CAN_IN_current.CAN_IN_current_float >= IMAXP - ITOLP
			&& CAN_IN_current.CAN_IN_current_float <= IMAXP + ITOLP
			&& CAN_IN_velocity.CAN_IN_velocity_float <= VMAX + VTOL)
	{
		// Speed control
		state = 302;
		if (CAN_IN_velocity.CAN_IN_velocity_float < VTOL)
		{
			// Regen
			state = 303;
		}
	}
	// If I < Imax && V < VMax
	else if (CAN_IN_current.CAN_IN_current_float < IMAXP - ITOLP
			&& CAN_IN_velocity.CAN_IN_velocity_float < VMAX - VTOL)
	{
		// Speed control
		state = 302;
		if (CAN_IN_velocity.CAN_IN_velocity_float < VTOL)
		{
			// Regen
			state = 303;
		}
	}
}

/**
 * @brief TEMP - read pot 1
 */
void state298(void)
{
	if (RCM_GetUserConvState() == RCM_USERCONV_IDLE)
	{
		//if Idle, then program a new conversion request
		RCM_RequestUserConv(Pot1Handle);
	}
	else if (RCM_GetUserConvState() == RCM_USERCONV_EOC)
	{
		//if Done, then read the captured value
		pot1_value = RCM_GetUserConv();

		//Scale to [m/s]
		CAN_IN_velocity.CAN_IN_velocity_float = ((float) pot1_value) / 45535.0 * VMAX;

		sprintf(msg_debug, "100x CAN_IN_velocity float (m/s): %hu\r\n", (int) (CAN_IN_velocity.CAN_IN_velocity_float*100.0));
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

		state = 203;
	}
}

//***** 3xx states: Writing drive commands *****//
/**
 * @brief Drive command entrance state
 */
void state300(void)
{
	state = 301;
}

/**
 * @brief Write torque ramp
 */
void state301(void)
{
	float finalTorque = CAN_IN_current.CAN_IN_current_float * IQMAXS16A;

	sprintf(msg_debug, "Torque ramp value: %hu\r\n", (int) finalTorque);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

	MC_ProgramTorqueRampMotor1((int) finalTorque, TORQUERAMPTIME);
	state = 304;
}

/**
 * @brief Write speed ramp
 */
void state302(void)
{
	float finalSpeed = CAN_IN_velocity.CAN_IN_velocity_float * 30.0 / (PI * WHEELRAD); //RPM

	sprintf(msg_debug, "Speed ramp value (RPM): %hu\r\n", (int) finalSpeed);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

	MC_ProgramSpeedRampMotor1(finalSpeed/6.0, SPEEDRAMPTIME);
	state = 304;
}

/**
 * @brief Write regen speed ramp
 */
void state303(void)
{
	float Vbat = CAN_OUT_busVoltage.CAN_OUT_busVoltage_float; //[V]
	float vehicleSpeed = CAN_OUT_carVelocity.CAN_OUT_carVelocity_float; //[m/s]
	float regenPower = Vbat * IREGENMAX * CAN_IN_current.CAN_IN_current_float; //P = VI
	float regenEnergy = 0.5 * CARMASS * vehicleSpeed * vehicleSpeed; //1/2 mV^2
	float regenTime = regenEnergy / regenPower; //[s]
	float regenTimeMS = regenTime * 1000.0; //[ms]

	MC_ProgramSpeedRampMotor1(0, regenTimeMS);

	state = 304;
}

/**
 * @brief Set motor status
 */
void state304(void)
{
	int motorState = MC_GetSTMStateMotor1();
	int motorSpeed = (int) CAN_OUT_mtrVelocity.CAN_OUT_mtrVelocity_float; //[RPM]

	sprintf(msg_debug, "Current motor speed: %hu\r\n", motorSpeed);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

	int motorSpin = 0;
	int userSpin = 1;
	int STMSpin = 0;

	if (motorSpeed > 0 && motorSpeed < NMAX)
	{
		motorSpin = 1;
	}
	if (CAN_IN_velocity.CAN_IN_velocity_float < VTOL || CAN_IN_current.CAN_IN_current_float < ITOLP) userSpin = 0;
	if (!(motorState == 0 || motorState == 7)) STMSpin = 1; //States in state_machine.h

	state = 400;

	//Cases:
	//If motor is     spinning AND should     be AND STM started --> Leave
	//If motor is     spinning AND should     be AND STM stopped --> Fake state
	//If motor is     spinning AND should not be AND STM started --> Stop
	//If motor is     spinning AND should not be AND STM stopped --> Fake state
	//If Motor is not spinning AND should     be AND STM started --> Check fault
	//If Motor is not spinning AND should     be AND STM stopped --> Start
	//If motor is not spinning AND should not be AND STM started --> Stop
	//If motor is not spinning AND should not be AND STM stopped --> Leave

	if      (motorSpin == 1 && userSpin == 1 && STMSpin == 1)
	{
		//motor = spin, user = spin, STM = spin
		//Leave
	}
	else if (motorSpin == 1 && userSpin == 1 && STMSpin == 0)
	{
		//motor = spin, user = spin, STM = stop
		//Fault/panic?? State not really possible
//		state = 706;
	}
	else if (motorSpin == 1 && userSpin == 0 && STMSpin == 1)
	{
		//motor = spin, user = stop, STM = spin
		//Stop
		sprintf(msg_debug, "User says motor should not be spinning\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		MC_StopMotor1();
	}
	else if (motorSpin == 1 && userSpin == 0 && STMSpin == 0)
	{
		//motor = spin, user = stop, STM = stop
		//Fault/panic?? State not really possible
//		state = 706;
	}
	else if (motorSpin == 0 && userSpin == 1 && STMSpin == 1)
	{
		//motor = stop, user = spin, STM = spin
		//Stop motor, check DC bus, then proceed to check faults
		sprintf(msg_debug, "Motor not spinning. Stopping STM\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		MC_StopMotor1();
	}
	else if (motorSpin == 0 && userSpin == 1 && STMSpin == 0)
	{
		//motor = stop, user = spin, STM = stop
		//Start
		MC_StartMotor1();
		sprintf(msg_debug, "Motor started\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		HAL_Delay(MOTORSTARTTIME);
	}
	else if (motorSpin == 0 && userSpin == 0 && STMSpin == 1)
	{
		//motor = stop, user = stop, STM = spin
		//Stop
		MC_StopMotor1();
		sprintf(msg_debug, "Motor stopped\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
	}
	else if (motorSpin == 0 && userSpin == 0 && STMSpin == 0)
	{
		//motor = stop, user = stop, STM = stop
		//Leave
	}
	// Proceed to fault checking (4xx)
}

//***** 4xx states: Checking faults *****//
/**
 * @brief Fault check entrance state
 */
void state400(void)
{
	state = 401;
}

/**
 * @brief Check MTR OT
 */
void state401(void)
{
	state = 402;
	if (HAL_GPIO_ReadPin(MTR_OT_GPIO_Port, MTR_OT_Pin) == 1)
	{
		sprintf(msg_debug, "MOTOR OT\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		HAL_Delay(1000);
		state = 701;
	}
}

/**
 * @brief Check MTR OC
 */
void state402(void)
{
	//TODO - MTR OC
	state = 403;
}

/**
 * @brief Check HV OV
 */
void state403(void)
{
	state = 404;
	if (HAL_GPIO_ReadPin(HV_OV_GPIO_Port, HV_OV_Pin) == 1)
	{
		sprintf(msg_debug, "DC BUS OV\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		HAL_Delay(1000);
		state = 703;
	}
}

/**
 * @brief Check Check FET OT
 */
void state404(void)
{
	state = 405;
	if (HAL_GPIO_ReadPin(FET_OT_GPIO_Port, FET_OT_Pin) == 1)
	{
		sprintf(msg_debug, "FET OT\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		HAL_Delay(1000);
		state = 704;
	}
}

/**
 * @brief Check Check communication fault
 */
void state405(void)
{
	state = 406;
	//TODO - check CAN msg
}

/**
 * @brief Check control fault
 */
void state406(void)
{
	state = 500;
	//TODO - extract error codes
	if (MC_GetCurrentFaultsMotor1() > 0 || MC_GetOccurredFaultsMotor1() > 0)
	{
		state = 706;
	}
}

//***** 5xx states: Fan control *****//
/**
 * @brief Fan control entrance state
 */
void state500(void)
{
	state = 501;
}

/**
 * @brief
 */
void state501(void)
{
	state = 502;
}

/**
 * @brief
 */
void state502(void)
{
	state = 503;
}

/**
 * @brief
 */
void state503(void)
{
	state = 600;
}

//***** 6xx states: Sending CAN messages *****//
/**
 * @brief CAN sending entrance state
 */
void state600(void)
{
	state = 601;
}

/**
 * @brief
 */
void state601(void)
{
	state = 602;
}

/**
 * @brief
 */
void state602(void)
{
	state = 603;
}

/**
 * @brief
 */
void state603(void)
{
	state = 100;
}

//***** 7xx states: Faults *****//
/**
 * @brief Fault entrance state
 */
void state700(void)
{
	sprintf(msg_debug, "State %hu\r\n", state);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(FLT_OUT_GPIO_Port, FLT_OUT_Pin, GPIO_PIN_SET); //Set FLT_OUT high
	HAL_GPIO_WritePin(DRV_DIS_GPIO_Port, DRV_DIS_Pin, GPIO_PIN_SET); //Set DRV_DIS high
	sprintf(msg_debug, "Fault\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
	HAL_Delay(1000);
	state = 701;
}

/**
 * @brief Hard fault - MTR_OT
 */
void state701(void)
{
	MC_StopMotor1();
	printState();
	while(1){}
}

/**
 * @brief Hard fault - MTR_OC
 */
void state702(void)
{
	MC_StopMotor1();
	printState();
	while(1){}
}

/**
 * @brief Hard Fault - HV_OV
 */
void state703(void)
{
	MC_StopMotor1();
	printState();
	while(1){}
}

/**
 * @brief Hard fault - FET_OT
 */
void state704(void)
{
	MC_StopMotor1();
	printState();
	while(1){}
}

/**
 * @brief Soft fault - Watchdog/communication error
 */
void state705(void)
{
	MC_StopMotor1();
	printState();
	while(1){}
}

/**
 * @brief Soft fault - Motor control error
 */
void state706(void)
{
	MC_StopMotor1();
	printState();
	while(1){}
}

/**
 * @brief Soft fault - Missing DC voltage
 */
void state707(void)
{
	MC_StopMotor1();
	sprintf(msg_debug, "Soft fault 707: DC voltage missing\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);

	CAN_OUT_busVoltage.CAN_OUT_busVoltage_float = ((float) PQD_MotorPowMeasM1.pVBS->AvBusVoltage_d) * ((float) PQD_MotorPowMeasM1.pVBS->ConversionFactor) / 65536.0;

	if (CAN_OUT_busVoltage.CAN_OUT_busVoltage_float > VBUSMIN)
	{
		//Restart controller
		sprintf(msg_debug, "DC voltage reconnected\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
		state = 1;
	}
}

// ************************* END STATE FUNCTIONS ************************* //

// ************************* HELPER FUNCTIONS ************************* //
float convertTempVal(uint16_t thermXX_value)
{
	thermXX_value = thermXX_value / 16.0; // Convert left-aligned to right-aligned (0-4096)
	float thermXX_voltage = thermXX_value * 3300.0 / 4096.0; //Convert to mV (to avoid decimal places)
	float thermXX_resistance = 3300.0 * 3300 / thermXX_voltage;
	thermXX_resistance = thermXX_resistance - 3300;
	float thermXX_temp = 1000.0*(log(thermXX_resistance / 10000.0) + (3435.0/298.0));
	thermXX_temp = 1000.0 * 3435.0 / thermXX_temp;
	thermXX_temp = thermXX_temp - 273.0;
	return thermXX_temp;
}

void printState(void)
{
	sprintf(msg_debug, "State %hu\r\n", state);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
	HAL_Delay(250);
}

void printNum(int num)
{
	sprintf(msg_debug, "Number: %hu\r\n", num);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg_debug, strlen(msg_debug), HAL_MAX_DELAY);
	HAL_Delay(250);
}

// ************************* END HELPER FUNCTIONS ************************* //
/* USER CODE END 4 */

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
