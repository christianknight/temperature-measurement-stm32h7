/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef struct tagTEMP_STRUCT {
	float 	temperature;
	float 	resistance;
} TEMP_STRUCT;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BANDGAP_VOLTAGE       1400    /* in mV */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static const TEMP_STRUCT cTemp_struct_array[] = {
		{-40.0 ,195.652}, {-35.0 ,148.171}, {-30.0 ,113.347}, {-25.0 ,87.559}, {-20.0 ,68.237},
		{-15.0 ,53.650}, {-10.0 ,42.506}, {-5.0,33.892}, {0.0, 27.219}, {5.0, 22.021}, {10.0, 17.926},
		{15.0, 14.674}, {20.0, 12.081}, {25.0, 10.000}, {30.0, 8.315}, {35.0, 6.948}, {40.0, 5.834},
		{45.0, 4.917}, {50.0, 4.161}, {55.0, 3.535}, {60.0, 3.014}, {65.0, 2.586}, {70.0, 2.228},
		{75.0, 1.925}, {80.0, 1.669}, {85.0, 1.452}, {90.0, 1.268}, {95.0, 1.110}, {100.0 ,0.974},
		{105.0 ,0.858}, {110.0 ,0.758}, {115.0 ,0.672}, {120.0 ,0.596}, {125.0 ,0.531}
};

uint16_t temperature = 0x010C,    /* Measured temperature in Celsius * 10 (default 0x010C -> 26.8C) */
         voltage = 0;             /* Estimated DC supply voltage, in V */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static float interpolate_func(float cur_resistor_val);
void         HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

	  if (((HAL_ADC_GetState(&hadc3) & HAL_ADC_STATE_READY) == 1UL)) {
		  HAL_ADC_Start_IT(&hadc3);
	  }

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC3 global interrupt.
  */
void ADC3_IRQHandler(void)
{
  /* USER CODE BEGIN ADC3_IRQn 0 */

  /* USER CODE END ADC3_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC3_IRQn 1 */

  /* USER CODE END ADC3_IRQn 1 */
}

/* USER CODE BEGIN 1 */
static float
interpolate_func(float cur_resistor_val) {
	int j, sample_count = sizeof(cTemp_struct_array) / sizeof(TEMP_STRUCT);
	double L = 0.0f;
	for (j = 0; j < sample_count; j++) {
		if (cTemp_struct_array[j].resistance <= cur_resistor_val)
			break;
	}
	if (j == 0) {
		L = cTemp_struct_array[0].temperature;
	}
	else if (j == sample_count) {
		L = cTemp_struct_array[sample_count-1].temperature;
	}
	else {
		L = cTemp_struct_array[j-1].temperature +
			(cTemp_struct_array[j].temperature - cTemp_struct_array[j-1].temperature) *
			(cur_resistor_val - cTemp_struct_array[j-1].resistance) /
			(cTemp_struct_array[j].resistance - cTemp_struct_array[j-1].resistance);
	}
	return L;
}

void
HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef * hadc) {
	uint32_t adc_val;                 /* To temporarily store ADC conversion value */
	float temp_val;                   /* To temporarily store calculated temperature value */
	static uint8_t conv_cnt = 0;      /* Counter to keep track of number of conversions in continuous scan mode */

	/* Retrieve the value for the completed conversion */
	adc_val = HAL_ADC_GetValue(&hadc3);

	/* PSU voltage calculation (rank #1) */
	if (!conv_cnt) {
		voltage = (4096/(float)adc_val * (float)BANDGAP_VOLTAGE) + 700;
		conv_cnt++;
	}

	/* Temperature calculation (rank #2) */
	else {
		HAL_ADC_Stop(&hadc3);    /* Stop the ADC from running */
		temp_val = 10.0f * (float)adc_val / (4096.0f - (float)adc_val);
		temp_val = interpolate_func(temp_val);
		temperature = (uint16_t)(10.0f * temp_val) + 100;
		conv_cnt = 0;
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
