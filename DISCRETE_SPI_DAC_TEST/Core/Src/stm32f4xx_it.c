/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint32_t data_len ;
extern uint8_t DAC_data[512];
extern uint8_t waveform_type;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim1_up;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
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

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if (waveform_type==0){ // Waveform type sine
		HAL_DMA_Abort(&hdma_tim1_up);
		data_len = 512;
		//--- sine create
		double sin_val,x,y;
		y = data_len;

		for (uint32_t i = 0; i<data_len;i++){
		  //data[index_count] = i;
		  x = i;
		  sin_val = (125 * sin((x / y) * 2.0 * M_PI)) + 127.0;
		  DAC_data[i] = (uint8_t)sin_val;
		}
		//----
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&DAC_data[0],  (uint32_t)&(SPI2->DR), data_len);
		waveform_type = 1;
	}
	else if (waveform_type==1){ //Waveform type fast sine
		HAL_DMA_Abort(&hdma_tim1_up);
		data_len = 50;
		//--- fast sine create
		double sin_val,x,y;
		y = data_len;

		for (uint32_t i = 0; i<data_len;i++){
		  //data[index_count] = i;
		  x = i;
		  sin_val = (125 * sin((x / y) * 2.0 * M_PI)) + 127.0;
		  DAC_data[i] = (uint8_t)sin_val;
		}
		//----
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&DAC_data[0],  (uint32_t)&(SPI2->DR), data_len);
		waveform_type = 2;
	}
	else if (waveform_type==2){// Waveform type ramp
		HAL_DMA_Abort(&hdma_tim1_up);
		data_len = 256;

		for (uint32_t i = 0; i< data_len;i++){
			DAC_data[i] = i;

		}
		//----
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&DAC_data[0],  (uint32_t)&(SPI2->DR), data_len);
		waveform_type = 3;
	}
	else if (waveform_type==3){// Waveform type fast ramp
		HAL_DMA_Abort(&hdma_tim1_up);
		data_len = 50;
		uint32_t step = 255 / data_len ;
		//uint32_t top = index_lim *step;
		uint8_t val_count =0;
		for (uint32_t i = 0; i< data_len;i++){
			DAC_data[i] = val_count;
			val_count	= val_count + step;
		}
		//----
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&DAC_data[0],  (uint32_t)&(SPI2->DR), data_len);
		waveform_type = 4;
	}
	else if (waveform_type==4){// Waveform type triangle
		HAL_DMA_Abort(&hdma_tim1_up);
		data_len = 512;

		for (uint32_t i = 0; i< 256;i++){
			DAC_data[i] = i;

		}
		for (uint32_t i = 0; i<256;i++){
			DAC_data[i+256] = (uint8_t)255-i;
		}
		//----
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&DAC_data[0],  (uint32_t)&(SPI2->DR), data_len);
		waveform_type = 5;
	}
	else if (waveform_type==5){// Waveform type fast triangle
		HAL_DMA_Abort(&hdma_tim1_up);
		data_len = 50;
		uint32_t index_lim = data_len/2;
		uint32_t step = 255 / index_lim ;
		//uint32_t top = index_lim *step;
		uint8_t val_count =0;
		for (uint32_t i = 0; i< index_lim;i++){
			DAC_data[i] = val_count;
			val_count	= val_count + step;
		}
		for (uint32_t i = 0; i<index_lim;i++){
			DAC_data[i+index_lim] = val_count;
			val_count	= val_count - step;
		}
		//----
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&DAC_data[0],  (uint32_t)&(SPI2->DR), data_len);
		waveform_type = 6;
	}
	else if (waveform_type==6){// Waveform type rand
		HAL_DMA_Abort(&hdma_tim1_up);
		data_len = 512;
		double rand_val ;
		//--- random
		for (uint32_t i = 0; i<data_len;i++){
		  //data[index_count] = i;
			double rand_val = (rand() % 255) ;
			DAC_data[i] = (uint8_t)rand_val;
		}
		//----
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&DAC_data[0],  (uint32_t)&(SPI2->DR), data_len);
		waveform_type = 0;
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
void DMA2_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

  /* USER CODE END DMA2_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim1_up);
  /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

  /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
