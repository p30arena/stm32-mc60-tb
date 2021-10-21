/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
uint16_t u1_old_pos = 0;
uint16_t u2_old_pos = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/**
 * some codes are from these links:
 * https://stm32f4-discovery.net/2017/07/stm32-tutorial-efficiently-receive-uart-data-using-dma/
 * https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 */

HAL_StatusTypeDef usart_process_data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len);
void usart_rx_check(uint16_t *old_pos_ptr, uint16_t rx_buffer_len, uint8_t *rx_buffer, UART_HandleTypeDef *huart, uint16_t rem);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
  * @brief This function handles Prefetch fault, memory access fault.
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
  HAL_SYSTICK_IRQHandler();
  /* USER CODE END SysTick_IRQn 1 */
}

void usart_rx_check(uint16_t *old_pos_ptr, uint16_t rx_buffer_len, uint8_t *rx_buffer, UART_HandleTypeDef *huart, uint16_t rx_buffer_free_len)
{
  HAL_StatusTypeDef result;
  uint16_t old_pos = *old_pos_ptr;                   // read pointer
  uint16_t pos = rx_buffer_len - rx_buffer_free_len; // dma write pointer

  if (pos != old_pos)
  {
    if (pos > old_pos)
    {
      result = usart_process_data(huart, &rx_buffer[old_pos], pos - old_pos);
    }
    else
    {
      result = usart_process_data(huart, &rx_buffer[old_pos], rx_buffer_len - old_pos);
      pos = 0;
    }

    if (result == HAL_OK)
    {
      *old_pos_ptr = pos;
    }
  }
}

/**
  * @brief gathers the data until it sees a new line, then sends the gathered data
  * 
  * 1. if <HEAD> is not new line: GATHER else: <recovery TRUE>
  * 2. if <HEAD> is new line
  * 3.   SEND
  * 4. else
  * 5.   INC <HEAD>
  */

HAL_StatusTypeDef cmd_send(const uint8_t *rx_buffer, uint16_t len, uint8_t *tx_buffer, uint16_t *tx_idx, uint16_t tx_buffer_len, UART_HandleTypeDef *huart)
{
  HAL_StatusTypeDef result = HAL_OK;
  uint8_t recovery = 0;

  if (tx_buffer[*tx_idx] != 10) // must not be new line
  {
    const uint8_t *b = rx_buffer;
    for (; len > 0; --len, ++b)
    {
      tx_buffer[*tx_idx] = *b;
      if (len > 1) // early stop idx inc
      {
        *tx_idx = (*tx_idx + 1) % tx_buffer_len;
      }
    }
  }
  else
  {
    recovery = 1;
  }

  if (tx_buffer[*tx_idx] == 10) // new line
  {
    result = HAL_UART_Transmit_DMA(huart, (uint8_t *)tx_buffer, *tx_idx + 1);

    if (result == HAL_OK)
    {
      *tx_idx = 0;
      tx_buffer[0] = 0;
    }
    else if (!recovery)
    {
      // couldn't send but data is written to buffer
      result = HAL_OK;
    }

    // must return busy becuase the buffer is not copied
    if (recovery)
    {
      result = HAL_BUSY;
    }
  }
  else
  {
    // inc - early stop
    *tx_idx = (*tx_idx + 1) % tx_buffer_len;
  }

  return result;
}

HAL_StatusTypeDef direct_send(uint8_t *buffer, uint16_t len, UART_HandleTypeDef *huart)
{
  return HAL_UART_Transmit_DMA(huart, buffer, len);
}

HAL_StatusTypeDef usart_process_data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t len)
{
  if (len > 0)
  {
    if (huart->Instance == USART1)
    {
      // sending commands ending with \n
      return cmd_send(data, len, u2_tx_buffer, &u2_tx_idx, xx_buffer_len, &huart2);
    }
    else if (huart->Instance == USART2)
    {
      return direct_send(data, len, &huart1);
    }
  }

  return HAL_OK;
}

void HAL_SYSTICK_Callback(void)
{
  uint32_t tick = HAL_GetTick();

  if (tick % 1000 == 0)
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  if (tick % 5 != 0)
  {
    return;
  }

  usart_rx_check(&u1_old_pos, xx_buffer_len, u1_rx_buffer, &huart1, __HAL_DMA_GET_COUNTER(huart1.hdmarx));
  usart_rx_check(&u2_old_pos, xx_buffer_len, u2_rx_buffer, &huart2, __HAL_DMA_GET_COUNTER(huart2.hdmarx));
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */
  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/