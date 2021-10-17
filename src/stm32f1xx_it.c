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
uint16_t u1_old_pos;
uint16_t u2_old_pos;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void cmd_send(const uint8_t *rx_buffer, uint16_t len, uint8_t *tx_buffer, uint16_t *tx_idx, uint16_t tx_buffer_len, UART_HandleTypeDef *huart);
void gather_send(const uint8_t *rx_buffer, uint16_t len, uint8_t *tx_buffer, uint16_t *tx_idx, uint16_t tx_buffer_len, UART_HandleTypeDef *huart, uint8_t is_partial);
/**
 * some codes are from these links:
 * https://stm32f4-discovery.net/2017/07/stm32-tutorial-efficiently-receive-uart-data-using-dma/
 * https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 */

/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void usart_process_data(UART_HandleTypeDef *huart, const void *data, uint16_t len, uint8_t is_partial);

/**
 * \brief           Check for new data received with DMA
 *
 * User must select context to call this function from:
 * - Only interrupts (DMA HT, DMA TC, UART IDLE) with same preemption priority level
 * - Only thread context (outside interrupts)
 *
 * If called from both context-es, exclusive access protection must be implemented
 * This mode is not advised as it usually means architecture design problems
 *
 * When IDLE interrupt is not present, application must rely only on thread context,
 * by manually calling function as quickly as possible, to make sure
 * data are read from raw buffer and processed.
 *
 * Not doing reads fast enough may cause DMA to overflow unread received bytes,
 * hence application will lost useful data.
 *
 * Solutions to this are:
 * - Improve architecture design to achieve faster reads
 * - Increase raw buffer size and allow DMA to write more data before this function is called
 */
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

  /* USER CODE END SysTick_IRQn 1 */
}

void idle_check(UART_HandleTypeDef *huart, uint16_t rx_buffer_len, uint8_t *rx_buffer, uint16_t *old_pos_ptr)
{
  uint32_t isrflags = READ_REG(huart->Instance->SR);
  uint32_t cr1its = READ_REG(huart->Instance->CR1);
  uint32_t errorflags = 0x00U;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver -------------------------------------------------*/
    if (((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
      return;
    }
  }

  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    usart_rx_check(old_pos_ptr, rx_buffer_len, rx_buffer, huart, __HAL_DMA_GET_COUNTER(huart->hdmarx));
  }
}

void usart_rx_check(uint16_t *old_pos_ptr, uint16_t rx_buffer_len, uint8_t *rx_buffer, UART_HandleTypeDef *huart, uint16_t rem)
{
  uint16_t old_pos = *old_pos_ptr;
  uint16_t pos;

  /* Calculate current position in buffer and check for new data available */
  pos = rx_buffer_len - rem;
  if (pos > old_pos)
  { /* Current position is over previous one */
    /*
      * Processing is done in "linear" mode.
      *
      * Application processing is fast with single data block,
      * length is simply calculated by subtracting pointers
      *
      * [   0   ]
      * [   1   ] <- old_pos |------------------------------------|
      * [   2   ]            |                                    |
      * [   3   ]            | Single block (len = pos - old_pos) |
      * [   4   ]            |                                    |
      * [   5   ]            |------------------------------------|
      * [   6   ] <- pos
      * [   7   ]
      * [ N - 1 ]
      */
    usart_process_data(huart, &rx_buffer[old_pos], pos - old_pos, 0);
  }
  else
  {
    /*
      * Processing is done in "overflow" mode..
      *
      * Application must process data twice,
      * since there are 2 linear memory blocks to handle
      *
      * [   0   ]            |---------------------------------|
      * [   1   ]            | Second block (len = pos)        |
      * [   2   ]            |---------------------------------|
      * [   3   ] <- pos
      * [   4   ] <- old_pos |---------------------------------|
      * [   5   ]            |                                 |
      * [   6   ]            | First block (len = N - old_pos) |
      * [   7   ]            |                                 |
      * [ N - 1 ]            |---------------------------------|
      */
    usart_process_data(huart, &rx_buffer[old_pos], rx_buffer_len - old_pos, pos > 0);
    if (pos > 0)
    {
      usart_process_data(huart, &rx_buffer[0], pos, 0);
    }
  }
  *old_pos_ptr = pos; /* Save current position as old for next transfers */
}

void cmd_send(const uint8_t *rx_buffer, uint16_t len, uint8_t *tx_buffer, uint16_t *tx_idx, uint16_t tx_buffer_len, UART_HandleTypeDef *huart)
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

  if (tx_buffer[*tx_idx] == 10) // new line
  {
    HAL_UART_Transmit_DMA(huart, (uint8_t *)tx_buffer, *tx_idx + 1);
    *tx_idx = 0;
  }
  else
  {
    // inc - early stop
    *tx_idx = (*tx_idx + 1) % tx_buffer_len;
  }
}

void gather_send(const uint8_t *rx_buffer, uint16_t len, uint8_t *tx_buffer, uint16_t *tx_idx, uint16_t tx_buffer_len, UART_HandleTypeDef *huart, uint8_t is_partial)
{
  memcpy(&tx_buffer[*tx_idx], rx_buffer, len);
  *tx_idx = (*tx_idx + len) % tx_buffer_len;

  if (!is_partial)
  {
    HAL_UART_Transmit_DMA(huart, (uint8_t *)tx_buffer, *tx_idx);
    *tx_idx = 0;
  }
}

void usart_process_data(UART_HandleTypeDef *huart, const void *data, uint16_t len, uint8_t is_partial)
{
  if (len > 0)
  {
    if (huart->Instance == USART1)
    {
      // sending commands ending with \n
      cmd_send(data, len, u2_tx_buffer, &u2_tx_idx, u2_tx_buffer_len, &huart2);
    }
    else if (huart->Instance == USART2)
    {
      gather_send(data, len, u1_tx_buffer, &u1_tx_idx, u1_tx_buffer_len, &huart1, is_partial);
    }
  }
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
  // never call usart_rx_check cause it is in command mode
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
  // no need to call usart_rx_check

  // if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
  // {
  //   return;
  // }

  // usart_rx_check(&u2_old_pos, u2_rx_buffer_len, u2_rx_buffer, &huart2, __HAL_DMA_GET_COUNTER(huart2.hdmarx));

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
  idle_check(&huart1, u1_rx_buffer_len, u1_rx_buffer, &u1_old_pos);
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
  idle_check(&huart2, u2_rx_buffer_len, u2_rx_buffer, &u2_old_pos);
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/