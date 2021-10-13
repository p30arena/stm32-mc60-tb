#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "retarget.h"

#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOC
#define LED_GPIO_CLK_ENABLE() __GPIOC_CLK_ENABLE()

extern void MX_USART1_UART_Init(void);

int main(void)
{
  HAL_Init();

  LED_GPIO_CLK_ENABLE();

  GPIO_InitTypeDef LED_GPIO_InitStruct = {0};

  LED_GPIO_InitStruct.Pin = LED_PIN;
  LED_GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  LED_GPIO_InitStruct.Pull = GPIO_PULLUP;
  LED_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(LED_GPIO_PORT, &LED_GPIO_InitStruct);

  MX_USART1_UART_Init();

  unsigned long loop_counter = 0;

  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);

    loop_counter++;
    printf("loop: %lu\r\n", loop_counter);

    HAL_Delay(1000);
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}