#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

UART_HandleTypeDef huart1 = {0};

void MX_USART1_UART_Init(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    RCC_PeriphCLKInitTypeDef perclk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);

    perclk.PeriphClockSelection = RCC_PERIPHCLK_USB;
    perclk.UsbClockSelection = RCC_SYSCLKSOURCE_HSE;

    HAL_RCCEx_PeriphCLKConfig(&perclk);

    __USART1_CLK_ENABLE();
    __GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef USART_GPIO_InitStruct = {0};

    USART_GPIO_InitStruct.Pin = GPIO_PIN_9;
    USART_GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    USART_GPIO_InitStruct.Pull = GPIO_NOPULL;
    USART_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &USART_GPIO_InitStruct);

    USART_GPIO_InitStruct.Pin = GPIO_PIN_10;
    USART_GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOA, &USART_GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart1);

    RetargetInit(&huart1);
}