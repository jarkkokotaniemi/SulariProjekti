/*
 * us100.c
 *
 *  Created on: 14.5.2022
 *      Author: Jarkko Kotaniemi
 */

#include "us100.h"


UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint16_t dist = 0;
uint8_t cmd_dist[] = {0x55};
uint8_t cmd_temp[] = {0x50};
uint8_t buffer[2] = {0};

static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

void us100_init()
{
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
}

int16_t get_dist(uint8_t sensor)
{

	switch (sensor){
	case 1:
		HAL_UART_Transmit(&huart1, cmd_dist, 1, HAL_MAX_DELAY);
		HAL_UART_Receive(&huart1, buffer, 2, HAL_MAX_DELAY);
		break;
	case 2:
		HAL_UART_Transmit(&huart2, cmd_dist, 1, HAL_MAX_DELAY);
		HAL_UART_Receive(&huart2, buffer, 2, HAL_MAX_DELAY);
		break;
	case 3:
		HAL_UART_Transmit(&huart3, cmd_dist, 1, HAL_MAX_DELAY);
		HAL_UART_Receive(&huart3, buffer, 2, HAL_MAX_DELAY);
		break;
	}

	dist = (buffer[0] << 8) + buffer[1];

	if (dist < 1000){
		return dist;
	}

	return -1;
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
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
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}
