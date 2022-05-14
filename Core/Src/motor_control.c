/*
 * motor_control.c
 *
 *  Created on: 14.5.2022
 *      Author: Jarkko Kotaniemi
 */

#include "motor_control.h"


TIM_HandleTypeDef htim3;

int16_t wheel1_distance = 0;
int16_t wheel2_distance = 0;
int16_t speed_to_wheel = 0.01; // satunnainen arvaus, pitää kalibroida.

static void MX_TIM3_Init(void);

void motor_control_init()
{
	MX_TIM3_Init();

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void set_motor_speed(int8_t speed1, int8_t speed2)
{
	if(speed1 < 0)  // wheel1 backwards
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = -speed1;
		wheel1_distance += speed1*speed_to_wheel;
	}
	else  // wheel1 forwards
	{
		TIM3->CCR1 = speed1;
		TIM3->CCR2 = 0;
		wheel1_distance += speed1*speed_to_wheel;
	}

	if(speed2 < 0)  // wheel2 backwards
	{
		TIM3->CCR3 = 0;
		TIM3->CCR4 = -speed2;
		wheel2_distance += speed2*speed_to_wheel;
	}
	else  // wheel1 forwards
	{
		TIM3->CCR3 = speed2;
		TIM3->CCR4 = 0;
		wheel2_distance += speed2*speed_to_wheel;
	}
}

static void MX_TIM3_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;	// todennäköisesti tarpeeton? rajataan pwm-taajuutta.
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}
