/*
 * motor_control.h
 *
 *  Created on: May 14, 2022
 *      Author: Jarkko Kotaniemi
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l1xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void motor_control_init();
void set_motor_speed(int8_t speed1, int8_t speed2);

#ifdef __cplusplus
}
#endif


#endif /* INC_MOTOR_CONTROL_H_ */
