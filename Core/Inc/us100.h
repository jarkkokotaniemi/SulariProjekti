/*
 * us100.h
 *
 *  Created on: 14.5.2022
 *      Author: Jarkko Kotaniemi
 */

#ifndef INC_US100_H_
#define INC_US100_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32l1xx_hal.h"

void us100_init();

int16_t get_dist(uint8_t sensor);


#ifdef __cplusplus
}
#endif

#endif /* INC_US100_H_ */
