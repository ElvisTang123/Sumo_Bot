/*
 * Remote_Control.h
 *
 *  Created on: Aug 8, 2022
 *      Author: Prestige
 */

#ifndef INC_REMOTE_CONTROL_H_
#define INC_REMOTE_CONTROL_H_

#define TIMCLOCK 96000000
#define PRESCALAR 95

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

struct RC
{
		TIM_HandleTypeDef* htim;
		uint16_t TIM_CH;

		GPIO_TypeDef* GPIO;
		uint32_t IDR_Mask;
		uint32_t IDR_Pos;

		uint16_t IC_Val1;
		uint16_t IC_Val2;

		uint16_t Diff;
		uint16_t usWidth;

//		uint16_t Width_buff;
//
//		uint8_t Edge_Flag;

} typedef RC_t;

void getWidth(RC_t* RC);

#endif /* INC_REMOTE_CONTROL_H_ */
