/*
 * Motor_Driver.h
 *
 *  Created on: Aug 8, 2022
 *      Author: Prestige
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

struct Motor
{
		TIM_HandleTypeDef* htim;

		GPIO_TypeDef* IN1_GPIO;
		uint16_t IN1_pin;

		GPIO_TypeDef*	IN2_GPIO;
		uint16_t	IN2_pin;

		GPIO_TypeDef*	EN_GPIO;
		uint16_t EN_pin;


} typedef Motor_t;

void	enable(Motor_t* mot);
void	disable(Motor_t* mot);
void	set_level(Motor_t* mot, int32_t level);

#endif /* INC_MOTOR_DRIVER_H_ */
