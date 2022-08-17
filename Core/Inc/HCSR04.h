/*
 * HCSR04.h
 *
 *  Created on: Aug 8, 2022
 *      Author: Prestige
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "USART.h"

extern UART_HandleTypeDef huart1;

struct HCSR04{

	GPIO_TypeDef* Trig_GPIO;
	uint32_t Trig_Pin;

	GPIO_TypeDef* Echo_GPIO;
	uint32_t Echo_IDR_Mask;
	uint32_t Echo_IDR_Pos;

	int16_t EXTI_Val1;
	int16_t EXTI_Val2;
	int16_t EXTI_Diff;

}typedef HCSR04;

void HCSR04_DistCalc(HCSR04* US);

#endif /* INC_HCSR04_H_ */
