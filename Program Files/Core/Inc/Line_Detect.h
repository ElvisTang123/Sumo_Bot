/*
 * Line_Detect.h
 *
 *  Created on: Aug 4, 2022
 *      Author: Prestige
 */

#ifndef INC_LINE_DETECT_H_
#define INC_LINE_DETECT_H_

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "USART.h"

extern UART_HandleTypeDef huart1;

struct IR_Sensor{

	GPIO_TypeDef* DI_GPIO;
	uint32_t IDR_Mask;
	uint32_t IDR_Pos;

}typedef IR_Sensor;

void LineDetect(IR_Sensor* IR);

#endif /* INC_LINE_DETECT_H_ */
