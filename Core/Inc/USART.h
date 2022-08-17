/*
 * USART.h
 *
 *  Created on: Aug 3, 2022
 *      Author: Prestige
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

void UART_Print_String(char string[100]);
void UART_Print_Val(char string[100], int Value);
void UART_MPU6050_ACC(float ACC, char axis);
void UART_MPU6050_Gyro(float Gyro, char axis);

#endif /* INC_USART_H_ */
