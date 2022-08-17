/*
 * USART.c
 *
 *  Created on: Aug 3, 2022
 *      Author: Prestige
 */

#include "USART.h"

void UART_Print_String(char string[100]){

	  char Buff[100];
	  uint16_t Buff_length;

	  Buff_length = sprintf(Buff, string);
	  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
}

void UART_Print_Val(char string[100], int Value){

	  char Buff[100];
	  uint16_t Buff_length;

	  Buff_length = sprintf(Buff, string, Value);
	  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
}

void UART_MPU6050_ACC(float ACC, char axis){
	  char Buff[100];
	  uint16_t Buff_length;
	  switch(axis){
	  case 'x':
		  Buff_length = sprintf(Buff,"Ax=");
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  Buff_length = sprintf(Buff, "%.2f", ACC);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  break;
	  case 'y':
		  Buff_length = sprintf(Buff,"Ay=");
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  Buff_length = sprintf(Buff, "%.2f", ACC);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  break;
	  case 'z':
		  Buff_length = sprintf(Buff,"Az=");
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  Buff_length = sprintf(Buff, "%.2f", ACC);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  break;
	  default:
		  Buff_length = sprintf(Buff,"Wrong input");
	  }

	  Buff_length = sprintf(Buff,"g\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
}

void UART_MPU6050_Gyro(float Gyro, char axis){
	  char Buff[100];
	  uint16_t Buff_length;
	  switch(axis){
	  case 'x':
		  Buff_length = sprintf(Buff,"Gx=");
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  Buff_length = sprintf(Buff, "%.2f", Gyro);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  break;
	  case 'y':
		  Buff_length = sprintf(Buff,"Gy=");
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  Buff_length = sprintf(Buff, "%.2f", Gyro);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  break;
	  case 'z':
		  Buff_length = sprintf(Buff,"Gz=");
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  Buff_length = sprintf(Buff, "%.2f", Gyro);
		  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
		  break;
	  default:
		  Buff_length = sprintf(Buff,"Wrong input");
	  }

	  Buff_length = sprintf(Buff,"(°/s)\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
}
