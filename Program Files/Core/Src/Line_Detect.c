/*
 * Line_Detect.c
 *
 *  Created on: Aug 4, 2022
 *      Author: Prestige
 */
#include "Line_Detect.h"


void LineDetect(IR_Sensor* IR){
	  char Buff[100];
	  uint16_t Buff_length;

	  if(IR->DI_GPIO->IDR & IR->IDR_Mask){
		  //Buff_length = sprintf(Buff,"Line Color: Black\r\n");
		  //HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
	  }
	  else{
		  //Buff_length = sprintf(Buff,"Line Color: White\r\n");
		  //HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);
	  }

}
