/*
 * HCSR04.c
 *
 *  Created on: Aug 8, 2022
 *      Author: Prestige
 */

#include "HCSR04.h"

void HCSR04_DistCalc(HCSR04* US){

	char Buff[100];
	uint16_t Buff_length;

	if (US->EXTI_Val2 > US->EXTI_Val1){
		US->EXTI_Diff = US->EXTI_Val2 - US->EXTI_Val1;
	}
	else if(US->EXTI_Val1 > US->EXTI_Val2){
		US->EXTI_Diff = (0xffff - US->EXTI_Val1) + US->EXTI_Val2;
	}
	US->EXTI_Diff = US->EXTI_Diff/58;
	//Buff_length = sprintf(Buff,"Distance: %d cm\r\n", US->EXTI_Diff);
	//HAL_UART_Transmit(&huart1, (uint8_t *)Buff, Buff_length, HAL_MAX_DELAY);

}
