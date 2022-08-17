/*
 * Remote_Control.c
 *
 *  Created on: Aug 8, 2022
 *      Author: Prestige
 */

#include "Remote_Control.h"

void getWidth(RC_t* RC){

	int refClock = TIMCLOCK/(PRESCALAR + 1);
	int mFactor = 1000000/refClock;

	if (RC->IC_Val2 > RC->IC_Val1){
		RC->Diff = RC->IC_Val2-RC->IC_Val1;
	}

	else if (RC->IC_Val1 > RC->IC_Val2){
		RC->Diff = (0xffff - RC->IC_Val1) + RC->IC_Val2;
	}
	RC->usWidth = RC->Diff * mFactor;
}
