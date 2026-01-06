/*
 * app_LED_report_error.c
 *
 *  Created on: 17-Dec-2025
 *      Author: Rajeev
 */

#include "app_includes.h"

void LED_Report8BitError(uint8_t Err)
{
	while(true)
	{
		for(int i=7; i>=0; i--)
		{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ( (Err >> i) & 0x01 ) );
			HAL_Delay(200);
		}
		/* 2 second delay before resending error code. */
		HAL_Delay(2000);
	}
}
