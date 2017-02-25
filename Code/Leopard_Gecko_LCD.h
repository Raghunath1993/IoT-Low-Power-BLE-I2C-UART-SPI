/*
 * Leopard_Gecko_LCD.h
 *
 *  Created on: Nov 15, 2016
 *      Author: raghunath
 */
#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "segmentlcd.h"
#include "em_leuart.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_lcd.h"
#include <string.h>


#ifndef LEOPARD_GECKO_LCD_H_
#define LEOPARD_GECKO_LCD_H_

void initLCD()
{
	CMU_ClockEnable(cmuClock_LCD, true);
	/* Initialize LCD */
	//SegmentLCD_Init(false);
	//SegmentLCD_Write("RAGHU");
}

#endif /* LEOPARD_GECKO_LCD_H_ */
