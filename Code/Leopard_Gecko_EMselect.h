/*
 * Leopard_Gecko_EMselect.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_EMSELECT_H_
#define LEOPARD_GECKO_EMSELECT_H_

#include <stdlib.h>
#include <stdio.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_acmp.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_adc.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "em_i2c.h"
#include "Leopard_Gecko_ADC.h"
#include "External_Lightsensor.h"
#include "Leopard_Gecko_loadpowermanagement.h"
#include "macros.h"
#include "Leopard_Gecko_ACMP.h"
#include "test_macros.h"

/**************************************************************************//**
 * @brief blockSleepMode
 * Helps in making sure that all peripherials in the device are in the right sleep mode
 * This part/function has been taken from silicon labs documentation which was later presented in kieth Grahan lecture slides
 *****************************************************************************/
void blockSleepMode(sleepstate_enum minimumMode)
{
	INT_Disable();                                                    //Making sure that gloabal variable doesnot get corrupted by disabling the LEDS
	sleep_block_counter[minimumMode]++;                               //Indicating that one peripherial is in that particluar sleep mode
	INT_Enable();                                                     //Enabling the interrupts
}

/**************************************************************************//**
 * @brief unblockSleepMode
 * Helps in making sure that all peripherials in the device are in the right sleep mode
 * This part/function has been taken from silicon labs documentation which was later presented in kieth Grahan lecture slides
 *****************************************************************************/
void unblockSleepMode(sleepstate_enum minimumMode)
{
	INT_Disable();                                                    //Making sure that gloabal variable doesnot get corrupted by disabling the LEDS
	if(sleep_block_counter[minimumMode]>0){
		sleep_block_counter[minimumMode]--;                          //Indicating that one peripherial is out of a particluar sleep mode
	}
	INT_Enable();                                                    //Enabling interrupts
}

/**************************************************************************//**
 * @brief Sleep
 * Device sleeps by using the variables adjusted by block sleep mode and unblock sleep mode
 * This mode part/function been taken from silicon labs documentation which was later presented in kieth Grahan lecture slides
 *****************************************************************************/
void sleep(void)
{
	if(sleep_block_counter[EM0]>0){
		return;
	}else if(sleep_block_counter[EM1]>0){
		EMU_EnterEM1();
	}else if(sleep_block_counter[EM2]>0){
		EMU_EnterEM2(true);
	}else if(sleep_block_counter[EM3]>0){
		EMU_EnterEM3(true);
	}else{
		EMU_EnterEM4();                                                   //Did not really go into this mode yet.
	}
}


void clear_sleep(void)
{
	int i=0;
	for(i=0;i<4;i++)
	{
		sleep_block_counter[i] = 0;
	}
}

#endif /* LEOPARD_GECKO_EMSELECT_H_ */
