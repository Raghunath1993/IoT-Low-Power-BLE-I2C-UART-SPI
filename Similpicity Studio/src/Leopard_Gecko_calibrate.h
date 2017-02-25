/*
 * calibrate.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef CALIBRATE_H_
#define CALIBRATE_H_

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


/**************************************************************************//**
 * @brief calibrate
 * Helps in calibrating the ULFRCO using LFXO value.
 * FInds the true value of counts required for one second using ULFRCO for LETIMER
 *****************************************************************************/


 void calibrate()
 {

	//Local variables
  	uint32_t LFXO_count =0;
  	uint16_t LFXO_count1 =0;
  	uint16_t LFXO_count2 =0;
  	uint32_t temp=0;
  	uint32_t ULFRCO_count =0;
  	uint32_t ULFRCO_count1 =0;
  	uint32_t ULFRCO_count2 =0;


    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

    /* Enable clock for TIMER0 module */
    CMU_ClockEnable(cmuClock_TIMER0, true);

    /* Enable clock for TIMER1 module */
    CMU_ClockEnable(cmuClock_TIMER1, true);

    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Enable clock for LETIMER0 module */
    CMU_ClockEnable(cmuClock_LETIMER0, true);


    /* Select TIMER0 parameters */
    TIMER_Init_TypeDef timerInit =
    {
      .enable     = true,                              //start the timer after the init function
      .debugRun   = true,							   //enable timer in debug mode
      .clkSel     = timerClkSelHFPerClk,               //select the low frequency clock
      .fallAction = timerInputActionNone,
      .riseAction = timerInputActionNone,
      .mode       = timerModeUp,                       //count up
      .dmaClrAct  = false,
      .quadModeX4 = false,
      .oneShot    = false,
      .sync       = true,                              //Sync the timer0 with timer 1, so that starting one would start other
    };

    TIMER_Init_TypeDef timerInit1 =
      {
        .enable     = true,                          //start the timer after the init fucntion
        .debugRun   = true,                          //enable timer in debug mode
        .clkSel     = timerClkSelCascade,            //Cascade the timer with the other timers
        .fallAction = timerInputActionNone,
        .riseAction = timerInputActionNone,
        .mode       = timerModeUp,					 //count up
        .dmaClrAct  = false,
        .quadModeX4 = false,
        .oneShot    = false,
        .sync       = true,                         //Sync the timer1 with timer 0, so that starting one would start other
      };
    LETIMER_CompareSet(LETIMER0, 0, ONE_SEC_LXFO);
    LETIMER_RepeatSet(LETIMER0, 0, ONE_SEC_LXFO_COUNT);
    const LETIMER_Init_TypeDef letimerInit =
      {
      .enable         = true,                   /* Start counting when init completed. */
      .debugRun       = true,                  /* Counter shall not keep running during debug halt. */
      .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
      .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
      .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
      .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
      .repMode        = letimerRepeatOneshot,   /* One shot mode */
      };

    TIMER_Init(TIMER0, &timerInit);                               //Initialise the timer0
    TIMER_Init(TIMER1, &timerInit1);                              //Initialise the timer1
    LETIMER_Init(LETIMER0, &letimerInit);                         //Initialise the timer1

    /* wait till one second */
    while((LETIMER0->IF&LETIMER0_IF_UF)==0);                      //Check if the under flow flag is set
    TIMER0->CMD |= TIMER0_CMD_STOP;                               //Stop timer0
    TIMER1->CMD |= TIMER0_CMD_STOP;                               //Stop timer1
    LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);                    //Clear the interrupt flags generated
    LFXO_count1 = TIMER0->CNT;                                    //Load the timer0 count
    LFXO_count2 = TIMER1->CNT;                                    //Load the tiemr1 count
    LFXO_count = LFXO_count1;                                     //load into a 32 bit register
    LFXO_count = LFXO_count2<<16;							      //load the timer1 count to the main count by shifting left by 16 times

    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);           //Select the Ultra low frequency RC oscillator
    LETIMER_CompareSet(LETIMER0, 0,ONE_SEC_ULFRCO);               //reload the letimer top again with count necessary for ULFRCO
    LETIMER_RepeatSet(LETIMER0, 0, ONE_SEC_ULFRCO_COUNT);         //Select the repeat set value necessary for REP0 value.

    TIMER_Init(TIMER0, &timerInit);                               //Start the timer0
    TIMER_Init(TIMER1, &timerInit1);							  //Start the timer1
    LETIMER_Init(LETIMER0, &letimerInit);                         //Start the LEtimer


    while((LETIMER0->IF&4)==0);									  //wait for the LEtimer underflow flag
    TIMER0->CMD |= TIMER_CMD_STOP;					              //Stop the timer0
    TIMER1->CMD |= TIMER_CMD_STOP;		                          //Stop the timer1


    LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);                    //Clear the interrupt flags
    ULFRCO_count1 = TIMER0->CNT;                                  //Load the timer0 count value
    ULFRCO_count2 = TIMER1->CNT;                                  //Load the timer1 count value
    ULFRCO_count = ULFRCO_count1;                                 //Load the timer0 vlaue into a 32 but register
    temp = ULFRCO_count2<<16;                                     //Get the total count
    ULFRCO_count |= temp;                                         //Just another way of loading 32 bit value

    osc_ratio = (float)LFXO_count/(float)ULFRCO_count;            //Find the oscillator value
    ULFRCO_per_second = (osc_ratio)*1000;                         //Number of counts required for one seconds with correction

    /* Disable clock for TIMER0 module */
	CMU_ClockEnable(cmuClock_TIMER0, false);
	/* Disable clock for TIMER1 module */
	CMU_ClockEnable(cmuClock_TIMER1, false);

  }

#endif /* CALIBRATE_H_ */
