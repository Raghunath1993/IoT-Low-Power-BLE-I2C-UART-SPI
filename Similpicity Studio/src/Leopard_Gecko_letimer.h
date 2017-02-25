/*
 * Leopard_Gecko_letimer.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_LETIMER_H_
#define LEOPARD_GECKO_LETIMER_H_

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
#include "Leopard_Gecko_EMselect.h"
/**************************************************************************//**
 * @brief LETIMER_time()
 * sets the rep0 and rep1 values which is later used.
 *****************************************************************************/
void LETIMER_time()
{

	if(CALIBRATE_ENABLE == 0)
	{
		ULFRCO_per_second = 1000;                                                         //In case ULFRCO calibration is disabled
	}
	float offtime;
	if(letimer_sleep >2)
	{
		ontime_count = (ontime*1000*ULFRCO_per_second)/((PRESCALAR*2)*THOUSAND);         //on time count considering pre scalar and also ULFRCO calibration
		offtime = timeperiod-ontime;
		offtime_count = (offtime*1000*ULFRCO_per_second)/((PRESCALAR*2)*THOUSAND);       //off time count considering pre scalar and also ULFRCO calibration

	}
	else
	{
		ontime_count = ((ontime*32768)/(PRESCALAR*2));                                   //on time count considering pre scalar.
		offtime = timeperiod-ontime;
		offtime_count = ((offtime*32768)/(PRESCALAR*2));                                 //off time count considering pre scalar.
	}
}



/**************************************************************************//**
 * @brief  LETIMER_setup
 * Configures and starts the LETIMER0
 *****************************************************************************/
void LETIMER_setup(void)
{
  /* Enable necessary clocks */
  if (letimer_sleep >2)
  {
	  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);                       //Use the low frequency external oscillator and connect it LFA clock routing
  }
  else
  {
	  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);                         //Use the low frequency external oscillator and connect it LFA clock routing
  }
  CMU_ClockEnable(cmuClock_CORELE, true);

  CMU_ClockEnable(cmuClock_LETIMER0, true);									    //Enable the clock to the LETIMER0 peripheral

  /* Set initial compare values for COMP0
     COMP1 keeps it's value and is used as TOP value
     for the LETIMER.
  */
  CMU->LFAPRESC0 |= PRESCALAR<<8;

  LETIMER_CompareSet(LETIMER0, 0, ontime_count);               //Making sure underflow flag is set every 10 ms
  LETIMER_CompareSet(LETIMER0, 1, offtime_count);              //Making sure underflow flag is set every 10 ms
  LETIMER_RepeatSet(LETIMER0, 0, 1);                           //REP0 is set to on time
  LETIMER_RepeatSet(LETIMER0, 1, 1);                           //REP1 value set to off time

  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit =
  {
  .enable         = true,                   /* Start counting when init completed. */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
  .bufTop         = true,                   /* load COMP1 into COMP0 when REP0 reaches 0. */
  .repMode        = letimerRepeatBuffered   /* Repeat buffered mode */
  };
  blockSleepMode(letimer_sleep);
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit);

  /* Enable LETIMER0 interrupt vector in NVIC*/
  NVIC_EnableIRQ(LETIMER0_IRQn); //Though currently not using but as a good practice used this

  /* Enable rep0 interrupt */
  LETIMER_IntEnable(LETIMER0, LETIMER_IF_REP0);

}



#endif /* LEOPARD_GECKO_LETIMER_H_ */
