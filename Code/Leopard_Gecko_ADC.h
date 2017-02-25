/*
 * Leopard_Gecko_ADC.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_ADC_H_
#define LEOPARD_GECKO_ADC_H_

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
#include "macros.h"





/**************************************************************************//**
 * @brief ConvertToCelsius
 * Converts the ADC reading to temperature in centigrade
 * This part/function has been taken from silicon labs documentation which was later presented in kieth Grahan lecture slides
 *****************************************************************************/
float ConvertToCelsius(int32_t adcsample)
{
	float temp;
	float cal_temp_0=(float)(( CAL_TEMP_0 & _DEVINFO_CAL_TEMP_MASK)>>_DEVINFO_CAL_TEMP_SHIFT);                                         //Get the production calibration temperature
	float cal_value_0=(float)((CAL_ADC_1V25 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)>>_DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);                     //Get the ADC value during calibration
	float t_grad = -6.27;																											   //temperature gradient
	temp = (cal_temp_0 - ((cal_value_0 - adcsample)/t_grad));                                                                          //calculate the temperature based on the current ADC value
	return temp;
}



/**************************************************************************//**
 * @brief ADC_Config
 * Initialize the ADC
 *****************************************************************************/

static void ADC_Config(void)
{

  CMU_ClockEnable(cmuClock_ADC0, true);                                       //Enable the Clock for ADC
  ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;                       //No filter,No tialgate, Warmup mode-- Normal
  ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;


  /* Init common settings for both single conversion and scan mode- */
  init.timebase=ADC_TimebaseCalc(0);
  init.ovsRateSel = ADC0_oversampling_1;

  /* Set ADC clock prescaler to 0, we are using 11MHz HFRCO, which results in HFPERCLK < 13MHz- */
  init.prescale=ADC_PrescaleCalc(PRESCALR_CALC,0);
  ADC_Init(ADC0, &init);

  //Initial VDD referrence is 1.25V
  /* Init for single conversion use, measure channel 0 with Vdd as reference. */
  singleInit.input      = adcSingleInputTemp;           //These #defines are inside an header file and easy to interpret instead of creating my own
  /* Resolution can be set lower for even more energy efficient operation. */
  singleInit.resolution = ADC0_resolution;                  //12bit Resolution

  /* Assuming we are mesuring a low impedance source we can safely use the shortest */
  /* acquisition time. */
  singleInit.acqTime = ADC0_acq_time;
  singleInit.rep = true;

  ADC_InitSingle(ADC0, &singleInit);
}


#endif /* LEOPARD_GECKO_ADC_H_ */
