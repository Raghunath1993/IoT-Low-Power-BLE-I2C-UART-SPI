/*
 * my_bsp.h
 *
 *  Created on: Nov 16, 2016
 *      Author: raghunath
 */

#ifndef MY_BSP_H_
#define MY_BSP_H_

/*
 * my_bsp.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */


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
#include "test_macros.h"


/*Macros*/
/*****************************************************************************/


#define  LED0        0
#define  LED1        1
#define  led0        2
#define  led1        3
#define GPIO_LEDARRAY_INIT {{gpioPortE,2},{gpioPortE,3}}
#define NO_OF_LEDS   2


//LED define
#define LED_PORT gpioPortE




/**************************************************************************//**
 * @brief Ledset
 * Sets the LED in the dev kit
 *****************************************************************************/
void LedSet(int ledNo)
{
  if ((ledNo >= 0) && (ledNo < NO_OF_LEDS))
  {
    GPIO_PinOutSet(ledArray[ledNo].port, ledArray[ledNo].pin);             //Glow the LED by using the port and pin number using the referrence manual

  }
}



/**************************************************************************//**
 * @brief LedClear
 * Sets the LED in the dev kit
 *****************************************************************************/
void LedClear(int ledNo)
{
  if ((ledNo >= 0) && (ledNo < NO_OF_LEDS))
  {
    GPIO_PinOutClear(ledArray[ledNo].port, ledArray[ledNo].pin);             //Glow the LED by using the port and pin number using the referrence manual

  }
}



/**************************************************************************//**
 * @brief LEDsInit
 * Initailzes the LEDs
 *****************************************************************************/
void LedsInit(void)
{
  int i;
  CMU_ClockEnable(cmuClock_GPIO, true);                                           //Enable the clock for the GPIO
  for ( i=0; i<NO_OF_LEDS; i++ )
  {
    GPIO_PinModeSet(ledArray[i].port, ledArray[i].pin, gpioModePushPull, 0);     //Initializing the LEDS in push pull configuration
  }
}


/**************************************************************************//**
 * @brief LedToggle
 * Toggles the LED
 *****************************************************************************/
void LedToggle(int ledNo)
{
  if ((ledNo >= 0) && (ledNo < NO_OF_LEDS))
  {
    GPIO_PinOutToggle(ledArray[ledNo].port, ledArray[ledNo].pin);                        //Toggle the LED
  }
}



#endif /* MY_BSP_H_ */


