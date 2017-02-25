/*
 * Leopard_Gecko_loadpowermanagement.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_LOADPOWERMANAGEMENT_H_
#define LEOPARD_GECKO_LOADPOWERMANAGEMENT_H_

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
#include "macros.h"
#include "my_bsp.h"
#include "Leopard_Gecko_letimer.h"



/**************************************************************************//**
 * @brief Load Power Management
 * Manages power to the load by enabling in a pins and peripherals in a right order
 *****************************************************************************/
void Load_power_management_lightsensor_enable()
{
	GPIO_PinOutSet(EXTERNAL_LIGHT_SENSOR_PORT,EXTERNAL_Light_Sensor_Pin);      //power the external  light sensor
	delay_3ms();                                                               //delay of approx 3 ms
	Enable_gpiopins();                                                         //Enable all the other gpios
	setup_external_lightsensor();                                              //Initialize the light sensor
	GPIO->IEN |= EXTERNAL_LIGHTSENSOR_INTERRUPT_ENABLE;                        //Enable the interrupts
}

void Load_power_management_lightsensor_disable()
{
	GPIO->IEN &= (~EXTERNAL_LIGHTSENSOR_INTERRUPT_ENABLE);                   //Disable the interrupt
    Disable_gpiopins();														 //Disable the GPIOs
    GPIO_PinOutClear(EXTERNAL_LIGHT_SENSOR_PORT,EXTERNAL_Light_Sensor_Pin);  //Clear the power pin for the external light sensor
}


#endif /* LEOPARD_GECKO_LOADPOWERMANAGEMENT_H_ */
