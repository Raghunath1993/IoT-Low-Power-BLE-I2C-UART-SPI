/*
 * Leopard_Gecko_acmp.h
 *
 *  Created on: Nov 17, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_ACMP_H_
#define LEOPARD_GECKO_ACMP_H_




/*
 * Leopard_Gecko_ACMP.h
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
#include "Leopard_Gecko_ACMP.h"
#include "test_macros.h"
#include "my_bsp.h"


/* Global variables */


#define IR_Sensor_VDD_scaling_Ref 29
#define IR_Sensor_ACMP_Ref acmpChannelVDD
#define IR_Sensor_ACMP_Channel acmpChannel0



/**************************************************************************//**
 * @brief ACMPInit
 * Initializes the Analog comparator
 *****************************************************************************/

void ACMPInit(void)
{
  //CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_ACMP0, true);                     //Enable the clock for the analog comparator

  const ACMP_Init_TypeDef acmp0_init =
  {
    false,                              /* No Full bias current*/
    false,                              /* No Half bias current */
    7,                                  /* Biasprog current configuration */
    true,                               /* Enable interrupt for falling edge */
    true,                               /* Enable interrupt for rising edge */
    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
    acmpHysteresisLevel1,               /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    true,                              /* Enable low power mode */
    IR_Sensor_VDD_scaling_Ref ,        /* Vdd reference scaling */
    true,                               /* Enable ACMP */
  };

  //blockSleepMode(acmp_sleep);                                   //Update the minimum sleep mode
  /* Init and set ACMP channels */

  ACMP_Init(ACMP0, &acmp0_init);
  ACMP_ChannelSet(ACMP0, IR_Sensor_ACMP_Ref,IR_Sensor_ACMP_Channel);     //Select the channel 0 of the ACMP0 as negative reference and channel 10 as positive reference
  ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE);                     //Enable edge interrupts

  /* Wait for warm up */
  while (!(ACMP0->STATUS & ACMP_STATUS_ACMPACT)) ;

  /* Enable wakeup interrupts */

  NVIC_ClearPendingIRQ(ACMP0_IRQn);
  NVIC_EnableIRQ(ACMP0_IRQn);
}


#endif /* LEOPARD_GECKO_ACMP_H_ */
