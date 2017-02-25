/*
 * Leopard_Gecko_DMA_config.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_DMA_CONFIG_H_
#define LEOPARD_GECKO_DMA_CONFIG_H_


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
#include "Leopard_Gecko_letimer.h"
#include "Leopard_Gecko_leuart.h"


/***************************************************************************//**
* @brief
*   Configure DMA usage for this application.
*******************************************************************************/
/***************************************************************************//**
* @brief
*   Call back function for the DMA and calculate the temperature
*******************************************************************************/
void ADCdmaTransferDone(unsigned int channel,bool primary,void *user)
{
	uint32_t temper_adc_sum=0;
	uint16_t temper_adc_avg=0;
	float temperature;
	int * intermediate;

	int i;

    DMA->IFC = DMA_IFC_CH0DONE;                                         //clear the interrupt flag
    ADC0->CMD = ADC_CMD_SINGLESTOP;								        //Stop the ADC
    CMU_ClockEnable(cmuClock_ADC0, false);						        //Disable ADC
	unblockSleepMode(adc_sleep);										//Unblock the ADC minimum sleep mode
	for(i=0;i<NO_OF_SAMPLES;i++)
	{
		temper_adc_sum += samples[i];                                   //Sum of the all the temperature values
	}
	temper_adc_avg = temper_adc_sum/NO_OF_SAMPLES;                      //Average

	temperature=ConvertToCelsius(temper_adc_avg);                       //ADC reading into temprature in celsius

	transmit_leuart_ble_temperature(&temperature);
	//transmit_leuart_ble_light_on();

	//transmit_leuart_ble_light_on();
	if((temperature >= LOWER_TEMPERATURE_LIMIT)   && (temperature <= HIGHER_TEMPERATURE_LIMIT)) //Check if the temperature is within the limits
	{
		GPIO_PinOutClear(LED_PORT,led1);                                //Clear LED1
	}
	else
	{

		GPIO_PinOutSet(LED_PORT,led1);								    //Set LED1
	}

}



static void DMAConfig(void)
{

  CMU_ClockEnable(cmuClock_DMA, true);              //Enable clock for DMA
  DMA_Init_TypeDef       dmaInit;
  DMA_CfgDescr_TypeDef   descrCfg;

  DMA_CfgChannel_TypeDef chnlCfg;
  ADC_cb.cbFunc = ADCdmaTransferDone;               //Call back function name
  ADC_cb.userPtr = NULL;
  ADC_cb.primary = true;                            //Channel 0 primary descriptor

  /* Configure general DMA issues */
  dmaInit.hprot        = 0;                         //No special privileges
  dmaInit.controlBlock = dmaControlBlock;			//DMA control block
  DMA_Init(&dmaInit);

  /* Configure DMA channel used */
  chnlCfg.highPri   = true;                         //high priority enabled
  chnlCfg.enableInt = true;                         //Enable the interrupts
  chnlCfg.select    = DMAREQ_ADC0_SINGLE;           //Select ADCO
  chnlCfg.cb        = &ADC_cb;                     // call back function details
  DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);


  descrCfg.dstInc  = dmaDataInc2;                  //half word increment
  descrCfg.srcInc  = dmaDataIncNone;			   //No increment
  descrCfg.size    = dmaDataSize2;                 //half word
  descrCfg.arbRate = ADC0_DMA_Arbitration;         //aribration value is 0
  descrCfg.hprot   = 0;                            // No special previliges
  DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);


  DMA->IFC  = DMA_IFC_CH0DONE;                    //Clear interrupts
  DMA->IEN |= DMA_IEN_CH0DONE;					  //Enable interrupts

  NVIC_ClearPendingIRQ(DMA_IRQn);                //Clear pending DMA requests
  NVIC_EnableIRQ(DMA_IRQn);						 //Enable nested DMA requets

}


#endif /* LEOPARD_GECKO_DMA_CONFIG_H_ */
