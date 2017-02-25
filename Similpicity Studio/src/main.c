/**************************************************************************//**
 * @file
 * @brief Empty Project
 * @author Energy Micro AS
 * @version 3.20.2
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/
#include <stdlib.h>
#include <stdio.h>

#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_acmp.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_adc.h"
#include "em_dma.h"
#include "dmactrl.h"
#include "em_i2c.h"

#include "macros.h"
#include "test_macros.h"
#include "External_Lightsensor.h"
#include "my_bsp.h"

#include "Leopard_Gecko_leuart.h"
#include "Leopard_Gecko_LCD.h"
#include "Leopard_Gecko_acmp.h"
#include "Leopard_Gecko_Gesture.h"
#include "Leopard_Gecko_I2C1.h"
#include "Leopard_Gecko_lesense_letouch.h"
#include "Leopard_Gecko_ACMP.h"
#include "Leopard_Gecko_EMselect.h"
#include "Leopard_Gecko_letimer.h"
#include "Leopard_Gecko_calibrate.h"
#include "Leopard_Gecko_DMA_config.h"
#include "Leopard_Gecko_delay.h"
#include "Leopard_Gecko_ADC.h"
#include "Leopard_Gecko_loadpowermanagement.h"


#define DEVICE_ID_ADDRESS 0x92



void disable_mode2_lesense(void)
{
	 /* Enable interrupt in NVIC. */
	 NVIC_DisableIRQ(LESENSE_IRQn);
	 CMU_ClockEnable(cmuClock_LESENSE, false);

}

void enable_mode2_lesense(void)
{
	 /* Enable interrupt in NVIC. */
	 NVIC_EnableIRQ(LESENSE_IRQn);
	 CMU_ClockEnable(cmuClock_LESENSE, true);

}

void disable_acmp0()
{
	NVIC_DisableIRQ(ACMP0_IRQn);
	CMU_ClockEnable(cmuClock_ACMP0, false);    /* Disable ACMP0 clock */
}

void disable_leuart()
{
	NVIC_DisableIRQ(LEUART0_IRQn);
	CMU_ClockEnable(cmuClock_LEUART1, false);    /* Disable LEUART1 clock */
}

void disable_letimer()
{
	NVIC_DisableIRQ(LETIMER0_IRQn);
	CMU_ClockEnable(cmuClock_LETIMER0, false);    /* Disable LEUART1 clock */

}

void disable_adc()
{
	NVIC_DisableIRQ(ADC0_IRQn);
	CMU_ClockEnable(cmuClock_ADC0, false);    /* Disable LEUART1 clock */

}

void disable_dma()
{
	NVIC_DisableIRQ(DMA_IRQn);
	CMU_ClockEnable(cmuClock_DMA, false);    /* Disable LEUART1 clock */
}


void disable_I2C()
{

	//NVIC_DisableIRQ(I2C0_IRQn);
	CMU_ClockEnable(cmuClock_I2C0, false);    /* Disable I2C0 clock */
	//CMU_ClockEnable(cmuClock_I2C1, false);    /* Disable I2C1 clock */

}

void disable_power_light_sensor()
{
	Load_power_management_lightsensor_disable();
}


/**************************************************************************//**
 * @brief LETIMER0_IRQHandler
 * Interrupt Service Routine for LETIMER
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{

  /* Clear LETIMER0 rep0 interrupt flag */
  LETIMER_IntClear(LETIMER0, LETIMER_IF_REP0);
  if(LETIMER_CompareGet(LETIMER0,1) == ontime_count){

	  LETIMER_CompareSet(LETIMER0, 1,offtime_count);                           //COMP 1 value set to off count
	  LETIMER_RepeatSet(LETIMER0, 1, 1);                                       //Set the REP1 to 1
	  /*Light sensor part */
      /* Start ADC conversion as soon as we wake up. */
	  CMU_ClockEnable(cmuClock_ADC0, true);                                    //Enable the clock for ADC0

	  if(external_lightsensor_mode == 0)
	  {
		  //Enable the load
		  //Enable the GPIO_interrupt
		  //CMU_ClockEnable(cmuClock_I2C0, true);
		  disable_mode2_lesense();
		  blockSleepMode(external_lightsensor_sleep);
		  Load_power_management_lightsensor_enable();
		  external_lightsensor_mode++;
		  unblockSleepMode(external_lightsensor_sleep);
		  //CMU_ClockEnable(cmuClock_I2C0, false);
		  //blockSleepMode(lowest_sleep);
		  //Initalize the I2C

	  }
	  else if(external_lightsensor_mode == 1)
	  {
		  external_lightsensor_mode++;
		  //wait for the I2C
	  }
	  else if(external_lightsensor_mode == 2)
	  {
	  	  //wait for the I2C
		  //unblockSleepMode(lowest_sleep);
		  enable_mode2_lesense();
		  Load_power_management_lightsensor_disable();
		  external_lightsensor_mode = 0;

		  //Disable the GPIO_interrupt
		  //Disable the load

	  }


   /* DMA tranfer or LETimer */
   #ifdef DMA_ON

	  DMA_ActivateBasic(DMA_CHANNEL,                                           //Channel 0
	                        true,											   //primary
	                        false,                                             //no burst
	                        samples,                                           //RAM location
	                        (void *)((uint32_t) &(ADC0->SINGLEDATA)),		   //Source address ,ADC
	                        NO_OF_SAMPLES - 1);                                //Total number of samples
	 blockSleepMode(adc_sleep);                                                //Block sleep mode
	 ADC_Start(ADC0, adcStartSingle);                                          //Start adc

   #else
      //Using polling
	  uint16_t sampleCount=0;
	  uint32_t temper_adc_sum=0;
	  uint16_t temper_adc_avg=0;
	  float temperature;

	  /* Block the sleep mode for ADC*/
	  blockSleepMode(adc_sleep);
	  ADC_Start(ADC0, adcStartSingle);                                        //Start the ADC
	  while(sampleCount <= NO_OF_SAMPLES)									  //Take NO_OF_Smaples
	  {

		  while (((ADC0->STATUS) & (ADC_STATUS_SINGLEDV)) == 0);              //Wait till the conversion is complete
		  /* Get ADC result */
		  temper_adc_sum += ADC_DataSingleGet(ADC0);
		  sampleCount ++;
	  }

	  ADC0->CMD = ADC_CMD_SINGLESTOP;                                         //Stop the ADC
	  CMU_ClockEnable(cmuClock_ADC0, false);								  //Disable the ADC0
	  unblockSleepMode(adc_sleep);                                            //unblock sleep mode so that we can go to deep sleep
	  temper_adc_avg = temper_adc_sum/NO_OF_SAMPLES;                          //Averaging the result
	  temperature=ConvertToCelsius(temper_adc_avg);							  //converted temperature
	  if((temperature >= LOWER_TEMPERATURE_LIMIT)   && (temperature <= HIGHER_TEMPERATURE_LIMIT))
	   {
	    	GPIO_PinOutClear(LED_PORT,led1);                                 //Switch off the led
	   }
	  else
	   {
	    	GPIO_PinOutSet(LED_PORT,led1);                                  //Switch on the led
	   }
	#endif

  #ifdef INTERNAL_LIGHTSENSOR
	CMU_ClockEnable(cmuClock_ACMP0, true);								   //Enable the clock for ACMP0
	GPIO_PinOutSet(LIGHT_SENSOR_PORT,Light_Sensor_Pin);                        //Excite the light sensor
	ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE);                             //Enable the Edge interrupts
  #endif

  }
  else{
	  LETIMER_CompareSet(LETIMER0, 1, ontime_count);                       //Making sure underflow flag is set for ontime
	  LETIMER_RepeatSet(LETIMER0, 1, 1);
	  /*Light sensor part */

	  #ifdef INTERNAL_LIGHTSENSOR
	  ACMP_IntDisable(ACMP0, ACMP_IEN_EDGE);                              //Disable the Edge interrupts
	  GPIO_PinOutClear(LIGHT_SENSOR_PORT,Light_Sensor_Pin);               //Enable the edge interrupts
	  CMU_ClockEnable(cmuClock_ACMP0, false);
	  #endif
  }
}





/***************************************************************************//**
* @brief GPIO_ODD_IRQHandler()
* This function is used to receive the interrupts from the interrupts generated from odd pins and then read the value from eth ADC of the
* light sensor and compare with the threahold values and set the LED0 based on that
*******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
	uint32_t Lightsensor_ADC_low_lbyte;
	uint32_t Lightsensor_ADC_low_hbyte;
	uint32_t Lightsensor_ADC_low_word;
	int* received_data;
	//CMU_ClockEnable(cmuClock_I2C0, true);                                                          //saving power
	GPIO_IntClear(1<<1);                                                                           //Clear the interrupt flag
	received_data=I2C_datareceive_word(external_adc_0_data_byte1_add);                             //receive data
	//Use the data to form a word data
	Lightsensor_ADC_low_lbyte = *received_data;
	Lightsensor_ADC_low_hbyte = *(received_data + 1);
	Lightsensor_ADC_low_word  = Lightsensor_ADC_low_lbyte;
	Lightsensor_ADC_low_word  = Lightsensor_ADC_low_word|(Lightsensor_ADC_low_hbyte << 8);

	//compare with the upper and lower threshold values
	if((threshold_low_word_data > Lightsensor_ADC_low_word)&& (Lightsensor_ADC_low_word   != 0 ))
	{
		GPIO_PinOutSet(LED_PORT,led0);
		transmit_leuart_ble_light_on();

	}

	else if( Lightsensor_ADC_low_word > threshold_high_word_data)
	{
	   GPIO_PinOutClear(LED_PORT,led0);
	   transmit_leuart_ble_light_off();
	}

	//clear interrupts
	I2C_EXTERNAL_LIGHTSENSOR_clear_interrupt();
	//CMU_ClockEnable(cmuClock_I2C0, false);
}






/**************************************************************************//**
 * @brief ACMP0_IRQHandler
 * Looks for the compare value and appropriately sets the portE led0 value
 *****************************************************************************/
void ACMP0_IRQHandler(void) {
	int AComp0;
	ACMP0 ->IFC = ACMP_IFC_EDGE;
	AComp0 = ((ACMP0 ->STATUS & ACMP_STATUS_ACMPOUT)>> _ACMP_STATUS_ACMPOUT_SHIFT); //Reading the comparater 0 result

	if (AComp0 == 1)
	{
		//GPIO_PinOutSet(LED_PORT,led0);                                    //Set LED0
		NVIC_DisableIRQ(LESENSE_IRQn);
		ACMP_IntDisable(ACMP0, ACMP_IEN_EDGE);          //Enable edge interrupts
		transmit_leuart_classic_command_mode();
		//Interrupt  Disable
	}

}


/**************************************************************************//**
 * @brief LESENSE_IRQHandler
 * Interrupt Service Routine for LESENSE Interrupt Line
 *****************************************************************************/
void LESENSE_IRQHandler(void) {
	uint8_t channel, i, valid_touch;
	uint32_t interrupt_flags, tmp, channels_enabled;
	uint16_t threshold_value;
	uint16_t channels_touched = 0;

	/* Get interrupt flag */
	interrupt_flags = LESENSE_IntGet();
	/* Clear interrupt flag */
	LESENSE_IntClear(interrupt_flags);

	/* Interrupt handles only one channel at a time */
	/* therefore only first active channel found is handled by the interrupt. */
	for (channel = 0; channel < NUM_LESENSE_CHANNELS; channel++) {
		if ((interrupt_flags >> channel) & 0x1) {
			break;
		}
	}

	/* To filter out possible false touches, the suspected channel is measured several times */
	/* All samples should be below threshold to trigger an actual touch. */

	/* Disable other channels. */
	channels_enabled = LESENSE ->CHEN;
	LESENSE ->CHEN = 1 << channel;

	/* Evaluate VALIDATE_CNT results for touched channel. */
	valid_touch = 1;
	for (i = 0; i < VALIDATE_CNT; i++) {
		/* Start new scan and wait while active. */
		LESENSE_ScanStart();
		while (LESENSE ->STATUS & LESENSE_STATUS_SCANACTIVE)
			;

		tmp = LESENSE ->SCANRES;
		if ((tmp & (1 << channel)) == 0) {
			valid_touch = 0;
		}
	}

	/* Enable all channels again. */
	LESENSE ->CHEN = channels_enabled;

	if (valid_touch) {
		/* If logic was switched clear button flag and set logic back, else set button flag and invert logic. */
		if (LESENSE ->CH[channel].EVAL & LESENSE_CH_EVAL_COMP) {
			buttons_pressed &= ~(1 << channel);
			LESENSE ->CH[channel].EVAL &= ~LESENSE_CH_EVAL_COMP;

			threshold_value = LESENSE ->CH[channel].EVAL
					& (_LESENSE_CH_EVAL_COMPTHRES_MASK);
			/* Change threshold value 1 LSB for hysteresis. */
			threshold_value -= 1;
			LESENSE_ChannelThresSet(channel, 0, threshold_value);
		} else {
			buttons_pressed |= (1 << channel);
			LESENSE ->CH[channel].EVAL |= LESENSE_CH_EVAL_COMP;

			threshold_value = LESENSE ->CH[channel].EVAL
					& (_LESENSE_CH_EVAL_COMPTHRES_MASK);
			/* Change threshold value 1 LSB for hysteresis. */
			threshold_value += 1;
			LESENSE_ChannelThresSet(channel, 0, threshold_value);
		}
	}

	/* Need to reset RTC counter so we don't get new calibration event right after buttons are pushed/released. */
	//RTC_CounterReset();
	/* Get channels that are pressed, result is or'ed together */
	channels_touched = LETOUCH_GetChannelsTouched();
	char k=0;
	k=k+1;
	/* Check if any channels are in the touched state. */
	if (channels_touched>0) {
		//Scan started

		disable_mode2_lesense();
		disable_leuart();

		GPIO_PinOutClear(LED_PORT,led0);
		GPIO_PinOutClear(LED_PORT,led1);



		//if(overall_mode == 1)
		//{
		disable_acmp0();

		//}

		//if(overall_mode == 1)
		//{
		disable_letimer();
		disable_adc();
		disable_dma();
		disable_I2C();
		disable_power_light_sensor();


		clear_sleep();
		blockSleepMode(sleep_mode_0);
		//}

		overall_mode = 0;
		setupI2C1();
		I2C1_EXTERNAL_GESTURE_SENSOR_GPIO_interrupt_setup();
		setup_external_colorsensor();

	}
	else
	{
		//Scan Stopped
		k++;
	}

}




/***************************************************************************//**
 * @brief GPIO_EVEN_IRQHandler()
 * This function is used to receive the interrupts from the interrupts generated from odd pins and then read the value from eth ADC of the
 * light sensor and compare with the threahold values and set the LED0 based on that
 *******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
	char red_l = 0;
	char red_h = 0;
	char blue_l = 0;
	char blue_h = 0;
	char green_l = 0;
	char green_h = 0;
	char ambient_l = 0;
	char ambient_h = 0;
	int red = 0;
	int blue = 0;
	int green = 0;

	GPIO_IntClear(1 << 6);                            //Clear the interrupt flag

	red_l = I2C1_datareceive_byte(GESTURE_RDATAL);
	red_h = I2C1_datareceive_byte(GESTURE_RDATAH);

	blue_l = I2C1_datareceive_byte(GESTURE_BDATAL);
	blue_h = I2C1_datareceive_byte(GESTURE_BDATAH);

	green_l = I2C1_datareceive_byte(GESTURE_GDATAL);
	green_h = I2C1_datareceive_byte(GESTURE_GDATAH);

	ambient_l = I2C1_datareceive_byte(GESTURE_CDATAL);
	ambient_h = I2C1_datareceive_byte(GESTURE_CDATAH);

	red = red_h;
	red = red << 8;
	red |= red_l;

	blue = blue_h;
	blue = blue << 8;
	blue |= blue_l;

	green = green_h;
	green = green << 8;
	green |= green_l;

	I2C1_clear_interrupt(GESTURE_ACLEAR_ADD);

	if ((green > 150) & ((green - blue) > 50) & ((green - red) > 50))
	{
		//disable_Gesture_sensor_I2C1();
		//disable_lesense();
		//setupI2C1();
		//I2C1_EXTERNAL_GESTURE_SENSOR_GPIO_interrupt_setup();
		//setup_external_colorsensor();
		/* Calibrate the ULFRCO based on the LFXO */
//		calibrate();
		overall_mode = 1;
		GPIO_PinOutSet(LED_PORT,led1);
		disable_Gesture_sensor_I2C1();
		enable_mode2_lesense();
		ACMPInit();
		initLeuart();


	}
	if ((blue > 150) & ((blue- green) > 50) & ((blue - red) > 50))
	{

	/* Calibrate the ULFRCO based on the LFXO */
//	  calibrate();
	  overall_mode = 2;
	  GPIO_PinOutSet(LED_PORT,led1);

	  enable_mode2_lesense();

	  disable_Gesture_sensor_I2C1();

	/* DMA configuration */
	  DMAConfig();

	  /* Initialize the ADC  */
	  ADC_Config();

	  //Enable the ADC IRQ
	  NVIC_EnableIRQ(ADC0_IRQn);

	  /* Enable the light sensor GPIO*/
	  External_LightSensorGpio();
	  /* Delay of 3 milliseconds */
	  delay_3ms();
	  /*setup the I2C*/
	  setupI2C();

	  /*Setup the external light sensor*/
	  setup_external_lightsensor();

	  /*Leuart initialization */
	  initLeuart();

	  /* Specific repeat values for buffer mode */
	  LETIMER_time();

	  /* Initialize LETIMER */
	  LETIMER_setup();
	  GPIO_PinOutClear(LED_PORT,led1);


	}


}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void) {
	/* Chip errata */
	//char status;
	//char enable;
	//setupI2C();

	calibrate();
	LedsInit();

	lc_lesense_main();
	disable_mode2_lesense();

	//ACMPInit();
	setupI2C1();

	//initLeuart();
	I2C1_EXTERNAL_GESTURE_SENSOR_GPIO_interrupt_setup();
	setup_external_colorsensor();

	blockSleepMode(sleep_mode_0);

	while (1)
	{
		sleep();
	}

}
