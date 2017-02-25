/*
 * External_LightSensor.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef EXTERNAL_LIGHTSENSOR_H_
#define EXTERNAL_LIGHTSENSOR_H_

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
#include "Leopard_Gecko_I2C1.h"


//External light sensor GPIO

//SDA and SCL lines
#define EXTERNAL_LIGHTSENSOR_SDA_PORT_gpioPortD gpioPortD
#define EXTERNAL_LIGHTSENSOR_SDA_PIN_7 7
#define EXTERNAL_LIGHTSENSOR_SDA_gpioModeWiredAndPullUpFilter gpioModeWiredAndPullUpFilter


#define EXTERNAL_LIGHTSENSOR_SCL_PORT_gpioPortD gpioPortD
#define EXTERNAL_LIGHTSENSOR_SCL_PIN_6 6
#define EXTERNAL_LIGHTSENSOR_SCL_gpioModeWiredAndPullUpFilter gpioModeWiredAndPullUpFilter

#define EXTERNAL_LIGHTSENSOR_SDA_gpioModeDisabled  gpioModeDisabled
#define EXTERNAL_LIGHTSENSOR_SCL_gpioModeDisabled  gpioModeDisabled
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_gpioModeDisabled  gpioModeDisabled


//External light sensor port and pins
#define EXTERNAL_LIGHT_SENSOR_PORT_gpioDriveModeLowest gpioDriveModeLowest
#define EXTERNAL_LIGHT_SENSOR_PORT gpioPortD
#define EXTERNAL_Light_Sensor_Pin 0
#define EXTERNAL_Light_Sensor_gpioModePushPull gpioModePushPull
#define EXTERNAL_Light_Sensor_gpioModePushPullDrive gpioModePushPullDrive
#define EXTERNAL_Light_Sensor_clear 0
#define EXTERNAL_Light_Sensor_Set 1

//Interrupt GPIO
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_PORTPIN GPIO_EXTIPSELL_EXTIPSEL1_PORTD
#define GPIO_EXTIRISE_1 2
#define GPIO_IEN_1 2
#define EXTERNAL_LIGHTSENSOR_gpioModeInput gpioModeInput
#define EXTERNAL_LIGHTSENSOR_gpioModeDisabled gpioModeDisabled
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_RAISINGEDGE GPIO_EXTIRISE_1
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_ENABLE GPIO_IEN_1
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_PORT gpioPortD
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_PIN 1
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_PIN_NUMBER 1
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_RISINGEDGE_FALSE false
#define EXTERNAL_LIGHTSENSOR_FALLINGEDGE_TRUE true
#define EXTERNAL_LIGHTSENSOR_INTERRUPT_ENABLE_TRUE true
#define CLEAR32BIT 0x00000000

#define SLAVE_ADD 0x39
#define SLAVE_ADD_SHIFTED SLAVE_ADD<<1
#define I2C_WRITE 0x00
#define I2C_READ 0x01
#define EXTERNAL_LIGHTSENSOR_SIZE_WORD 0x20

//External light sensor

#define I2C_EXTERNAL_CMD 0x80

//lower threshold
#define threshold_low_word_data  0x000f
#define threshold_high_word_data 0x0800

#define threshold_low_byte1_data 0x0f
#define threshold_low_byte1_add  0x02 | I2C_EXTERNAL_CMD

#define threshold_low_byte2_data 0x00
#define threshold_low_byte2_add  0x03 | I2C_EXTERNAL_CMD

//Higher threshold
#define threshold_high_byte1_data 0x00
#define threshold_high_byte1_add  0x04 | I2C_EXTERNAL_CMD

#define threshold_high_byte2_data 0x08
#define threshold_high_byte2_add  0x05 | I2C_EXTERNAL_CMD


//Information bytes for the external light sensor
#define external_adc_0_data_byte1_add 0x0c | I2C_EXTERNAL_CMD
#define external_adc_0_data_byte2_add 0x0d | I2C_EXTERNAL_CMD
#define external_adc_1_data_byte1_add 0x0e | I2C_EXTERNAL_CMD
#define external_adc_1_data_byte2_add 0x0f | I2C_EXTERNAL_CMD

//Persistance
#define interrupt_add 0x06 | I2C_EXTERNAL_CMD
#define interrupt_persistance4 4

//interrupt enabled
#define interrupt_INTREN 0x13

//Integration time
#define timing_add 0x01 | I2C_EXTERNAL_CMD
#define timing_INTEG1 0x01

//Low gain
#define timing_GAINLOW 0x01


//Power up
#define control_add 0x00 | I2C_EXTERNAL_CMD
#define control_POWER 0x03

#define LIGHT_SENSOR_clear_interrupt 0xC0

//
#define I2C0_EXTERNAL_LIGHTSENSOR_ENABLE_true true
#define I2C0_EXTERNAL_LIGHTSENSOR_MASTER_true true
#define I2C0_EXTERNAL_LIGHTSENSOR_STANDARD_FREQUENCY I2C_FREQ_STANDARD_MAX

#define I2C_EXTERNAL_LIGHTSENSOR {                                                                         \
  I2C0_EXTERNAL_LIGHTSENSOR_ENABLE_true,                    /* Enable when init done */                    \
  I2C0_EXTERNAL_LIGHTSENSOR_MASTER_true,                    /* Set to master mode */                       \
  0,                                                        /* Use currently configured reference clock */ \
  I2C0_EXTERNAL_LIGHTSENSOR_STANDARD_FREQUENCY,             /* Set to standard rate assuring being */ \
  i2cClockHLRStandard                                      /* Set to use 4:4 low/high duty cycle */       \
}

/**************************************************************************//**
 * @brief Enable_gpiopins
 * Enables all the pin connections to the board expect for the power pin connected to the external peripheral
 *****************************************************************************/

void Enable_gpiopins()
{

	  GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_SDA_PORT_gpioPortD, EXTERNAL_LIGHTSENSOR_SDA_PIN_7, EXTERNAL_LIGHTSENSOR_SDA_gpioModeWiredAndPullUpFilter, 0);
	  GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_SCL_PORT_gpioPortD, EXTERNAL_LIGHTSENSOR_SCL_PIN_6, EXTERNAL_LIGHTSENSOR_SCL_gpioModeWiredAndPullUpFilter, 0);
	  GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_INTERRUPT_PORT, EXTERNAL_LIGHTSENSOR_INTERRUPT_PIN, EXTERNAL_LIGHTSENSOR_gpioModeInput, 0);

}


/**************************************************************************//**
 * @brief Disable_gpiopins
 * Disables all the gpio pins connected to the board except for the power pin to the external pheripheral
 *****************************************************************************/
void Disable_gpiopins()
{
	GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_SDA_PORT_gpioPortD, EXTERNAL_LIGHTSENSOR_SDA_PIN_7, EXTERNAL_LIGHTSENSOR_SDA_gpioModeDisabled, 0);
    GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_SCL_PORT_gpioPortD, EXTERNAL_LIGHTSENSOR_SCL_PIN_6, EXTERNAL_LIGHTSENSOR_SCL_gpioModeDisabled, 0);
    GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_INTERRUPT_PORT, EXTERNAL_LIGHTSENSOR_INTERRUPT_PIN, EXTERNAL_LIGHTSENSOR_INTERRUPT_gpioModeDisabled, 0);

}


/**************************************************************************//**
  * @brief External_LightSensorGPIO
  * Initializes the GPIO required for external light sensor
  *****************************************************************************/

void External_LightSensorGpio(void)
{
	GPIO_PinModeSet(EXTERNAL_LIGHT_SENSOR_PORT,EXTERNAL_Light_Sensor_Pin, EXTERNAL_Light_Sensor_gpioModePushPull, EXTERNAL_Light_Sensor_clear);          //setup the light sensor excite pin
	GPIO_DriveModeSet(EXTERNAL_LIGHT_SENSOR_PORT,EXTERNAL_LIGHT_SENSOR_PORT_gpioDriveModeLowest);
	GPIO_PinOutSet(EXTERNAL_LIGHT_SENSOR_PORT,EXTERNAL_Light_Sensor_Pin);                               //Clear the excite pin to save some initial energy
}


/***************************************************************************//**
* @brief I2C_EXTERNAL_LIGHTSENSOR_GPIO_interrupt_setup()
* This function is used to set the falling edge interrupt for pin PD1
*******************************************************************************/
void I2C_EXTERNAL_LIGHTSENSOR_GPIO_interrupt_setup()
{
//	GPIO->EXTIPSELL = EXTERNAL_LIGHTSENSOR_INTERRUPT_PORTPIN;
//	GPIO->EXTIRISE = EXTERNAL_LIGHTSENSOR_INTERRUPT_RAISINGEDGE;
//	GPIO->IEN =  EXTERNAL_LIGHTSENSOR_INTERRUPT_ENABLE;
	GPIO_DriveModeSet(EXTERNAL_LIGHTSENSOR_INTERRUPT_PORT, gpioDriveModeLowest);       //Set the drive for the portd D as least
	GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_INTERRUPT_PORT, EXTERNAL_LIGHTSENSOR_INTERRUPT_PIN, EXTERNAL_LIGHTSENSOR_gpioModeInput, 1); //select PD1 as an input
	GPIO->IFC = CLEAR32BIT; //clear all the GPIO interrupts
	NVIC_EnableIRQ(GPIO_ODD_IRQn); //Enable the Nested vector interrupts for GPIO
	GPIO_IntConfig(EXTERNAL_LIGHTSENSOR_INTERRUPT_PORT,EXTERNAL_LIGHTSENSOR_INTERRUPT_PIN,EXTERNAL_LIGHTSENSOR_INTERRUPT_RISINGEDGE_FALSE,EXTERNAL_LIGHTSENSOR_FALLINGEDGE_TRUE,EXTERNAL_LIGHTSENSOR_INTERRUPT_ENABLE_TRUE);
}


/***************************************************************************//**
* @brief setup_external_lightsensor()
*  Setup the Light sensor by writing the appropriate bits to its registers
*******************************************************************************/

void setup_external_lightsensor()
{
	I2C_external_lightsensor_abort();
	//Powering up with I2C
	I2C_datatransfer_byte(control_add,control_POWER);

	//Setting the thresholds for external light sensor
	I2C_datatransfer_byte(threshold_low_byte1_add,threshold_low_byte1_data);
	I2C_datatransfer_byte(threshold_low_byte2_add,threshold_low_byte2_data);
	I2C_datatransfer_byte(threshold_high_byte1_add,threshold_high_byte1_data);
	I2C_datatransfer_byte(threshold_high_byte2_add,threshold_high_byte2_data);


	//Setting persistance for external light sensor
	I2C_datatransfer_byte(interrupt_add,interrupt_persistance4);

	//Enabling the interrupt for external light sensor
	I2C_datatransfer_byte(interrupt_add,interrupt_INTREN);
	//Setting the integration time
	I2C_datatransfer_byte(timing_add,timing_INTEG1);
	//Setting the low gain
	I2C_datatransfer_byte(timing_add,timing_INTEG1);


	I2C_EXTERNAL_LIGHTSENSOR_clear_interrupt();
	I2C_EXTERNAL_LIGHTSENSOR_GPIO_interrupt_setup();

}

/***************************************************************************//**
* @brief I2C_EXTERNAL_LIGHTSENSOR_clear_interrupt()
* This function is used to clear the level triggered interrupts generated
*******************************************************************************/
void I2C_EXTERNAL_LIGHTSENSOR_clear_interrupt()
{
	I2C0->TXDATA = SLAVE_ADD_SHIFTED | I2C_WRITE ;          //write to  the slave address
	I2C0->CMD = I2C_CMD_START;                              //start
	while((I2C0->IF & I2C_IF_ACK)==0);                      //wait for ack
	I2C0->IFC = I2C_IFC_ACK;                                //clear ack
	I2C0->TXDATA = LIGHT_SENSOR_clear_interrupt;            //transmit data
	while((I2C0->IF & I2C_IF_ACK)==0);                      //wait for ack
	I2C0->IFC = I2C_IFC_ACK;                                //clear ack
	I2C0->CMD = I2C_CMD_STOP;                               //stop
}





#endif /* EXTERNAL_LIGHTSENSOR_H_ */
