/*
 * Leopard_Gecko_T2C.h
 *
 *  Created on: Nov 18, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_T2C_H_
#define LEOPARD_GECKO_T2C_H_



// Include files
/*****************************************************************************/
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
#include "Leopard_Gecko_ACMP.h"
#include "test_macros.h"
#include "my_bsp.h"
#include "Leopard_Gecko_Gesture.h"
#include "External_Lightsensor.h"



void setupI2C(void)
{
  // Using default settings
  // Enable the clock for ADC
  CMU_ClockEnable(cmuClock_I2C0, true);
  CMU_ClockEnable(cmuClock_HFPER , true);

  I2C_Init_TypeDef i2cInit = I2C_EXTERNAL_LIGHTSENSOR;

  /* Using PD6 (SDA) and PD7 (SCL) */
  GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_SDA_PORT_gpioPortD, EXTERNAL_LIGHTSENSOR_SDA_PIN_7, EXTERNAL_LIGHTSENSOR_SDA_gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(EXTERNAL_LIGHTSENSOR_SCL_PORT_gpioPortD, EXTERNAL_LIGHTSENSOR_SCL_PIN_6, EXTERNAL_LIGHTSENSOR_SCL_gpioModeWiredAndPullUpFilter, 1);


  /* Enable pins at location 1 */
  I2C0->ROUTE = I2C_ROUTE_SDAPEN |
                I2C_ROUTE_SCLPEN |
                (1 << _I2C_ROUTE_LOCATION_SHIFT); //Route to PD6-SCL,PD7-SDA

  /* Initializing the I2C */
  I2C_Init(I2C0, &i2cInit);


}



/***************************************************************************//**
* @brief setup I2C
*  Setting up the I2C with necessary port and pins
*******************************************************************************/
void setupI2C1(void)
{
  // Using default settings
  // Enable the clock for ADC
  CMU_ClockEnable(cmuClock_I2C1, true);
  CMU_ClockEnable(cmuClock_HFPER , true);
  I2C_Init_TypeDef i2cInit = I2C_EXTERNAL_LIGHTSENSOR;

  /* Using PD6 (SDA) and PD7 (SCL) */
  GPIO_PinModeSet(EXTERNAL_GESTURE_SDA_PORT_gpioPortC, EXTERNAL_GESTURE_SDA_PIN_4, EXTERNAL_GESTURE_SDA_gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(EXTERNAL_GESTURE_SCL_PORT_gpioPortC, EXTERNAL_GESTURE_SCL_PIN_5, EXTERNAL_GESTURE_SCL_gpioModeWiredAndPullUpFilter, 1);


  /* Enable pins at location 1 */
  I2C1->ROUTE = I2C_ROUTE_SDAPEN |
                I2C_ROUTE_SCLPEN; //Route to PD6-SCL,PD7-SDA

  /* Initializing the I2C */
  I2C_Init(I2C1, &i2cInit);


}


/***************************************************************************//**
* @brief I2C_datareceive_byte()
* This function is used to set receive a byte of data from the light sensor
*******************************************************************************/
int  I2C1_datareceive_byte(uint32_t address)
{

	I2C1->TXDATA = SLAVE_ADD_SHIFTED | I2C_WRITE ;          //write to  the slave address
	I2C1->CMD = I2C_CMD_START;                              //Start
	while((I2C1->IF & I2C_IF_ACK)==0);                      //Wait for ack
	I2C1->IFC = I2C_IFC_ACK;                                //clear the ack
	I2C1->TXDATA = address;                                 //transmit the data
	while((I2C1->IF & I2C_IF_ACK)==0);
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->CMD = I2C_CMD_STOP;

	I2C1->TXDATA = SLAVE_ADD_SHIFTED | I2C_READ ;
	I2C1->CMD = I2C_CMD_START;
	while((I2C1->IF & I2C_IF_ACK)==0);
	I2C1->IFC = I2C_IFC_ACK;
	while((I2C1->IF & I2C_IF_RXDATAV)==0);
	int received_byte1 = I2C1->RXDATA;
	I2C1->CMD = I2C_CMD_NACK;
	I2C1->CMD = I2C_CMD_STOP;

	return received_byte1;

}

/***************************************************************************//**
* @brief I2C_datatransfer_byte
*  Transfer the byte data to the slave by taking the address in the slave and data
*******************************************************************************/

void I2C1_datatransfer_byte(uint32_t address, uint32_t data)
{
	I2C1->TXDATA = SLAVE_ADD_SHIFTED | I2C_WRITE ;          //write to  the slave address
	I2C1->CMD = I2C_CMD_START;                              //start
	while((I2C1->IF & I2C_IF_ACK)==0);						//wait for ack
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA = address;                                 //set the address pointer
	while((I2C1->IF & I2C_IF_ACK)==0);                      //wait for ack
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA = data;                                    //send the data to be written to the data register
	while((I2C1->IF & I2C_IF_ACK)==0);                      //wait for ack
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->CMD = I2C_CMD_STOP;                               //stop
}

/***************************************************************************//**
* @brief I2C1_clear_interrupt
*  Transfer the byte data to the slave by taking the address in the slave and data
*******************************************************************************/

void I2C1_clear_interrupt(uint32_t address)
{
	I2C1->TXDATA = SLAVE_ADD_SHIFTED | I2C_WRITE ;          //write to  the slave address
	I2C1->CMD = I2C_CMD_START;                              //start
	while((I2C1->IF & I2C_IF_ACK)==0);						//wait for ack
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->TXDATA = address;                                 //set the address pointer
	while((I2C1->IF & I2C_IF_ACK)==0);                      //wait for ack
	I2C1->IFC = I2C_IFC_ACK;
	I2C1->CMD = I2C_CMD_STOP;                               //stop
}










/***************************************************************************//**
* @brief I2C_datatransfer_byte
*  Transfer the byte data to the slave by taking the address in the slave and data
*******************************************************************************/

void I2C_datatransfer_byte(uint32_t address, uint32_t data)
{
	I2C0->TXDATA = SLAVE_ADD_SHIFTED | I2C_WRITE ;          //write to  the slave address
	I2C0->CMD = I2C_CMD_START;                              //start
	while((I2C0->IF & I2C_IF_ACK)==0);						//wait for ack
	I2C0->IFC = I2C_IFC_ACK;
	I2C0->TXDATA = address;                                 //set the address pointer
	while((I2C0->IF & I2C_IF_ACK)==0);                      //wait for ack
	I2C0->IFC = I2C_IFC_ACK;
	I2C0->TXDATA = data;                                    //send the data to be written to the data register
	while((I2C0->IF & I2C_IF_ACK)==0);                      //wait for ack
	I2C0->IFC = I2C_IFC_ACK;
	I2C0->CMD = I2C_CMD_STOP;                               //stop
}




/***************************************************************************//**
* @brief I2C1_external_lightsensor_abort()
*  Abort the connection if your bus is held from previous connection
*******************************************************************************/
void I2C1_external_gesturesensor_abort()
{
	if(I2C1->STATE &I2C_STATE_BUSY)
	{
		  I2C1->CMD = I2C_CMD_ABORT;
	}
}

/***************************************************************************//**
* @brief I2C_external_lightsensor_abort()
*  Abort the connection if your bus is held from previous connection
*******************************************************************************/
void I2C_external_lightsensor_abort()
{
	if(I2C0->STATE &I2C_STATE_BUSY)
	{
		  I2C0->CMD = I2C_CMD_ABORT;
	}
}


/***************************************************************************//**
* @brief I2C_datareceive_word()
* This function is used to set receive a word of data from the I2C slave device -- light sensor
*******************************************************************************/
int* I2C_datareceive_word(int address)
{
	int received_byte[1];
	I2C0->TXDATA = SLAVE_ADD_SHIFTED | I2C_WRITE ;          //write to  the slave address
	//start--waitforack--clearack
	I2C0->CMD = I2C_CMD_START;
	while((I2C0->IF & I2C_IF_ACK)==0);
	I2C0->IFC = I2C_IFC_ACK;

	//transmit address from where the data to be read--wait for ack
	I2C0->TXDATA = address | EXTERNAL_LIGHTSENSOR_SIZE_WORD;
	while((I2C0->IF & I2C_IF_ACK)==0);
	I2C0->IFC = I2C_IFC_ACK;

    //repeated start -- transmit slave add followed by read --wait for ack -- clear ack
	I2C0->CMD = I2C_CMD_START;
	I2C0->TXDATA = SLAVE_ADD_SHIFTED | I2C_READ ;
	while((I2C0->IF & I2C_IF_ACK)==0);
	I2C0->IFC = I2C_IFC_ACK;

	//wait till you get all receive one byte
	while((I2C0->IF & I2C_IF_RXDATAV)==0);
	received_byte[0] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_ACK; //send ack

	//wait till the second byte
	while((I2C0->IF & I2C_IF_RXDATAV)==0);
	received_byte[1] = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_NACK;//send nack


	//stop
	I2C0->CMD = I2C_CMD_STOP;

	return received_byte;

}


/***************************************************************************//**
* @brief I2C_datareceive_byte()
* This function is used to set receive a byte of data from the light sensor
*******************************************************************************/
int  I2C_datareceive_byte(uint32_t address)
{
	I2C0->TXDATA = SLAVE_ADD_SHIFTED | I2C_WRITE ;          //write to  the slave address
	I2C0->CMD = I2C_CMD_START;                              //Start
	while((I2C0->IF & I2C_IF_ACK)==0);                      //Wait for ack
	I2C0->IFC = I2C_IFC_ACK;                                //clear the ack
	I2C0->TXDATA = address;                                 //transmit the data
	while((I2C0->IF & I2C_IF_ACK)==0);
	I2C0->IFC = I2C_IFC_ACK;
	I2C0->CMD = I2C_CMD_STOP;

	I2C0->TXDATA = SLAVE_ADD_SHIFTED | I2C_READ ;
	I2C0->CMD = I2C_CMD_START;
	while((I2C0->IF & I2C_IF_ACK)==0);
	I2C0->IFC = I2C_IFC_ACK;
	while((I2C0->IF & I2C_IF_RXDATAV)==0);
	int received_byte1 = I2C0->RXDATA;
	I2C0->CMD = I2C_CMD_NACK;
	I2C0->CMD = I2C_CMD_STOP;

	return received_byte1;

}



#endif /* LEOPARD_GECKO_T2C_H_ */
