/*
 * Leopard_Gecko_T2C.h
 *
 *  Created on: Nov 18, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_Gesture_H_
#define LEOPARD_GECKO_Gesture_H_

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
#include "Leopard_Gecko_Gesture.h"
#include "test_macros.h"
#include "my_bsp.h"
#include "External_Lightsensor.h"
#include "Leopard_Gecko_I2C1.h"

#define GESTURE_ENABLE_pon 0x01
#define GESTURE_CONFIG_gwtime_gldrive 0x09
#define GESTURE_COUNT_gplen_gpulse 0x47
#define GESTURE_ENABLE_gen 0x41
#define GESTURE_CONFIG4_gien_gmode 0x03

#define GESTURE_ENABLE_ADD 0x80
#define GESTURE_CONFIG_ADD 0xA3
#define GESTURE_COUNT_ADD 0xA6
#define GESTURE_CONFIG4_ADD 0xAB


#define GESTURE_FIFO_UP_COUNT_ADD 0xFC
#define GESTURE_FIFO_DOWN_COUNT_ADD 0xFD
#define GESTURE_FIFO_LEFT_COUNT_ADD 0xFE
#define GESTURE_FIFO_RIGHT_COUNT_ADD 0xFF


#define GESTURE_ENABLE_aen_aien_wen 0x1A
#define GESTURE_ENABLE_aen_aien_wen_pon 0x1B
#define GESTURE_ENABLE_aen_pon 0x03
#define GESTURE_ENABLE_aien_aen_pon 0x13
#define GESTURE_ENABLE_pon 0x01


#define GESTURE_CDATAL	 0x94
#define GESTURE_CDATAH	 0x95
#define GESTURE_RDATAL   0x96
#define GESTURE_RDATAH	 0x97
#define GESTURE_GDATAL	 0x98
#define GESTURE_GDATAH	 0x99
#define GESTURE_BDATAL   0x9A
#define GESTURE_BDATAH	 0x9B

#define GESTURE_AILTL_ADD 			0x84
#define GESTURE_AILTH_ADD 			0x85
#define GESTURE_AIHTL_ADD 			0x86
#define GESTURE_AIHTH_ADD 			0x87
#define GESTURE_PERSISTANCE_ADD 	0x8C
#define GESTURE_STATUS_ADD          0x93
#define GESTURE_ACLEAR_ADD			0xE7

#define GESTURE_GAIN_ADD			0x8F
#define GESTURE_GAIN_4				3



#define GESTURE_AILTL_VAL 			0x00
#define GESTURE_AILTH_VAL 			0x00
#define GESTURE_AIHTL_VAL 			0xFF
#define GESTURE_AIHTH_VAL 			0x00
#define GESTURE_PERSISTANCE_VAL 	0x0A

//External Gesture

//SDA and SCL lines
#define EXTERNAL_GESTURE_SDA_PORT_gpioPortC gpioPortC
#define EXTERNAL_GESTURE_SDA_PIN_4 4
#define EXTERNAL_GESTURE_SDA_gpioModeWiredAndPullUpFilter gpioModeWiredAndPullUpFilter

#define EXTERNAL_GESTURE_SCL_PORT_gpioPortC gpioPortC
#define EXTERNAL_GESTURE_SCL_PIN_5 5
#define EXTERNAL_GESTURE_SCL_gpioModeWiredAndPullUpFilter gpioModeWiredAndPullUpFilter

#define EXTERNAL_GESTURE_SDA_gpioModeDisabled  gpioModeDisabled
#define EXTERNAL_GESTURE_SCL_gpioModeDisabled  gpioModeDisabled
#define EXTERNAL_GESTURE_INTERRUPT_gpioModeDisabled  gpioModeDisabled






void setup_external_gesturesensor()
{
	I2C1_external_gesturesensor_abort();
	//Powering up with I2C
	I2C1_datatransfer_byte(GESTURE_ENABLE_ADD,GESTURE_ENABLE_pon);

	//Setting the thresholds for external light sensor
	//I2C1_datatransfer_byte(GESTURE_CONFIG_ADD,GESTURE_CONFIG_gwtime_gldrive);
	//I2C1_datatransfer_byte(GESTURE_COUNT_ADD,GESTURE_COUNT_gplen_gpulse);


	I2C1_datatransfer_byte(GESTURE_ENABLE_ADD,GESTURE_ENABLE_gen);
	I2C1_datatransfer_byte(GESTURE_CONFIG4_ADD,GESTURE_CONFIG4_gien_gmode);


}

/**************************************************************************//**
  * @brief External_Gesture SensorGPIO
  * Initializes the GPIO required for external gesture sensor
  *****************************************************************************/
#define EXTERNAL_GESTURE_SENSOR_PORT gpioPortC
#define EXTERNAL_GESTURE_SENSOR_PIN  6
#define EXTERNAL_Gesture_Sensor_gpioModePushPull gpioModePushPull
#define EXTERNAL_Gesture_Sensor_Clear 0
#define EXTERNAL_GESTURE_SENSOR_PORT_gpioDriveModeLowest gpioDriveModeLowest
#define EXTERNAL_GESTURE_SENSOR_gpioModeInput gpioModeInput
#define EXTERNAL_GESTURE_SENSOR_INTERRUPT_RISINGEDGE_FALSE false
#define EXTERNAL_GESTURE_SENSOR_FALLINGEDGE_TRUE true
#define EXTERNAL_GESTURE_SENSOR_INTERRUPT_ENABLE_TRUE true
#define EXTERNAL_GESTURE_SENSOR_INTERRUPT_ENABLE_FALSE false

void External_GestureSensorGpio(void)
{
	GPIO_PinModeSet(EXTERNAL_GESTURE_SENSOR_PORT,EXTERNAL_GESTURE_SENSOR_PIN, EXTERNAL_Gesture_Sensor_gpioModePushPull, EXTERNAL_Gesture_Sensor_Clear);          //setup the light sensor excite pin
	GPIO_DriveModeSet(EXTERNAL_GESTURE_SENSOR_PORT,EXTERNAL_GESTURE_SENSOR_PORT_gpioDriveModeLowest);
	GPIO_PinOutSet(EXTERNAL_GESTURE_SENSOR_PORT,EXTERNAL_GESTURE_SENSOR_PIN);                               //Clear the excite pin to save some initial energy
}


/***************************************************************************//**
* @brief I2C_EXTERNAL_GESTURE_SENSOR_GPIO_interrupt_setup()
* This function is used to set the falling edge interrupt for pin PD1
*******************************************************************************/
void I2C1_EXTERNAL_GESTURE_SENSOR_GPIO_interrupt_setup()
{

	//GPIO_DriveModeSet(EXTERNAL_GESTURE_SENSOR_PORT, gpioDriveModeLowest);       //Set the drive for the portd D as least
	GPIO_PinModeSet(EXTERNAL_GESTURE_SENSOR_PORT, EXTERNAL_GESTURE_SENSOR_PIN, EXTERNAL_GESTURE_SENSOR_gpioModeInput, 1); //select PD1 as an input
	GPIO->IFC = CLEAR32BIT; //clear all the GPIO interrupts
	NVIC_EnableIRQ(GPIO_EVEN_IRQn); //Enable the Nested vector interrupts for GPIO
	GPIO_IntConfig(EXTERNAL_GESTURE_SENSOR_PORT,EXTERNAL_GESTURE_SENSOR_PIN,EXTERNAL_GESTURE_SENSOR_INTERRUPT_RISINGEDGE_FALSE,EXTERNAL_GESTURE_SENSOR_FALLINGEDGE_TRUE,EXTERNAL_GESTURE_SENSOR_INTERRUPT_ENABLE_TRUE);

}


void setup_external_colorsensor()
{
	I2C1_external_gesturesensor_abort();

	//Powering up with I2C
	//Setting the thresholds for external light sensor

	//I2C1_datatransfer_byte(GESTURE_CONFIG_ADD,GESTURE_CONFIG_gwtime_gldrive);
	//I2C1_datatransfer_byte(GESTURE_COUNT_ADD,GESTURE_COUNT_gplen_gpulse);

	//I2C1_datatransfer_byte(GESTURE_ENABLE_ADD,GESTURE_ENABLE_aen_aien_wen);
	//I2C1_datatransfer_byte(GESTURE_ENABLE_ADD,GESTURE_ENABLE_aen_pon);

	I2C1_datatransfer_byte(GESTURE_ENABLE_ADD,GESTURE_ENABLE_pon);

	I2C1_datatransfer_byte(GESTURE_GAIN_ADD,GESTURE_GAIN_4);
	I2C1_datatransfer_byte(GESTURE_AILTL_ADD,GESTURE_AILTL_VAL);
	I2C1_datatransfer_byte(GESTURE_AILTH_ADD,GESTURE_AILTH_VAL);
	I2C1_datatransfer_byte(GESTURE_AIHTL_ADD,GESTURE_AIHTL_VAL);
	I2C1_datatransfer_byte(GESTURE_AIHTH_ADD,GESTURE_AIHTH_VAL);


	I2C1_datatransfer_byte(GESTURE_PERSISTANCE_ADD,GESTURE_PERSISTANCE_VAL);
	I2C1_clear_interrupt(GESTURE_ACLEAR_ADD);

	I2C1_datatransfer_byte(GESTURE_ENABLE_ADD,GESTURE_ENABLE_aien_aen_pon);


	//I2C1_datatransfer_byte(GESTURE_CONFIG4_ADD,GESTURE_CONFIG4_gien_gmode);


}

void disable_Gesture_sensor_I2C1(void)
{
	//Disable the interrupt
	//Disable the clock for I2C1
	NVIC_DisableIRQ(GPIO_EVEN_IRQn); //Enable the Nested vector interrupts for GPIO
	GPIO_IntConfig(EXTERNAL_GESTURE_SENSOR_PORT,EXTERNAL_GESTURE_SENSOR_PIN,EXTERNAL_GESTURE_SENSOR_INTERRUPT_RISINGEDGE_FALSE,EXTERNAL_GESTURE_SENSOR_FALLINGEDGE_TRUE,EXTERNAL_GESTURE_SENSOR_INTERRUPT_ENABLE_FALSE);
	CMU_ClockEnable(cmuClock_I2C1, false);
}



#endif /* LEOPARD_GECKO_Gesture_H_ */
