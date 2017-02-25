/*
 * macros.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef MACROS_H_
#define MACROS_H_

/*Macros*/
/*****************************************************************************/

#define  THOUSAND    1000                                             //conversion of seconds to milliseconds
#define  TEN         10                                               //division at later stage
#define  ten_ms_LFXO 918                                              //count value for twenty eight milli seconds using low frequency oscillator
#define  ten_ms_ULFXO 28                                              //count value for twenty eight milli seconds using ultra low frequency oscillator
#define  one_ms_LFXO 16                                               //count value for one milli seconds using low frequency oscillator
#define  one_ms_ULFXO 1                                               //count value for one milli seconds using ultra low frequency oscillator
#define  LED0        0
#define  led0        2
#define  led1        3
#define GPIO_LEDARRAY_INIT {{gpioPortE,2},{gpioPortE,3}}
#define NO_OF_LEDS   2
#define VDD 3.3                                                       //Maximum value of VDD
#define PRESCALAR 2                                                   //Prescalar for the LETIMER
#define lowerthreshold 0x0100 										  //2 value in VDDLEVEL for the lowerthreshold of the lighht sensor
#define higherthreshold 0x3D00                                        //61 value in VDDLEVEL for the upperthreshold of the lighht sensor
#define CLR_VDD_LEVEL 0xffff00ff                                      //Custom define to clear 8-13 bits
#define ONE_SEC_LXFO 32768                                            //Counts for one second in LETIMER using LFXO
#define ONE_SEC_LXFO_COUNT 1                                          //repeat value of the letimer should
#define LETIMER0_IF_UF 4                                              //Overflow flag
#define TIMER0_CMD_STOP 2                                             //Stop for teh timer0,1
#define ONE_SEC_ULFRCO 1000                                           //Counts for one second in LETIMER using LFXO
#define ONE_SEC_ULFRCO_COUNT 1                                        //repeat value of the letimer should

//LED define
#define LED_PORT gpioPortE


//ACMP define
#define Light_Sensor_ACMP_Channel  acmpChannel6                      //connect the light sensor to the acmp0 channel6
#define Light_Sensor_ACMP_Ref acmpChannelVDD						 //use scaled VDD has a regferrence
#define Light_Sensor_Darkness_Ref        2  					     //2 value in VDDLEVEL for the lowerthreshold of the lighht sensor
#define Light_Sensor_Light_Ref           61                          //61 value in VDDLEVEL for the upperthreshold of the lighht sensor
#define Light_Sensor_Pin 6
#define LIGHT_SENSOR_PORT gpioPortD




//ADC define
#define ADC0_oversampling_1 1
#define NO_OF_RESBITS 12
#define NO_OF_SAMPLES_PER_SEC 75000
#define PRESCALR_CALC ((NO_OF_RESBITS + 1)*NO_OF_SAMPLES_PER_SEC)
#define NO_OF_SAMPLES 400                                          //Total number of ADC samples
#define CAL_TEMP_0 (*(volatile unsigned long *) (0x0FE081B0))       //Production calibration temperature
#define CAL_ADC_1V25 (*(volatile unsigned long *) (0x0FE081BC))     //12bit temperature output with 1.25V referrence
#define TEMP_ADC_INPUT_DMA DMAREQ_ADC0_SINGLE
#define DMA_CHANNEL 0                                               //DMA channel 0
#define ADC0_resolution adcRes12Bit
#define ADC0_reference adcRef1V25
#define ADC0_channel adcSingleInpTemp
#define ADC0_acq_time adcAcqTime1
#define ADC0_rep true
#define ADC0_warmup adcWarmupNormal
#define ADC0_DMA_Arbitration dmaArbitrate1
#define TEMP_Sensor_DMA true

DMA_CB_TypeDef ADC_cb;                                              //call back function

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




/*****************************************************************************/


#endif /* MACROS_H_ */
