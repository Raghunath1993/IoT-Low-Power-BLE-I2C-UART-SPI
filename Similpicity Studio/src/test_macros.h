/*
 * test_macros.h
 *
 *  Created on: Nov 2, 2016
 *      Author: Raghunath Reddy Jangam
 */

#ifndef TEST_MACROS_H_
#define TEST_MACROS_H_

/* Macros for energy states */
/*****************************************************************************/
typedef enum sleepstate_enum { EM0,EM1,EM2,EM3} sleepstate_enum;
typedef struct
{
  GPIO_Port_TypeDef   port;
  unsigned int        pin;
} tLedArray;

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

/*****************************************************************************/
/*Test - macros & variables*/
/*****************************************************************************/
#define  timeperiod 5.00                           //Period in seconds , multiples of 10 milliseconds
#define  ontime     0.004                          //On-time in milli seconds, multiples of 10 milli seconds
#define  CALIBRATE_ENABLE 1                        //Enable the calibration for ULFRCO with LFXO
#define  DMA_ON                                    //Switch on the DMA
sleepstate_enum letimer_sleep = EM2;               //Letimer minimum sleep mode
sleepstate_enum acmp_sleep = EM1;				   //ACMP minimum sleep mode
sleepstate_enum adc_sleep = EM1;				   //ADC minimum sleep mode
sleepstate_enum leuart_sleep = EM2;				   //Leuart minimum sleep mode
sleepstate_enum external_lightsensor_sleep = EM1;
sleepstate_enum sleep_mode_0 = EM2;
sleepstate_enum lowest_sleep = EM2;
//#define INTERNAL_LIGHTSENSOR
#define LOWER_TEMPERATURE_LIMIT 15                 //Set the lower temperature in centigrade
#define HIGHER_TEMPERATURE_LIMIT 30				   //Set the higher temperature in centigrade

/*****************************************************************************/



/*Global variables*/
/*****************************************************************************/
static const tLedArray ledArray[ NO_OF_LEDS ] = GPIO_LEDARRAY_INIT;
int sleep_block_counter[6]={0,0,0,0,0,0};                                    //Initialization of all the sleep block counter
int ten_ms;
int one_ms;
uint32_t ontime_count;
uint32_t offtime_count;
uint32_t AComp0     = 0;
int k=0;                                                                     //Used for rotation of checking on and off of the light sensor
float osc_ratio=0;                                                           //Used during calibration
uint32_t ULFRCO_per_second =0;                                               //After calibration this value is updated
uint16_t samples[NO_OF_SAMPLES];                                             //Storing 1000 samples in RAM using a global variable
uint32_t external_lightsensor_mode;
int overall_mode=0;




#endif /* TEST_MACROS_H_ */
