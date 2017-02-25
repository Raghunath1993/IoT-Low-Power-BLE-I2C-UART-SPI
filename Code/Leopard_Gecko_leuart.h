/*
 * Leopard_Gecko_leuart.h
 *
 *  Created on: Nov 3, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_LEUART_H_
#define LEOPARD_GECKO_LEUART_H_

#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_leuart.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_rtc.h"
#include <string.h>
#include "my_bsp.h"


//Test Variable
#define  circular_buffer 										//Enable and disable circular buffer by commenting and uncommenting

//#defines
#define  ringbuffer_length  15                                  //Length of the ring buffer/circular buffer used
#define  regularbuffer_length 5									//Length of the regular buffer


#define scan_length 200
#define CMD_length 5
#define END_length 5
#define expected_devices 10
#define length_bid 12
#define length_name 25




#define NEW_LINE 0x0A
#define CARRIAGE_RETURN 0x0D

//Global variable
char     ringbuffer_data[ringbuffer_length];
int      ringbuffer_tail=0;
int      ringbuffer_head=0;
char     regular_buffer[regularbuffer_length];
int      regularbuffer_pointer = 0;
int      classic_mode = 0;
char      verify_CMD[CMD_length];
char      verify_END[END_length];
int      classic_received = 0;
char     scan_result[scan_length];
char 	 bluetooth_ID[expected_devices][length_bid];
char 	 bluetooth_name[expected_devices][length_name];
int		 devices_found;
int 	 command_mode_start_count;


char 	raghu_ID[12]= {'D','C','5','3','6','0','2','A','7','7','7','1'};



#define BLE_LEUARTDISABLE leuartDisable
#define BLE_LEUARTENABLE leuartEnable
#define BLE_INHERIT_CLOCKFREQUENCY 0
#define BLE_BAUDRATE 9600
#define CLASSIC_BAUDRATE 115200
#define BLE_LEUARTDATABITS8 leuartDatabits8
#define BLE_LEUARTNOPARITY leuartNoParity
#define BLE_LEUARTSTOPBITS2 leuartStopbits2
#define BLE_LEUARTSTOPBITS1 leuartStopbits1

void transmit_leuart_classic_scan_mode(void);


#define COMMAND_MODE 1
#define SCAN_MODE 2
#define END_MODE 3



/* Defining the LEUART0 initialization data */
LEUART_Init_TypeDef leuart0Init =
{
  .enable   = BLE_LEUARTENABLE,             /* Activate data reception on LEUn_TX pin. */
  .refFreq  = BLE_INHERIT_CLOCKFREQUENCY,   /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = BLE_BAUDRATE,             /* Baudrate = 9600 bps */
  .databits = BLE_LEUARTDATABITS8,          /* Each LEUART frame containes 8 databits */
  .parity   = BLE_LEUARTNOPARITY,           /* No parity bits in use */
  .stopbits = BLE_LEUARTSTOPBITS1,          /* Setting the number of stop bits in a frame to 2 bitperiods */
};

/**************************************************************************//**
 * @brief  Initialize Low Energy UART 1
 *
 * Here the LEUART is initialized with the chosen settings. It is then routed
 * to location 0 to avoid conflict with the LCD pinout. Finally the GPIO mode
 * is set to push pull.
 *
 *****************************************************************************/
void initLeuart(void)
{
  /* Start LFXO, and use LFXO for low-energy modules */
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

  /* Enabling clocks, all other remain disabled */
  CMU_ClockEnable(cmuClock_CORELE, true);     /* Enable CORELE clock */
  CMU_ClockEnable(cmuClock_GPIO, true);       /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART1 clock */


  /* Reseting and initializing LEUART1 */
  LEUART_Reset(LEUART0);
  LEUART_Init(LEUART0, &leuart0Init);

  /* Route LEUART1 TX pin to DMA location 0 */
  LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN | LEUART_ROUTE_LOCATION_LOC0;

  /* Enable GPIO for LEUART1. TX is on D4 */
  GPIO_PinModeSet(gpioPortD,                /* GPIO port */
                  4,                        /* GPIO port number */
                  gpioModePushPull,         /* Pin mode is set to push pull */
                  1);                       /* High idle state */


  GPIO_PinModeSet(gpioPortD,                /* GPIO port */
                    5,                      /* GPIO port number */
                    gpioModeInputPull,
                    1);                      /* High idle state */


  /* LEUART in loop back mode */
  //LEUART0->CTRL   |=    LEUART_CTRL_LOOPBK;
  //LEUART0->CMD    |=    LEUART_CMD_RXEN;
  //LEUART0->CTRL   |=    LEUART_CTRL_AUTOTRI;
  LEUART0->IEN    |=    LEUART_IEN_RXDATAV;
  //LEUART0->IEN  |=    LEUART_IEN_TXC;
  //LEUART0->IEN  |=    LEUART_IEN_TXBL;

  //LEUART0->CMD |= LEUART_CMD_TXEN;
  NVIC_ClearPendingIRQ(LEUART0_IRQn);
  NVIC_EnableIRQ(LEUART0_IRQn);
}

void delay_classic_transmit()
{
	int k,i,j;
	for(i=0;i<100;i++)
		for(j=0;j<100;j++)
		{
			k= k+1;
		}
}


void delay_random()
{
	int i,k;
	for(i=0;i<100;i++)
	{
		k= k+1;
	}
}


void transmit_leuart_ble( char data[] , int len)
{
	int i;
	int temp_head;
	int length_remain;

	#ifdef circular_buffer

		temp_head = ringbuffer_head;
		if(ringbuffer_head == ringbuffer_length)
		{
			ringbuffer_head = 0;
			temp_head = 0;

		}
		if(ringbuffer_tail == ringbuffer_length)
		{
			ringbuffer_tail = 0;

		}
		if((ringbuffer_head - ringbuffer_tail) > 0)
		{
			length_remain = ringbuffer_length - (ringbuffer_head - ringbuffer_tail);
		}
		else if((ringbuffer_head - ringbuffer_tail) < 0)
		{
			length_remain = (ringbuffer_tail- ringbuffer_head);
		}
		else
		{
			length_remain = ringbuffer_length;
		}

		if(length_remain >= len )
		{

			for(i= temp_head;i<(len+temp_head);i++)
			{
				ringbuffer_data[i] = data[i-temp_head] ;
			}

			if(ringbuffer_tail == ringbuffer_head)
			{
				//blockSleepMode(leuart_sleep);
				//INT_Disable();
				LEUART0->IEN    &=    ~LEUART_IEN_TXC;                                 //Enable TXBL
				//NVIC_DisableIRQ(LESENSE_IRQn);
				LEUART0->TXDATA = ringbuffer_data[ringbuffer_tail];

				ringbuffer_head = ringbuffer_head + len ;
				//INT_Enable();
				LEUART0->IEN    |=    LEUART_IEN_TXBL;                                 //Enable TXBL


			}
			else
			{
				ringbuffer_head = ringbuffer_head + len ;

			}


		}
	#else
		regularbuffer_pointer = 0;
		for(i= 0;i<(len);i++)
		{
			regular_buffer[i] = data[i] ;
		}

		LEUART0->TXDATA = regular_buffer[0];


   #endif

}



void check_raghu()
{
	int i;
	int j;
	char m,n;
	int match_found = 0;
	int match_count=0;
	for(i=0;i<devices_found;i++)
	{
		for(j=0;j<length_bid;j++)
		{
			if( bluetooth_ID[i][j] == raghu_ID[j])
			{
				match_count++;
			}

		}

		if(match_count == 12)
		{
			match_found = 1;
			break;
		}
		else
		{

			match_count=0;
		}

	}
	if(match_found == 1)
	{
		LedSet(LED0);
		LedClear(LED1);
	}
	else
	{
		LedSet(LED1);
		LedClear(LED0);
	}


}


void transmit_leuart_classic_end_mode()
{
	char end_mode[5];
	int length;
	length = 5;
	end_mode[0] = '-';
	end_mode[1] = '-';
	end_mode[2] = '-';
	end_mode[3] = 0x0D;
	end_mode[4] = 0;
	classic_mode = END_MODE;
	classic_received = 0 ;
	transmit_leuart_ble(end_mode,length);
}


void LEUART0_IRQHandler()
{
	//delay_random();
	static int count_newline = 0;
	static int count_devices = 0;
	static int count_bid_name = 0;
	static char received_value;
	static int count_bid=0;
	static int count_name=0;


	if((LEUART0->IF & LEUART_IF_RXDATAV) == 4 )          //Check what is cause for the interrupt
	{
		if(classic_mode == COMMAND_MODE)
				{

					verify_CMD[classic_received]	= LEUART0->RXDATA;

					if(classic_received == 4)
					{

						transmit_leuart_classic_scan_mode();
						//LEUART0->IEN &= ~LEUART_IEN_RXDATAV;


					}
					else
					{

						classic_received = classic_received + 1;

					}

				}
				else if(classic_mode == SCAN_MODE)
				{
					scan_result[classic_received]	= LEUART0->RXDATA;

					//received_value   = LEUART0->RXDATA;
					received_value = scan_result[classic_received];

					classic_received++;

					if(count_newline < 2)
					{
						if(received_value == NEW_LINE)
						{
							count_newline++;
						}
						if(classic_received == 26)
						{
							if(((received_value >= 0x30) & (received_value <= 0x40)) == 1 )
							{
								devices_found = (received_value - 0x30);
							}
							else
							{
								devices_found = 0;
								check_raghu();
								count_newline = 0;
								classic_received = 0;
								count_devices = 0;
								transmit_leuart_classic_end_mode();
							}
						}
					}
					else
					{
						if(devices_found > count_devices)
						{
							if(received_value == NEW_LINE)
							{
								count_devices++;
								count_bid_name = 0;
								count_bid	=0;
								count_name=0;
							}
							else if(received_value == ',')
							{
								count_bid_name++;
							}
							else if(received_value != CARRIAGE_RETURN)
							{
								if(count_bid_name == 0)
								{

									bluetooth_ID[count_devices][count_bid]	 = received_value;
									count_bid ++;

								}
								else if(count_bid_name == 1)
								{
									bluetooth_name[count_devices][count_name] = received_value;
									count_name ++;
									//bluetooth_name_length[count_devices] =
								}

							 }
						 }

						else
						{
							if(received_value == NEW_LINE)
							{
								check_raghu();
								//ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE);                     		  //Enable edge interrupts
								count_newline = 0;
								classic_received = 0;
								count_devices = 0;
								transmit_leuart_classic_end_mode();


							}
						}


					}

				}

				else if(classic_mode == END_MODE)
				{
					verify_END[classic_received]	= LEUART0->RXDATA;

					if(classic_received == 2)
					{

						ACMP_IntEnable(ACMP0, ACMP_IEN_EDGE);                     		  //Enable edge interrupts
						classic_received = 0;
						//NVIC_EnableIRQ(LESENSE_IRQn);

					}
					else
					{
						classic_received++;
					}


	              }
	}

	else
	{
		if((LEUART0->IF & LEUART_IF_TXC) == LEUART_IF_TXC)
		{
			LEUART0->IFC = LEUART_IFC_TXC;
			LEUART0->IEN &= ~LEUART_IEN_TXBL;
			LEUART0->IEN |=  LEUART_IEN_TXC;

			#ifdef circular_buffer

				if(ringbuffer_tail == 15)
				{
					ringbuffer_tail = 0;
				}
				else
				{
					ringbuffer_tail = ringbuffer_tail +1;
				}

				if(ringbuffer_tail != ringbuffer_head)
				{

					//INT_Disable();

					delay_classic_transmit();
					LEUART0->TXDATA = ringbuffer_data[ringbuffer_tail];

					//INT_Enable();
				}
				else
				{
					//unblockSleepMode(leuart_sleep);
					NVIC_EnableIRQ(LESENSE_IRQn);
				}

			#else

				if (regularbuffer_pointer < (regularbuffer_length-1))
				{
					regularbuffer_pointer = regularbuffer_pointer + 1;
					LEUART0->TXDATA = regular_buffer[regularbuffer_pointer];
				}
			#endif
		}
	}
}






void transmit_leuart_ble_temperature(float *temperature)
{
	char *temp_temperature;
	char temperature_data[5];
	char temp_data[5];
	int *temp;
	int data;
	int length;
	float check;
	//temp_temperature = *temperature;

	temp  = temperature;
	data  =  *temp;

	length = 5;
	temperature_data[0] = 'T';                                        //--Temperature data
	temperature_data[1] = ((data));
	temp_data[0]        = temperature_data[1];
	temperature_data[2] = ((data)>>8);
	temp_data[1]        = temperature_data[2];
	temperature_data[3] = ((data)>>16);
	temp_data[2]        = temperature_data[3];
	temperature_data[4] = ((data)>>24);
	temp_data[3]        = temperature_data[4];
	memcpy(&check, &temp_data, sizeof(check));
	transmit_leuart_ble(temperature_data,length);

}






void transmit_leuart_classic_scan_mode()
{
	char scan_mode[5];
	int length;
	length = 5;
	scan_mode[0] = 'I';
	scan_mode[1] = ',';
	scan_mode[2] = '0';
	scan_mode[3] = '5';
	scan_mode[4] = 0x0D;
	classic_mode = SCAN_MODE;
	classic_received = 0 ;
	transmit_leuart_ble(scan_mode,length);
}


void transmit_leuart_classic_command_mode()
{
	char command_mode[5];
	int length;
	length = 5;
	command_mode[0] = '0';
	command_mode[1] = '0';
	command_mode[2] = '$';
	command_mode[3] = '$';
	command_mode[4] = '$';
	classic_received= 0;
	classic_mode	=  COMMAND_MODE;
	transmit_leuart_ble(command_mode,length);
}


void transmit_leuart_ble_light_on()
{

	char light_data[5];
	int length;
	length = 5;
	light_data[0] = 'O';
	light_data[1] = 0;
	light_data[2] = 0;
	light_data[3] = 0;
	light_data[4] = 0;
	transmit_leuart_ble(light_data,length);
}


void transmit_leuart_ble_light_off()
{
	char light_data[5];
	int length;
	length = 5;
	light_data[0] = 'F';
	light_data[1] = 0;
	light_data[2] = 0;
	light_data[3] = 0;
	light_data[4] = 0;
	transmit_leuart_ble(light_data,length);
}


#endif /* LEOPARD_GECKO_LEUART_H_ */
