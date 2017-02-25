/*
 * Leopard_Gecko_delay.h
 *
 *  Created on: Nov 2, 2016
 *      Author: raghunath
 */

#ifndef LEOPARD_GECKO_DELAY_H_
#define LEOPARD_GECKO_DELAY_H_

//Random delay function to generate 3ms seconds delay
void delay_3ms()
{
	int count=0;
	int i ;
	for(i=0;i<1000;i++)
	{
		count = count +1;
	}

}


#endif /* LEOPARD_GECKO_DELAY_H_ */
