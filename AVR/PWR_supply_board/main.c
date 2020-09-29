/*
 * PWR_supply_board.c
 *
 * Author : PaulMich
 * GITHUB: https://github.com/PaulMich/MobilePlatform
 * LICENSE: MIT License, https://github.com/PaulMich/MobilePlatform/blob/master/LICENSE
 */ 
#define F_CPU 1000000L
#include <avr/io.h>
//#include <util/delay.h>



//! This functions makes measurements on ADC1, ADC2, and ADC3 inputs and saves them into 
/*!
	provided variables.
	\param adc1 - final result of ADC1 measurement
	\param adc2 - final result of ADC2 measurement
	\param adc3 - final result of ADC3 measurement
*/
int measureADC(int *adc1, int *adc2, int *adc3)
{
	//! flags & counters used to keep track of measurments
	int numberOfSamplesInMeasurment = 10;
	
	
	//! Variables to store summarized ADC measurements
	int sum_CELL1 = 0;
	int sum_CELL2 = 0;
	int sum_CELL3 = 0;
	
	for(int samplesCounter = 0; samplesCounter < numberOfSamplesInMeasurment; samplesCounter++) 
	{
		ADMUX = _BV(REFS0);	//!< AVcc as Vref
		ADMUX |= _BV(MUX1);	 //!< 1st cell connected to PC2
		ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0); //!< enable ADC, prescaler equal to 32
		ADCSRA |= _BV(ADSC); //!< start measuring
		while(ADCSRA & _BV(ADSC)); //!< wait until the end of measurement
		sum_CELL1 += ADC; //!< add the result to summarized value
	
		ADMUX = _BV(REFS0);	
		ADMUX |= _BV(MUX0) | _BV(MUX1);	//!< 2nd cell connected to PC3
		ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);
		ADCSRA |= _BV(ADSC);
		while(ADCSRA & _BV(ADSC));
		sum_CELL2 += ADC;
	
		ADMUX = _BV(REFS0);	
		ADMUX |= _BV(MUX0);	//!< 3rd cell connected to PC1
		ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);
		ADCSRA |= _BV(ADSC);
		while(ADCSRA & _BV(ADSC));
		sum_CELL3 += ADC;
	}
	
	*adc1 = sum_CELL1 / numberOfSamplesInMeasurment; //!< compute mean value of all samples and save as final result
	*adc2 = sum_CELL2 / numberOfSamplesInMeasurment;
	*adc3 = sum_CELL3 / numberOfSamplesInMeasurment;
	
	return 0;
}

int main(void)
{
	//! Stationary power supply mode. 
	/*!
		1 - Disabled
		0 - Enabled
	*/
	DDRB &= ~_BV(1);
	PORTB |= _BV(1);
	
	
	
	//! PWR BOARD status LED, lights up when system is running
	DDRB  |= _BV(6);	
	PORTB |= _BV(6);
	
	//! ERROR red LED 
	/*!
		lights up when voltage measured on any ADC input
		is outside specified range (3.3, 4.2) [V]
	*/
	DDRB  |= _BV(2);	 
	PORTB &= ~_BV(2);
	
	//! MOSFET
	/*!
		turned on by default, turns of when any of the cells voltages
		reaches limit values.
	*/
	DDRB |= _BV(0);
	PORTB &= ~_BV(0);
	
	
	//! LED battery level indicator
	DDRD = 0xFF;
	
	PORTD |= _BV(0);
	PORTD |= _BV(1);
	PORTD |= _BV(2);
	PORTD |= _BV(3);
	PORTD |= _BV(4);
	PORTD |= _BV(5);
	PORTD |= _BV(6);
	PORTD |= _BV(7);
	
	
	//! Variables storing measurments results
	int res_CELL1 = 0;
	int res_CELL2 = 0;
	int res_CELL3 = 0;
	
	
	//! Cells voltage limits 
	int minCellVoltage = 676; //!< ~3.3[V]
	int maxCellVoltage = 870; //!< ~4.25[V]
	
	int minBatteryVoltage = minCellVoltage*3; //!< ~9.9 [V]
	int maxBatteryVoltage = maxCellVoltage*3; //!< ~12.75 [V]
	
	int voltageLevelStep = (maxBatteryVoltage - minBatteryVoltage) / 8; //!< 0.36[V]

	while (1)
	{
		measureADC(&res_CELL1, &res_CELL2, &res_CELL3);
		
		int batteryVoltage = res_CELL1 + res_CELL2 + res_CELL3;
		
		if(batteryVoltage <= minBatteryVoltage + voltageLevelStep * 1) 
		{
			PORTD |= _BV(7);	//!< red_2		= ON
			PORTD &= ~_BV(6);	//!< red_1		= OFF
			PORTD &= ~_BV(5);	//!< yellow_3	= OFF
			PORTD &= ~_BV(4);	//!< yellow_2	= OFF
			PORTD &= ~_BV(3);	//!< yellow_1	= OFF
			PORTD &= ~_BV(2);	//!< green_3	= OFF
			PORTD &= ~_BV(1);	//!< green_2	= OFF
			PORTD &= ~_BV(0);	//!< green_1	= OFF
		}
		else if(batteryVoltage <= minBatteryVoltage + voltageLevelStep * 2) 
		{
			PORTD |= _BV(7);	//!< red_2		= ON
			PORTD |= _BV(6);	//!< red_1		= ON
			PORTD &= ~_BV(5);	//!< yellow_3	= OFF
			PORTD &= ~_BV(4);	//!< yellow_2	= OFF
			PORTD &= ~_BV(3);	//!< yellow_1	= OFF
			PORTD &= ~_BV(2);	//!< green_3	= OFF
			PORTD &= ~_BV(1);	//!< green_2	= OFF
			PORTD &= ~_BV(0);	//!< green_1	= OFF
		}
		else if(batteryVoltage <= minBatteryVoltage + voltageLevelStep * 3) 
		{
			PORTD |= _BV(7);	//!< red_2		= ON
			PORTD |= _BV(6);	//!< red_1		= ON
			PORTD |= _BV(5);	//!< yellow_3	= ON
			PORTD &= ~_BV(4);	//!< yellow_2	= OFF
			PORTD &= ~_BV(3);	//!< yellow_1	= OFF
			PORTD &= ~_BV(2);	//!< green_3	= OFF
			PORTD &= ~_BV(1);	//!< green_2	= OFF
			PORTD &= ~_BV(0);	//!< green_1	= OFF
		}
		else if(batteryVoltage <= minBatteryVoltage + voltageLevelStep * 4) 
		{
			PORTD |= _BV(7);	//!< red		= ON
			PORTD |= _BV(6);	//!< yellow_1	= ON
			PORTD |= _BV(5);	//!< yellow_2	= ON
			PORTD |= _BV(4);	//!< green_1	= ON
			PORTD &= ~_BV(3);	//!< yellow_1	= OFF
			PORTD &= ~_BV(2);	//!< green_3	= OFF
			PORTD &= ~_BV(1);	//!< green_2	= OFF
			PORTD &= ~_BV(0);	//!< green_1	= OFF
		}
		else if(batteryVoltage <= minBatteryVoltage + voltageLevelStep * 5) 
		{
			PORTD |= _BV(7);	//!< red		= ON
			PORTD |= _BV(6);	//!< yellow_1	= ON
			PORTD |= _BV(5);	//!< yellow_2	= ON
			PORTD |= _BV(4);	//!< green_1	= ON
			PORTD |= _BV(3);	//!< green_2	= ON
			PORTD &= ~_BV(2);	//!< green_3	= OFF
			PORTD &= ~_BV(1);	//!< green_2	= OFF
			PORTD &= ~_BV(0);	//!< green_1	= OFF
		}
		else if(batteryVoltage <= minBatteryVoltage + voltageLevelStep * 6)
		{
			PORTD |= _BV(7);	//!< red		= ON
			PORTD |= _BV(6);	//!< yellow_1	= ON
			PORTD |= _BV(5);	//!< yellow_2	= ON
			PORTD |= _BV(4);	//!< green_1	= ON
			PORTD |= _BV(3);	//!< green_2	= ON
			PORTD |= _BV(2);	//!< green_3	= ON
			PORTD &= ~_BV(1);	//!< green_2	= OFF
			PORTD &= ~_BV(0);	//!< green_1	= OFF
		}
		else if(batteryVoltage <= minBatteryVoltage + voltageLevelStep * 7)
		{
			PORTD |= _BV(7);	//!< red		= ON
			PORTD |= _BV(6);	//!< yellow_1	= ON
			PORTD |= _BV(5);	//!< yellow_2	= ON
			PORTD |= _BV(4);	//!< green_1	= ON
			PORTD |= _BV(3);	//!< green_2	= ON
			PORTD |= _BV(2);	//!< green_3	= ON
			PORTD |= _BV(1);	//!< green_2	= ON
			PORTD &= ~_BV(0);	//!< green_1	= OFF
		}
		else if(batteryVoltage <= minBatteryVoltage + voltageLevelStep * 8) 
		{
			PORTD |= _BV(7);	//!< red		= ON
			PORTD |= _BV(6);	//!< yellow_1	= ON
			PORTD |= _BV(5);	//!< yellow_2	= ON
			PORTD |= _BV(4);	//!< green_1	= ON
			PORTD |= _BV(3);	//!< green_2	= ON
			PORTD |= _BV(2);	//!< green_3	= ON
			PORTD |= _BV(1);	//!< green_2	= ON
			PORTD |= _BV(0);	//!< green_1	= ON
		}
		
		else 
		{
			PORTD |= _BV(7);	//!< red		= ON
			PORTD |= _BV(6);	//!< yellow_1	= OFF
			PORTD |= _BV(5);	//!< yellow_2	= ON
			PORTD |= _BV(4);	//!< green_1	= OFF
			PORTD |= _BV(3);	//!< green_2	= ON
			PORTD |= _BV(2);	//!< green_3	= OFF
			PORTD |= _BV(1);	//!< green_2	= ON
			PORTD |= _BV(0);	//!< green_1	= OFF
		}
		
		if(res_CELL1 <= minCellVoltage || res_CELL2 <= minCellVoltage || res_CELL3 <= minCellVoltage
			|| res_CELL1 >= maxCellVoltage || res_CELL2 >= maxCellVoltage || res_CELL3 >= maxCellVoltage)	//!< if limits are reached by any of the cells
		{
			if((PINB & _BV(1)) == 0)
			{
				PORTB |= _BV(2);	//!< turn on robot power indicator LED
				PORTB |= _BV(0);	//!< turn on robot power supply, by switching on the MOSFET
			}
			else
			{
				PORTB &= ~_BV(2);	//!< turn off robot power indicator LED
				PORTB &= ~_BV(0);	//!< cut off robot power supply, by switching off the MOSFET
			}
		}
		else //!< if everything is OK
		{
			PORTB |= _BV(2);	//!< turn on robot power indicator LED
			PORTB |= _BV(0);	//!< turn on robot power supply, by switching on the MOSFET
		}
		
	}
}
