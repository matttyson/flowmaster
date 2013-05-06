#include <avr/io.h>
#include <avr/interrupt.h>

#include "globals.h"

/* variables for counting RPM pulses */
volatile static uint8_t fan_tach = 0;
volatile static uint8_t pump_tach = 0;

extern volatile uint8_t update;

ISR(INT1_vect)
{
	pump_tach++;
}

ISR(TIMER2_COMPA_vect)
{
	//#define OFLOW (F_CPU / 1024 / 0xFF)
	/*
	 *	use 225 for 14.7 mhz crystal
	 *	Gives no overflow ticks.
	 * */

	/* Timer 2 overflows too fast, so we use this to make up for the lack of resolution */
	static uint8_t second_counter = OFLOW;
	second_counter--;

	if(second_counter == 0) {
		second_counter = OFLOW;
		/* Collect input from sensors and store in memory */
		fan_rpm = fan_tach;
		pump_rpm = pump_tach;
	
		/* Reset counters */
		fan_tach = 0;
		pump_tach = 0;

		/* Main function can call the update routine */
		update = 1;
	}
}

/* ADC conversion complete interrupt */
ISR(ADC_vect)
{
	/* 
	 * Store the result of the ADC conversion
	 * and switch to the next conversion target
	 *
	 * Currently the power consumption is not monitored.
	 * */
	switch(adc_current_target){
		case ADC_AMBIENT_TEMP:
			ambient_temp = ADC;
			adc_current_target = ADC_COOLANT_TEMP;
			ADMUX = (ADMUX & 0xE0) | ADC_COOLANT_REG;
			break;
		case ADC_COOLANT_TEMP:
			coolant_temp = ADC;
			adc_current_target = ADC_AMBIENT_TEMP;
			ADMUX = (ADMUX & 0xE0) | ADC_AMBIENT_REG;
			break;
		case ADC_POWER_USAGE:
			power_usage = ADC;
			break;
	}

	/* Begin the next conversion */
	ADCSRA |= (1 << ADSC);
}

