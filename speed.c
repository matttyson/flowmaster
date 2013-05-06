
#include "globals.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

/*
 * Given the current temperature, regulate the pump and fan speeds.
 *
 * */

//#define SET_SPEED(x) fan_duty_cycle = (x); pump_duty_cycle = (x);

#define C_ENABLE()  TCCR0B = (1 << CS02) | (1 << CS01) | (1 << CS00)
#define C_DISABLE() TCCR0B = 0

#define INT_ENABLE()  EIMSK = EIMSK | (1 << INT0)
#define INT_DISABLE() EIMSK = EIMSK & ~(1 << INT0)

#define FAN_FULL() PINC = PINC | (1 << PC1)
#define FAN_NORM() PINC = PINC & ~(1 << PC1)

#define FAN_CLK (F_CPU / 1024)


void
speed_set_fan(uint8_t high, uint8_t low)
{
	const uint16_t temp = (high << 8) | low;
	FAN_REGISTER = MIN(TIMER1_TOP, temp);
}

void
speed_set_pump(uint8_t high, uint8_t low)
{
	const uint16_t temp = (high << 8) | low;
	PUMP_REGISTER = MIN(TIMER1_TOP, temp);
}


/*
 * To measure the speed of a 3 wire fan that is being controlled
 * via PWM we must use a method called pulse stretching.
 *
 * This is where we run the fan at 100% power for one full RPM cyce so we
 * get an accurate tacho reading.
 *
 * */

void
speed_begin_measure_rpm()
{

	/* Clear the counter */
	TCNT0 = 0;
	/* Set up the variables */
	fan_int_ctr = 4;
	fan_int_oflow = 0;
	fan_int_ticks = 0;

	/* Put the fan to 100% speed. */
	C_ENABLE();
	FAN_FULL();
	INT_ENABLE();
}

void
speed_end_measure_rpm()
{
	/*
	 * Calculate the RPM and store it in memory
	 *
	 * We know the number of clock ticks that have elapsed,
	 * and how long a clock tick is.  From this we can calculate
	 * how long it took for the fan to travel the distance, and
	 * from that the RPMs
	 *
	 * */

	uint16_t ticks = fan_int_ticks;

	SF1_CLEAR_BIT(SF1_FAN_SPEED);

	if(fan_int_oflow){
		/* Overflow condition, multiply counter by 0xff */
	//	ticks += (fan_int_oflow * 0xFF);
	//	ticks -= (fan_int_oflow * 15);
	}
	
	/* 
	 * We have the number of ticks, now how long does it take for
	 * this to take place?
	 * */
	//fan_rpm_1 = (1/(((fan_int_oflow*0xff)+ticks) / (F_CPU/1024)))*30;
//	ticks -= 50;
	//fan_rpm_1 = (1.0/((float)ticks / FAN_CLK))*30;
 	fan_rpm_1 = ticks;
	fan_rpm_2 = fan_int_oflow;
//	fan_rpm_2 = fan_int_oflow;
}

/*
 *	Calling this intterupt costs about 11 - 14 clock cycles.
 *	need to factor this in.
 * */
ISR(INT0_vect)
{
	if((--fan_int_ctr) == 0){
		/* We've measured our pulse. turn off interrupt, timer and save the counter */
		
		fan_int_ticks = TCNT0;

		C_DISABLE();
		INT_DISABLE();
		FAN_NORM();

		SF1_SET_BIT(SF1_FAN_SPEED);
	}
}

/* This adds a bunch of clock cycles each time it's called */
ISR(TIMER0_OVF_vect)
{
	++fan_int_oflow;
}
