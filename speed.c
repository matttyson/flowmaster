
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

