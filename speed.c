
#include "globals.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/eeprom.h>

#define ADDRESS_MIN 0
#define ADDRESS_MAX 65

static inline uint16_t
speed_read_table(uint16_t address)
{
	return eeprom_read_word(&(fan_table[address]));
}

/*
 * Regulate the fan speed according to a linear interpolation.
 *
 * This algorithim comes from Microchip Application Note AN942.
 * */
void
speed_regulate()
{
	uint16_t input;
	uint16_t output;
	uint8_t index;
	uint8_t span;
	uint8_t slope;
	uint8_t offset;

	uint16_t ft;
	uint16_t ft1;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		input = coolant_temp;
	}

	span = (uint8_t)(input & 0xF);
	index = (uint8_t)(input / 16);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		ft = speed_read_table(index);
		ft1 = speed_read_table(index+1);
	}

	slope = (uint8_t)(ft1 - ft);
	offset = (uint8_t)((slope * span) / 16);
	output = ft + offset;

	FAN_REGISTER = MIN(output, TIMER1_TOP);
}

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

