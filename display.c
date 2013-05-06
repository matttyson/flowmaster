#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "tables.h"
#include "display.h"
#include "vfd.h"
#include "globals.h"

#define DISP_BUFFER_SIZE 4

static char buffer[DISP_BUFFER_SIZE];

/* the display we are currently showing to the user  */
static uint8_t display_menu = 0;

/* 21 bytes, so we have a null terminator */
#define FAN_PC_COL 6
#define FAN_RPM_COL 11
const static char DISP_RPM[2][21] PROGMEM = {
	"Fans:    %      rpm ",
	"Pump:    %      rpm "
};

const static char DISP_FLOW[2][21] PROGMEM = {
	"Flow:      lph      ",
	"Power:     ma       "
};

#define TEMP_COL 8
#define DELTA_COL 15
const static char DISP_TEMP[2][21] PROGMEM = {
	"Coolant:     c Delta",
	"Ambient:     c     c"
};

const static char * const VENDOR_STRING[] PROGMEM = {
	"Flowmaster","9000"
};

/* Convert our representation of fan speed to a percentage */
//#define FAN_TICKS_TO_RPM(x) (uint16_t) ((x) << 5)
#define FAN_TICKS_TO_RPM(x) (uint16_t) ((x) * 30)
/* Convert the duty cycle to a percentage */
#define DC_TO_PC(x) pgm_read_byte(&(dcycle_to_pc[(x)]))

static void display_write_int8(uint8_t num, uint8_t digits);
static void display_write_int16(uint16_t);
static uint8_t nibble_to_char(uint8_t byte);
void display_write_hex8(uint8_t num);
static void print_temp(uint16_t adc_val);
static void print_delta();

/*
 *
 * */


static void
display_draw(const char d[2][21])
{
	vfd_set_cursor(HD_LINE_0, 0);
	vfd_write_buffer_P(d[0]);
	vfd_set_cursor(HD_LINE_1, 0);
	vfd_write_buffer_P(d[1]);
}

static void
display_update_rpm()
{
	uint16_t temp;

	vfd_set_cursor(HD_LINE_0, FAN_RPM_COL);
//	display_write_int16(FAN_TICKS_TO_RPM(fan_rpm));
	display_write_int16(fan_rpm_1);
	vfd_set_cursor(HD_LINE_0, FAN_PC_COL);
	temp = FAN_REGISTER;
	display_write_int8(DC_TO_PC(temp),3);

	vfd_set_cursor(HD_LINE_1, FAN_RPM_COL);
//	display_write_int16(FAN_TICKS_TO_RPM(pump_rpm));
	display_write_int16(fan_rpm_2);
	vfd_set_cursor(HD_LINE_1, FAN_PC_COL);
	temp = PUMP_REGISTER;
	display_write_int8(DC_TO_PC(temp),3);
}

static void
display_update_flow()
{
}

static void
display_update_temp()
{
	/* Grab the current temps from memory */

	vfd_set_cursor(HD_LINE_0, TEMP_COL);
	print_temp(coolant_temp);
	vfd_set_cursor(HD_LINE_1, TEMP_COL);
	print_temp(ambient_temp);

	print_delta();
}

void
display_switch(uint8_t display)
{
	display_menu = display;
	switch(display){
		case DISP_MENU_RPM:
			display_draw(DISP_RPM);
			display_update_rpm();
			break;
		
		case DISP_MENU_FLOW:
			display_draw(DISP_FLOW);
			display_update_flow();
			break;

		case DISP_MENU_TEMP:
			display_draw(DISP_TEMP);
			display_update_temp();
			break;
	}
}

void
display_update()
{
	switch(display_menu){
		case DISP_MENU_RPM:
			display_update_rpm();
			break;

		case DISP_MENU_FLOW:
			display_update_flow();
			break;

		case DISP_MENU_TEMP:
			display_update_temp();
			break;
	}
}

/*
 *	TODO: Fix this, it's nasty
 * */

static void
display_write_int16(uint16_t num)
{
	uint8_t i = 0;

	if(num == 0) {
		vfd_write(HD_DATA,' ');
		vfd_write(HD_DATA,' ');
		vfd_write(HD_DATA,' ');
		vfd_write(HD_DATA,'0');
		return;
	}

	for(; i < DISP_BUFFER_SIZE; i++) {
		if(num != 0){
			uint8_t digit = num % 10;
			num = num / 10;

			buffer[i] = digit + 0x30;
		}
		else {
			buffer[i] = ' ';
		}
	}

	vfd_write(HD_DATA,buffer[3]);
	vfd_write(HD_DATA,buffer[2]);
	vfd_write(HD_DATA,buffer[1]);
	vfd_write(HD_DATA,buffer[0]);
}

void
display_write_int8(uint8_t num, uint8_t digits)
{
	uint8_t i = 0;

	if(num == 0){
		switch(digits){
			case 3:	vfd_write(HD_DATA,' ');
			case 2:	vfd_write(HD_DATA,' ');
			case 1:	vfd_write(HD_DATA,'0');
		}
		return;
	}

	
	for(; i < 3 ; i++) {
		if( num != 0){
			uint8_t digit = num % 10;
			num = num / 10;

			buffer[i] = digit + 0x30;
		}
		else {
			buffer[i] = ' ';
		}
	}
	switch(digits){
		case 3:	vfd_write(HD_DATA,buffer[2]);
		case 2:	vfd_write(HD_DATA,buffer[1]);
		case 1:	vfd_write(HD_DATA,buffer[0]);
	}
}


static uint8_t
nibble_to_char(uint8_t byte)
{
	switch(byte){
		case 0x00: return '0';
		case 0x01: return '1';
		case 0x02: return '2';
		case 0x03: return '3';
		case 0x04: return '4';
		case 0x05: return '5';
		case 0x06: return '6';
		case 0x07: return '7';
		case 0x08: return '8';
		case 0x09: return '9';
		case 0x0A: return 'A';
		case 0x0B: return 'B';
		case 0x0C: return 'C';
		case 0x0D: return 'D';
		case 0x0E: return 'E';
		case 0x0F: return 'F';
	}
	return 0;
}

void
display_write_hex8(uint8_t num)
{
	vfd_write(HD_DATA, nibble_to_char(num >> 4));
	vfd_write(HD_DATA, nibble_to_char(num & 0x0F));
}

/* A union of evil. */
union evil
{
	struct temp16 temp;
	uint16_t data;
};

static void
print_delta()
{
	union evil coolant;
	union evil ambient;
	uint8_t whole;
	uint8_t remainder;

	ambient.data = pgm_read_word(&temp_table[TEMP_ADDR(ambient_temp)]);
	coolant.data = pgm_read_word(&temp_table[TEMP_ADDR(coolant_temp)]);

	whole = coolant.temp.whole - ambient.temp.whole;
	remainder = coolant.temp.remainder - ambient.temp.remainder;

	vfd_set_cursor(HD_LINE_1, DELTA_COL);
	display_write_int8(whole,2);
	vfd_write(HD_DATA,'.');
	display_write_int8(remainder,1);
}

static void
print_temp(uint16_t adc_val)
{
	union evil data;

	data.data = pgm_read_word(&temp_table[TEMP_ADDR(adc_val)]);

	if(data.temp.sign){
		vfd_write(HD_DATA,'-');
	}
	else {
		vfd_write(HD_DATA,' ');
	}

	display_write_int8(data.temp.whole,2);
	vfd_write(HD_DATA,'.');
	display_write_int8(data.temp.remainder,1);
}
/*
	if(ambient_temp < ADC_MIN){
		return;
	}
	else if(ambient_temp > (ADC_MAX - ADC_MIN)){
		return;
	}
*/
