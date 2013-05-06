#include "vfd.h"

#include <avr/pgmspace.h>

#define RAISE_CS() VFD_CS_PORT = VFD_CS_PORT | (1 << VFD_CS_PIN)
#define LOWER_CS() VFD_CS_PORT = VFD_CS_PORT & ~(1 << VFD_CS_PIN)

#define WAIT_TX() while(!(SPSR & (1 << SPIF)));

#define WRITE_BIT(BYTE, BIT) PORTB = PORTB 


/*
	Test code for printing out to the vacuum fluro display.
	Not sure if I'll use this in the final system
*/

void
vfd_init()
{
#if 0
	/* SS, MIS0, MOSI, SCK - set  as outputs  */
	DDRB = DDRB | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);
	/* Configure user defined chip select pin */
	VFD_CS_DDR = VFD_CS_DDR | (1 << VFD_CS_PIN);

	/* Enable SPI, Master, Mode 3 */
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (0 << SPR1) | (0 << SPR0);
	SPSR = SPSR | (1 << SPI2X);
#endif

	RAISE_CS();

	vfd_write(HD_COMMAND,0x01);
	vfd_write(HD_COMMAND,0x02);
	vfd_write(HD_COMMAND,0x06);
	vfd_write(HD_COMMAND,0x0C);
	vfd_write(HD_COMMAND,0x38);
	vfd_write(HD_COMMAND,0x80);
}

/*
 *
 * */

void
vfd_write(uint8_t command, uint8_t data)
{
	LOWER_CS();
	SPDR = command;
	WAIT_TX();
	SPDR = data;
	WAIT_TX();
	RAISE_CS();
}

void
vfd_write_buffer(const char *data)
{
	while(*data){
		vfd_write(HD_DATA, *data++);
	}
}

void
vfd_write_buffer_P(const char *data)
{
	while(pgm_read_byte(data)){
		vfd_write(HD_DATA,pgm_read_byte(data));
		data++;
	}
}

void
vfd_set_cursor(uint8_t line, uint8_t column)
{
	vfd_write(HD_COMMAND, HD_CMD_DDADDR | HD_CURSOR_SHIFT | (line + column));
}

void
vfd_clear()
{
	vfd_write(HD_COMMAND, HD_CMD_RESET);
}
