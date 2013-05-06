#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/eeprom.h>
#include <util/delay.h>

#include "bootloader_protocol.h"

static void bl_init();
static uint8_t  bl_read();
static void bl_write(uint8_t);
static void bl_reset() __attribute__((noreturn));
static void bl_check();

static void bl_vfd_init();
static void bl_vfd_write(uint8_t command, uint8_t data);
static void bl_vfd_write_buffer_P(const char *data);
static void bl_vfd_set_cursor(uint8_t line, uint8_t column);
/*
 * This is largly taken from Atmel Application note AVR109
 * */

/* VFD Commands */
/* Command or Data packet */
#define HD_COMMAND 0xF8
#define HD_DATA    0xFA

/* To set line numbers for vfd_set_cursor */
#define HD_LINE_0 0x00
#define HD_LINE_1 0x40

int 
main(void)
{
	uint16_t address = 0;
	uint16_t erase = 0;
	uint16_t data;

	const static char str_erasing[] PROGMEM = "Erasing...";
	const static char str_programming[] PROGMEM = "Programming...";
	const static char str_update[] PROGMEM = "Firmware Update Mode";

	/* Configure the bootloader enable pin as an input, and check to see if it is set */
	/*
	if(bl_override == 0){
		bl_check();
	}
	*/

	bl_init();
	bl_vfd_init();
	bl_vfd_write_buffer_P(str_update);
	bl_vfd_set_cursor(HD_LINE_1,0);

	for(;;){
		const uint8_t cmd = bl_read();

		switch(cmd){
			case BL_PING: // ping
				bl_write(BL_ACK);
				break;
			case BL_SET_ADDR: // Manually set address
				address = (bl_read() << 8) | bl_read();
				bl_write(BL_ACK);
				break;
			case BL_ERASE: // Chip Erase
				bl_vfd_write(HD_COMMAND, 0x01);
				bl_vfd_set_cursor(HD_LINE_0,0);
				bl_vfd_write_buffer_P(str_erasing);
				bl_vfd_set_cursor(HD_LINE_1,0);

				for(address = 0; address < BOOTAPPEND ; address += 2){
					boot_spm_busy_wait();
					boot_page_erase(address);
					erase += 2;
					if(erase > (BOOTAPPEND / 1 / 20)){
						erase = 0;
						bl_vfd_write(HD_DATA,'*');
					}
				}
				address = 0;

				bl_write(BL_ACK);
				bl_vfd_write(HD_COMMAND, 0x01);
				bl_vfd_write_buffer_P(str_update);
				break;
			case BL_ERASE_EEPROM: // EEPROM Erase

				for(address = 0; address < EEPROM_SZ; address += 2){
					eeprom_busy_wait();
					eeprom_write_word(&address, 0);
				}
				address = 0;
				break;

			case BL_PROGRAM:
				if(address > BOOTAPPEND){
					bl_write(BL_NAK);
					break;
				}

				data = (bl_read() << 8) | bl_read();

				boot_spm_busy_wait();
				boot_page_fill(address, data);
				boot_spm_busy_wait();
				boot_page_write(address);
				address+=2;

				bl_write(BL_ACK);

				break;
			case BL_PROGRAM_MESSAGE: // show the 'programming' message
				bl_vfd_write(HD_COMMAND, 0x01);
				bl_vfd_set_cursor(HD_LINE_0,0);
				bl_vfd_write_buffer_P(str_programming);
				bl_vfd_set_cursor(HD_LINE_1,0);
				break;
			case BL_PERCENT: // Programing percentage complete
				bl_vfd_write(HD_DATA,'*');
				break;
			case BL_RESET: // reset
				bl_reset();
				break;
		}
	}
}

static void
bl_reset()
{
	//((void(*)(void))(0))();
	// abuse the watchdog timer
	wdt_enable(WDTO_250MS);
	for(;;);
}


static void 
bl_init()
{
	/* Init the serial port to default values*/
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	/* Force baud rate to makefile defined value */
#define BAUD_PRESCALE(x) (F_CPU / ((x) * 16UL) - 1UL)
	UBRR0H = (uint8_t) (BAUD_PRESCALE(USART_BAUD) >> 8);
	UBRR0L = (uint8_t) BAUD_PRESCALE(USART_BAUD);
#undef BAUD_PRESCALE


	/* Disable PWM, drive lines to high */
}

static uint8_t 
bl_read()
{
	while(!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

static void 
bl_write(uint8_t byte)
{
	UDR0 = byte;
	while(!(UCSR0A & (1 << TXC0)));
	UCSR0A |= (1 << TXC0);
}

#define VFD_CS_PORT PORTC
#define VFD_CS_DDR  DDRC
#define VFD_CS_PIN  PC0

#define RAISE_CS() VFD_CS_PORT = VFD_CS_PORT | (1 << VFD_CS_PIN)
#define LOWER_CS() VFD_CS_PORT = VFD_CS_PORT & ~(1 << VFD_CS_PIN)

#define WAIT_TX() while(!(SPSR & (1 << SPIF)));


static void 
bl_vfd_init()
{
	/* SS, MIS0, MOSI, SCK - set  as outputs  */
	DDRB = DDRB | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);
	/* Configure user defined chip select pin */
	VFD_CS_DDR = VFD_CS_DDR | (1 << VFD_CS_PIN);
	RAISE_CS();

	/* Enable SPI, Master, Mode 3 */
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (0 << SPR1) | (0 << SPR0);
	SPSR = SPSR | (1 << SPI2X);

	bl_vfd_write(HD_COMMAND,0x01);
	bl_vfd_write(HD_COMMAND,0x02);
	bl_vfd_write(HD_COMMAND,0x06);
	bl_vfd_write(HD_COMMAND,0x0C);
	bl_vfd_write(HD_COMMAND,0x38);
	bl_vfd_write(HD_COMMAND,0x80);
}


static void 
bl_vfd_write(uint8_t command, uint8_t data)
{
	LOWER_CS();
	SPDR = command;
	WAIT_TX();
	SPDR = data;
	WAIT_TX();
	RAISE_CS();
}

static void
bl_vfd_write_buffer_P(const char *data)
{
	while(pgm_read_byte(data)){
		bl_vfd_write(HD_DATA,pgm_read_byte(data));
		data++;
	}
}

static void
bl_vfd_set_cursor(uint8_t line, uint8_t column)
{
	bl_vfd_write(HD_COMMAND, 0x80 | (line + column));
}

static void
bl_check()
{
	DDRB = DDRB & ~(1 << PB0);
	PORTB = PORTB | (1 << PB0);

	/* Sleep a bit, allow the pull up to kick in */
	_delay_ms(200.0);
	
	/* If the pin is low, bootloader enabled */
	if(!(PINB & (1 << PB0))){
		/* Bootloader is enabled. */
		return;
	}

	/* Bootloader is not enabled, start the main program */
	((void(*)(void))(0x0000))();
}
