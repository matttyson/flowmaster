#ifndef VFD_H
#define VFD_H

#include <inttypes.h>

/*
 *	Driver for the Samsung 20T202DA2JA VFD Display.
 *
 *	It's basicly a HD44780 with an SPI interface.
 *
 *	P1 - GND
 *	P2 - VCC
 *	P3 - SIO (Serial IO)
 *	P4 - /STB (Chip select, active low)
 *	P5 - SCK (serial clock)
 *
 * */

// Command options.
#define HD_CMD_RESET	0x01 // Clear display.
#define HD_CMD_RETURN	0x02 // Return home.
#define HD_CMD_ENTRY	0x04 // Entry Mode Set.
#define HD_CMD_DISPLAY	0x08 // Display Control.
#define HD_CMD_CURSOR	0x10 // Cursor or display shift.
#define HD_CMD_FUNCTION	0x20 // Function set.
#define HD_CMD_CGADDR	0x40 // Character generator address
#define HD_CMD_DDADDR	0x80 // DRAM address.

/* HD_CMD_ENTRY options */
#define HD_INCREMENT (0x02)
#define HD_DECREMENT 0
#define HD_SHIFT_ON 0x01
#define HD_SHIFT_OFF 0

/* HD_CMD_DISPLAY */
#define HD_DISPLAY_ON (0x04)
#define HD_DISPLAY_OFF 0
#define HD_CURSOR_ON (0x02)
#define HD_CURSOR_OFF 0
#define HD_CURSOR_BLINK (0x01)
#define HD_CURSOR_SOLID 0

/* HD_CMD_CURSOR */
#define HD_DISPLAY_SHIFT 0x08
#define HD_CURSOR_SHIFT 0
#define HD_SHIFT_RIGHT 0x04
#define HD_SHIFT_LEFT 0

/* HD_CMD_FUNCTION */
#define HD_DATA_8BIT (0x10)
#define HD_DATA_4BIT 0
#define HD_DISP_2L (0x08)
#define HD_DISP_1L 0
#define HD_FONT_5X10 (0x04)
#define HD_FONT_5X8 0

/* Command or Data packet */
#define HD_COMMAND 0xF8
#define HD_DATA    0xFA

/* To set line numbers for vfd_set_cursor */
#define HD_LINE_0 0x00
#define HD_LINE_1 0x40

/* The Chip Select pin  */
/*
#define VFD_CS_PORT PORTB
#define VFD_CS_DDR  DDRB
#define VFD_CS_PIN  PB2
*/

#define VFD_CS_PORT PORTC
#define VFD_CS_DDR  DDRC
#define VFD_CS_PIN  PC0

void vfd_init();

void vfd_set_cursor(uint8_t line, uint8_t column);
void vfd_write(uint8_t command, uint8_t data);

void vfd_write_buffer(const char *data);
void vfd_write_buffer_P(const char *data);

void vfd_clear();

#endif
