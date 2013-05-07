#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "protocol.h"
#include "serial.h"
#include "globals.h"
#include "speed.h"

static uint8_t calc_crc8(volatile uint8_t* data_pointer, uint8_t number_of_bytes);
static uint8_t validate_rx_buffer();
static void serial_begin_tx();
static void serial_send_reply(uint8_t type);
static void serial_send_adc_value();
static void serial_call_bootloader();

#define SERIAL_BUF_LENGTH 24
#define SERIAL_BUF_PAYLOAD 19

volatile static uint8_t tx_buffer[SERIAL_BUF_LENGTH];
volatile static uint8_t tx_len = 0;
volatile static uint8_t tx_ptr = 0;
volatile static uint8_t tx_checksum = 0;

volatile static uint8_t rx_buffer[SERIAL_BUF_LENGTH];
volatile static uint8_t rx_len = 0;

#define DISABLE_TX_INT() UCSR0B = UCSR0B & ~(1 << TXCIE0)
#define ENABLE_TX_INT()  UCSR0B = UCSR0B | (1 << TXCIE0)

#define DISABLE_RX_INT() UCSR0B = UCSR0B & ~(1 << RXCIE0)
#define ENABLE_RX_INT() UCSR0B = UCSR0B | (1 << RXCIE0)

/* CTS / RTS handshaking */
#define RAISE_CTS()
#define LOWER_CTS()

/* where data is stored in a packet */
#define PKT_TYPE 0
#define PKT_LENG 1
#define PKT_CSUM (rx_buffer[PKT_LENG] + 2)
#define PKT_TAIL (PKT_CSUM + 1)

/* For debugging only */
void
write_byte(uint8_t byte)
{
	while (!(UCSR0A & (1 << UDRE0)));
	UCSR0A = UCSR0A | (1 << TXC0);
	UDR0 = byte;
}

void
write_string(const char *string)
{
	while (*string) {
		write_byte(*string++);
	}
}

void
serial_process_input()
{
	if(validate_rx_buffer() != 0){
		goto cleanup;
	}

	/* 
	 * Packet has been validated.
	 * Figure out what it is and what to do with it.
	 * */

	switch(rx_buffer[PKT_TYPE]){
		case PACKET_TYPE_START_HEARTBEAT:
			SF1_SET_BIT(SF1_HB_ENABLE);
			serial_send_reply(PACKET_TYPE_ACK);
			break;
		case PACKET_TYPE_STOP_HEARTBEAT:
			SF1_CLEAR_BIT(SF1_HB_ENABLE);
			serial_send_reply(PACKET_TYPE_ACK);
			break;
		case PACKET_TYPE_PING:
			serial_send_reply(PACKET_TYPE_PONG);
			break;
		case PACKET_TYPE_REQUEST_STATUS:
			serial_send_status();
			break;
		case PACKET_TYPE_MANUAL:
			SF1_CLEAR_BIT(SF1_AUTOMATED);
			serial_send_reply(PACKET_TYPE_ACK);
			break;
		case PACKET_TYPE_AUTOMATIC:
			SF1_SET_BIT(SF1_AUTOMATED);
			serial_send_reply(PACKET_TYPE_ACK);
			break;
		case PACKET_TYPE_SYS_VERSION:
			serial_send_reply(FLOWMASTER_VERSION);
			break;
		case PACKET_TYPE_ROTATE:
			SF1_SET_BIT(SF1_ROTATE);
			serial_send_reply(PACKET_TYPE_ACK);
			break;
		case PACKET_TYPE_NO_ROTATE:
			SF1_CLEAR_BIT(SF1_ROTATE);
			serial_send_reply(PACKET_TYPE_ACK);
			break;
		case PACKET_TYPE_BOOTLOADER:
			serial_call_bootloader();
			break;
		case PACKET_TYPE_SET_FAN:
			speed_set_fan(rx_buffer[2], rx_buffer[3]);
			serial_send_reply(PACKET_TYPE_ACK);
			break;
		case PACKET_TYPE_SET_PUMP:
			speed_set_pump(rx_buffer[2], rx_buffer[3]);
			serial_send_reply(PACKET_TYPE_ACK);
			break;
		case PACKET_TYPE_GET_ADC:
			serial_send_adc_value();
			break;
		default:
			serial_send_reply(PACKET_TYPE_NAK);
			break;
	}

	cleanup:
	SF1_CLEAR_BIT(SF1_RX_WAITING);
	ENABLE_RX_INT();
	RAISE_CTS();
}

static void
serial_add_byte(uint8_t byte)
{
	if(byte == DLE){
		tx_buffer[tx_len++] = DLE;
	}
	tx_buffer[tx_len++] = byte;
}

static void
serial_add_word(uint16_t word)
{
	serial_add_byte((uint8_t)(word >> 8));
	serial_add_byte((uint8_t)(word & 0x00FF));
}

static void
serial_init_frame(uint8_t type, uint8_t length)
{
	tx_len = 2;
	tx_checksum = 0;
	tx_buffer[0] = DLE;
	tx_buffer[1] = STX;
	serial_add_byte(type);
	serial_add_byte(length);
}

static void
serial_end_frame()
{
	/* TODO: This CRC is broken, this will consider DLE bytes used as padding
	 * The mechanism needs to be modified so escapes are ignored
	 * */
	const uint8_t crc = calc_crc8(&tx_buffer[2],tx_buffer[3] + 2);
	serial_add_byte(crc);
	tx_buffer[tx_len++] = DLE;
	tx_buffer[tx_len++] = ETX;
}

void
serial_send_status()
{
	serial_init_frame(PACKET_TYPE_HEARTBEAT,11);

	serial_add_word(FAN_REGISTER);
	serial_add_word(PUMP_REGISTER);
	serial_add_byte(fan_rpm);
	serial_add_byte(pump_rpm);
	serial_add_word(ambient_temp);
	serial_add_word(coolant_temp);
	serial_add_byte(flow_rate);
	serial_end_frame();

	serial_begin_tx();
}

static void
serial_send_adc_value()
{
/*
	serial_init_frame(PACKET_TYPE_GET_ADC,2);
	serial_add_word(adc_store);
	serial_end_frame();

	serial_begin_tx();
*/
}

static void
serial_send_reply(uint8_t type)
{
	serial_init_frame(type,0);
	serial_end_frame();
	serial_begin_tx();
}


/*
 *	Validates the RX buffer, if it is not valid, build and transmit a response
 *	return false.
 *

void
dump_rx_buffer()
{
	uint8_t i;

	cli();

	vfd_write(HD_COMMAND, HD_CMD_RESET);
	vfd_write(HD_COMMAND, HD_CMD_RETURN);

	display_write_hex8(rx_len);
	
	vfd_set_cursor(HD_LINE_1,0);
	
	for(i = 0; i < rx_len; i++)
	{
		display_write_hex8(rx_buffer[i]);
	}
	for(;;);
}

 * */
union temp_u
{
	struct {
		uint8_t high;
		uint8_t low;
	};
	uint16_t word;
};

static uint8_t
validate_rx_buffer()
{
	uint8_t checksum;
	uint16_t temp;

	if(rx_buffer[PKT_LENG] > SERIAL_BUF_PAYLOAD){
		/* Payload length is greater than maximum */
		serial_send_reply(PACKET_TYPE_BAD_LENGTH);
		return 1;
	}

	if((rx_buffer[PKT_LENG] + 3) != rx_len){
		/* Bad payload length */
		serial_send_reply(PACKET_TYPE_BAD_LENGTH);
		return 1;
	}

	checksum = calc_crc8(&rx_buffer[PKT_TYPE],rx_buffer[PKT_LENG] + 2);
	if(checksum != rx_buffer[PKT_CSUM]){
		/* Checksum Error */
		serial_send_reply(PACKET_TYPE_BAD_CSUM);
		return 1;
	}

	/* TODO: Validate command */

	switch(rx_buffer[PKT_TYPE]){
		/* Fan & pump speed is two bytes, a 16 bit integer */
		case PACKET_TYPE_SET_FAN:
		case PACKET_TYPE_SET_PUMP:

		if(rx_buffer[PKT_LENG] != 2){
			return 1;
		}

		/* Perform sanity checking, make sure the values are in range */
		/*

		temp = (rx_buffer[2] << 8) | rx_buffer[3];

		if(temp > TIMER1_TOP){
			serial_send_reply(PACKET_TYPE_NAK);
			return -1;
		}
		else if(temp < (uint16_t)(TIMER1_TOP * 0.3)){
			serial_send_reply(PACKET_TYPE_NAK);
			return -1;
		}
		*/
	}

	/* If we got here, then the packet checks out, lets process it */

	return 0;
}


/* Serial RX interrupt */
ISR(USART_RX_vect)
{
	static uint8_t previous = 0x00;
	uint8_t byte = UDR0;
#if 1
	if(previous == DLE){
		previous = byte;
		switch(byte){
			case STX:
				/* Start of transmission */
				rx_len = 0;
				return;
			case ETX:
				/* End of transmission */
				SF1_SET_BIT(SF1_RX_WAITING);
				LOWER_CTS();
				DISABLE_RX_INT();
				return;
			case DLE:
				goto dle_unstuff;
		}
	}

	previous = byte;

	if(byte != DLE){
		dle_unstuff:

		if(rx_len == SERIAL_BUF_LENGTH){
			/* Buffer Overflow */
			//serial_send_reply(PACKET_TYPE_OVERFLOW);
			return;
		}
		rx_buffer[rx_len++] = byte;

	}
#endif
}

/* Serial transmit interrupt */
ISR(USART_TX_vect)
{
	if(tx_ptr == tx_len){
		tx_len = 0;
		tx_ptr = 0;
		DISABLE_TX_INT();
		return;
	}

	UDR0 = tx_buffer[tx_ptr++];
}

static void
serial_begin_tx()
{
	tx_ptr = 0;
	/* Switch on the TX complete interrupt */
	ENABLE_TX_INT();

	/* Kick off the transmission */
	UDR0 = tx_buffer[tx_ptr++];
}

/*
 *	Dallas CRC-8.
 *	Stolen from here: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=34907
 * */
static uint8_t
calc_crc8(volatile uint8_t* data_pointer, uint8_t number_of_bytes)
{
	uint8_t temp1, bit_counter, feedback_bit, crc8_result = 0;

	while (number_of_bytes--) {
		temp1 = *data_pointer++;

		for (bit_counter = 8; bit_counter; bit_counter--) {
			feedback_bit = (crc8_result & 0x01);
			crc8_result >>= 1;
			if (feedback_bit ^ (temp1 & 0x01)) {
				crc8_result ^= 0x8c;
			}
			temp1 >>= 1;
		}
	}
	return crc8_result;
}

static void
serial_call_bootloader()
{
	/* Disable interrupts and the watchdog */
	cli();
	MCUSR = 0;
	wdt_disable();
	((void(*)(void))(BOOTADDR))();
}
