#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "protocol.h"
#include "serial.h"
#include "globals.h"
#include "speed.h"

static uint8_t calc_crc8(volatile uint8_t* data_pointer, uint8_t number_of_bytes);
static void calc_crc_byte(uint8_t byte);
static uint8_t validate_rx_buffer();
static void serial_begin_tx();
static void serial_send_reply(uint8_t type);
static void serial_send_top();
static void serial_call_bootloader();
static void serial_set_fan_profile();
static void serial_get_fan_profile();

#define RX_BUF_LENGTH 32
#define RX_BUF_PAYLOAD_LEN (RX_BUF_LENGTH - 5)

#define TX_BUF_LENGTH 24
#define TX_BUF_PAYLOAD_LEN (TX_BUF_LENGTH - 2)

volatile static uint8_t tx_buffer[TX_BUF_LENGTH];
/* How many bytes been packed into the buffer */
volatile static uint8_t tx_len = 0;
/* Used to know where we are when transmitting */
volatile static uint8_t tx_ptr = 0;
/* Used as the checksum value for serial transmission */
volatile static uint8_t tx_checksum = 0;

volatile static uint8_t rx_buffer[RX_BUF_LENGTH];
volatile static uint8_t rx_len = 0;

enum serial_tx_state {
	STATE_START_DLE,
	STATE_START_STX,
	STATE_SEND_DATA,
	STATE_SEND_CSUM,
	STATE_END_DLE,
	STATE_END_ETX,
	STATE_DONE
};

/* State machine for the serial transmit interrupt */
volatile static enum serial_tx_state tx_state = STATE_DONE;


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
/*
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
*/

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
		case PACKET_TYPE_GET_TOP:
			serial_send_top();
			break;
		case PACKET_TYPE_SET_FAN_PROFILE:
			serial_set_fan_profile();
			break;
		case PACKET_TYPE_GET_FAN_PROFILE:
			serial_get_fan_profile();
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
	tx_buffer[tx_len++] = byte;
	calc_crc_byte(byte);
}

static void
serial_add_word(uint16_t word)
{
	serial_add_byte((uint8_t)(word >> 8));
	serial_add_byte((uint8_t)(word & 0x00FF));
}

/*
static uint16_t
serial_get_word(uint8_t offset)
{
	return (rx_buffer[offset] << 8) | rx_buffer[offset+1];
}
*/

static void
serial_init_frame(uint8_t type, uint8_t length)
{
	tx_len = 0;
	tx_checksum = 0;

	serial_add_byte(type);
	serial_add_byte(length);
}

static void
serial_end_frame()
{
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
	serial_add_byte(0);
	serial_end_frame();

	serial_begin_tx();
}


/**
 * This will update the EEPROM table where the fan speed table is stored
 * address is the address where the first element will be written.
 * count is the number of data bytes in the packet.
 *
 * every time a byte is written the address will be incremented.
 */

static void
serial_set_fan_profile()
{
	uint8_t i;
	uint8_t count; /* The number of elements in the data packet */
	uint8_t index; /* The address of the first element */
	uint8_t ptr = 4;
	uint16_t data;

	count = rx_buffer[2];
	index = rx_buffer[3];

	if(index > (FAN_TABLE_SIZE - 1)){
		/* index out of range, reject with an error */
		serial_send_reply(PACKET_TYPE_NAK);
		return;
	}

	for(i = 0; i < count; i++) {
		data = rx_buffer[ptr++] << 8;
		data |= rx_buffer[ptr++];

		eeprom_busy_wait();
		eeprom_update_word(&(fan_table[index]), data);
		index++;
	}

	/* send an ack */
	serial_send_reply(PACKET_TYPE_ACK);
}

static void
serial_get_fan_profile()
{
	uint8_t i;
	uint8_t index = rx_buffer[2];
	/* the number of elemenets we can fit in the packet */
	const uint8_t elements = (TX_BUF_PAYLOAD_LEN - 2) / 2;
	uint8_t to_send = elements;

	if(index > (FAN_TABLE_SIZE - 1)){
		/* index out of range, reject with an error */
		serial_send_reply(PACKET_TYPE_NAK);
		return;
	}

	/* if we are close to the limit, clamp it */
	if((index + to_send) > FAN_TABLE_SIZE){
		to_send = FAN_TABLE_SIZE - index;
	}

	/* work out how many sensor values we can send */
	const uint8_t bytes_to_send = (to_send * 2) + 1;

	if(to_send == 0 || to_send > FAN_TABLE_SIZE){
		serial_send_reply(PACKET_TYPE_NAK);
		return;
	}

	serial_init_frame(PACKET_TYPE_GET_FAN_PROFILE, bytes_to_send);
	serial_add_byte(to_send);

	for(i = 0; i < to_send; i++) {
		const uint16_t word = eeprom_read_word(&(fan_table[i+index]));
		serial_add_word(word);
	}

	serial_end_frame();
	serial_begin_tx();
}

static void
serial_send_top()
{
	serial_init_frame(PACKET_TYPE_GET_TOP,2);
	serial_add_word(TIMER1_TOP);
	serial_end_frame();
	serial_begin_tx();
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
*/

static uint8_t
validate_rx_buffer()
{
	uint8_t checksum;
	uint16_t temp;

	if(rx_buffer[PKT_LENG] > RX_BUF_PAYLOAD_LEN){
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
		case PACKET_TYPE_SET_PUMP:

			if(rx_buffer[PKT_LENG] != 2){
				return 1;
			}

			/* Perform sanity checking, make sure the values are in range */

			temp = (rx_buffer[2] << 8) | rx_buffer[3];

			if(temp > TIMER1_TOP){
				serial_send_reply(PACKET_TYPE_NAK);
				return -1;
			}
			else if(temp < (uint16_t)DUTY_CYCLE_MIN){
				/* Don't let the fans or pump go below 20% speed */
				serial_send_reply(PACKET_TYPE_NAK);
				return -1;
			}
			break;
		default:
			break;
	}

	/* If we got here, then the packet checks out, lets process it */

	return 0;
}


/* Serial RX interrupt */
ISR(USART_RX_vect)
{
	static uint8_t rx_previous = 0;
	const uint8_t byte = UDR0;

	if(byte == DLE && rx_previous != DLE) {
		/*
		 * If the current byte is a DLE, and the
		 * previous byte was NOT a DLE, discard it.
		 * */
	}
	else {
		/*
		 * Either we have an escaped byte, or a normal byte.
		 * If it's escaped, check to see what it is and act
		 * accordingly.
		 *
		 * STX indicates the start of a packet. Reset the buffer
		 * and prepare for transmission.
		 *
		 * ETX indicates the end of a transmission. Set the
		 * RX waiting flag so the code knows to check and
		 * process the packet.
		 *
		 * DLE means a DLE character occurred as part of a
		 * normal transmission.  Store it as a valid byte
		 * in the RX buffer.
		 * */
		if(rx_previous == DLE){
			switch(byte){
				case STX:
					rx_len = 0;
					break;
				case ETX:
					SF1_SET_BIT(SF1_RX_WAITING);
					LOWER_CTS();
					DISABLE_RX_INT();
					break;
				case DLE:
					rx_buffer[rx_len++] = byte;
					rx_previous = 0;
					/* return so we don't trip the rx_previous set at the end */
					return;
			}
		}
		else {
			rx_buffer[rx_len++] = byte;
		}
	}
	rx_previous = byte;
}

/* Serial transmit interrupt */

ISR(USART_TX_vect)
{
	static uint8_t previous = 0;

	switch(tx_state) {
		case STATE_START_DLE:
			UDR0 = DLE;
			tx_state = STATE_START_STX;
			break;
		case STATE_START_STX:
			UDR0 = STX;
			previous = 0;
			tx_state = STATE_SEND_DATA;
			break;
		case STATE_SEND_DATA:
			if(previous == DLE){
				UDR0 = DLE;
				previous = 0;
			}
			else {
				UDR0 = previous = tx_buffer[tx_ptr++];
				if(tx_ptr == tx_len){
					tx_state = STATE_SEND_CSUM;
					previous = 0;
				}
			}
			break;
		case STATE_SEND_CSUM:
			/* We may need to pad the checksum if it hits a DLE */
			if(tx_checksum == DLE && previous == 0){
				UDR0 = previous = DLE;
			}
			else {
				tx_state = STATE_END_DLE;
				UDR0 = tx_checksum;
			}
			break;
		case STATE_END_DLE:
			UDR0 = DLE;
			tx_state = STATE_END_ETX;
			break;
		case STATE_END_ETX:
			DISABLE_TX_INT();
			UDR0 = ETX;
			previous = 0;
			tx_state = STATE_DONE;
			break;
		case STATE_DONE:
			break;
	}
}

static void
serial_begin_tx()
{
	tx_ptr = 0;
	/* Switch on the TX complete interrupt */
	ENABLE_TX_INT();

	tx_state = STATE_START_STX;

	/* Kick off the transmission */
	UDR0 = DLE;
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
calc_crc_byte(uint8_t byte)
{
	uint8_t bit_counter;
	uint8_t feedback_bit;
	uint8_t temp1 = byte;
	uint8_t crc8_result = tx_checksum;

	for(bit_counter = 8; bit_counter; bit_counter--){
		feedback_bit = (crc8_result & 0x01);
		crc8_result >>= 1;
		if (feedback_bit ^ (temp1 & 0x01)) {
			crc8_result ^= 0x8c;
		}
		temp1 >>= 1;
	}

	tx_checksum = crc8_result;
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
