#ifndef PROTOCOL_H
#define PROTOCOL_H

/*
 * Serial wire protocol for pumpcontrol.
 *
 * Each packet begins with [STX,<packet type>,<data len>,(packet_data,(packet_data)+ ),packet_checksum, ETX]
 *
 * Max payload length is 11 bytes. Any more will result in an overflow condition.
 *
 * When calculating the checksum, all bytes between STX and checksum are considered
 * STX, ETX and cksum are not calculated
 *
 * Packet Type, length and any data are calculated.
 *
 * The length field is mandatory.
 * If the packet contains no data, then the length will be 0.
 *
 * Packet Types
 *
 * If the STX or ETX characters appear in the mormal data transmission, 
 * then they must be stuffed (eg, sent twice)
 *
 * */

#define DLE 0x8F
#define STX 0xAA
#define ETX 0x55


/* Successful - Acknoledge */
#define PACKET_TYPE_ACK 0x01

/* Failure - Negative Ack  */
#define PACKET_TYPE_NAK 0x02

/* Instruct pumpcontrol to send regular hearbeats */
#define PACKET_TYPE_START_HEARTBEAT 0x03

/* Instruct pumpcontrol to shut up  */
#define PACKET_TYPE_STOP_HEARTBEAT 0x04

/* A heartbeat packet containing all system status (explained later) */

/*
 * Contains 7 bytes giving the system status.
 *
 * 0 - Fan duty cycle
 * 1 - Pump duty cycle
 * 2 - Fan RPM
 * 3 - Pump RPM
 * 4 - Ambient temp
 * 5 - Water temp
 * 6 - Flow rate
 * */
#define PACKET_TYPE_HEARTBEAT 0x05

/* Ping - Are you there? */
#define PACKET_TYPE_PING 0x06

/* Pong - Yep */
#define PACKET_TYPE_PONG 0x07

/* Request status (sends one heartbeat packet) */
#define PACKET_TYPE_REQUEST_STATUS 0x08

/* Manual override - disable automated control */
#define PACKET_TYPE_MANUAL 0x09

/* Back to automated control */
#define PACKET_TYPE_AUTOMATIC 0x0A

/* Display a custom message to the LCD */
#define PACKET_TYPE_MESSAGE 0x0B

/* Set the cursor position on the display */
/* Format is a single byte containing the address */
#define PACKET_TYPE_CURSOR 0x0C

/* Update a configuration item */
#define PACKET_TYPE_CONFIG_SET 0x0D

/* Get a configuration item */
#define PACKET_TYPE_CONFIG_GET 0x0E

/* Get the system version */
#define PACKET_TYPE_SYS_VERSION 0x0F

/* Serial buffer overflow */
#define PACKET_TYPE_OVERFLOW 0x10

/* Serial checksum failure */
#define PACKET_TYPE_BAD_CSUM 0x11

/* Stop auto rotating the display */
#define PACKET_TYPE_NO_ROTATE 0x12

/* Start rotating the display as normal */
#define PACKET_TYPE_ROTATE 0x13

/* Length header is bad */
#define PACKET_TYPE_BAD_LENGTH 0x14

/* enter the serial bootloader */
#define PACKET_TYPE_BOOTLOADER 0x15

/* Set the fan duty cycle */
#define PACKET_TYPE_SET_FAN 0x16

/* Set the pump dute cycle */
#define PACKET_TYPE_SET_PUMP 0x17

#define PACKET_TYPE_GET_ADC 0x18

/* Send the TOP value of TIMER1 */
#define PACKET_TYPE_GET_TOP 0x19

/* Update the fan profile */
#define PACKET_TYPE_SET_FAN_PROFILE 0x1A

/* */
#define PACKET_TYPE_GET_FAN_PROFILE 0x1B

#endif
