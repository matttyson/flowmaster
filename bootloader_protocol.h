#ifndef BOOTLOADER_PROTOCOL_H
#define BOOTLOADER_PROTOCOL_H

#define BL_BAUDRATE 19200

#define BL_ACK 0x06
#define BL_NAK 0x15

#define BL_PING 'p'
#define BL_SET_ADDR 'A'
#define BL_ERASE 'e'
#define BL_ERASE_EEPROM 'E'
#define BL_PROGRAM 'C'
#define BL_RESET 'R'

/* Show Programming message */
#define BL_PROGRAM_MESSAGE 'm'

/* Show a percentage point */
#define BL_PERCENT '%'


#endif
