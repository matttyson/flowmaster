#ifndef SERIAL_H
#define SERIAL_H

#include <inttypes.h>

void write_byte(uint8_t byte);
void write_string(const char *string);

/* Send the hearbeat packet */
void serial_send_status();
/* Process the data we've been sent */
void serial_process_input();
#endif
