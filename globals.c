
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <inttypes.h>

#include "globals.h"

const uint8_t fm_version_str[] PROGMEM = "0.1";

/* Duty cycle for the fans and pumps */
volatile uint8_t fan_duty_cycle  = 100;
volatile uint8_t pump_duty_cycle = 100;

/* Fan and pump speeds */
volatile uint8_t fan_rpm = 0;
volatile uint8_t pump_rpm = 0;

/* Temperatures - Stored as the raw ADC value */
volatile uint16_t ambient_temp = 0;
volatile uint16_t coolant_temp = 0;


volatile uint16_t power_usage = 0;

/* Coolant flow rate */
volatile uint8_t flow_rate = 0;

/* 
 * The serial port will set this to 1 if
 * there is data waiting to be processed
 * */
volatile uint8_t status_flag_1 = SF1_AUTOMATED | SF1_ROTATE;

/* 
 * Max Temp, at this speed fans and pump will
 * be forced to 100% 
 * */

uint8_t eeprom_max_temp EEMEM;
uint8_t eeprom_temp_bands[4] EEMEM;

volatile uint8_t adc_current_target = ADC_AMBIENT_TEMP;
