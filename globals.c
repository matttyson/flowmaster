
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <inttypes.h>

#include "globals.h"

/* Duty cycle for the fans and pumps */
volatile uint8_t fan_duty_cycle  = 0;
volatile uint8_t pump_duty_cycle = 0;

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
 * The status flag, contains info about the state of
 * the system
 * */
volatile uint8_t status_flag_1 = SF1_ROTATE;

/* 
 * Max Temp, at this speed fans and pump will
 * be forced to 100% 
 * */

volatile uint8_t adc_current_target = ADC_AMBIENT_TEMP;

