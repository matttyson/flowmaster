#ifndef GLOBALS_H
#define GLOBALS_H

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

#define FLOWMASTER_VERSION 1
extern const uint8_t fm_version_str[] PROGMEM;

/* Timer registers for the fan and pumps */
#define FAN_REGISTER OCR1A
#define PUMP_REGISTER OCR1B

#if F_CPU == 14745600UL
	#define TIMER1_TOP ((uint16_t)288)
	#define OFLOW 225
#else
	#error define these values for F_CPU
#endif


/* Fan and pump speeds (<<5 to get RPMs)  */
extern volatile uint8_t fan_rpm;
extern volatile uint8_t pump_rpm;

/* Temperatures (what unit?) */
extern volatile uint16_t ambient_temp;
extern volatile uint16_t coolant_temp;

/* Pump flow rate */
extern volatile uint8_t flow_rate;

/* User configurable maximum */
extern uint8_t eeprom_max_temp EEMEM;

/* power consumption */
extern volatile uint16_t power_usage;

/* Where the result of the current ADC conversion is going */
#define ADC_AMBIENT_TEMP 1
#define ADC_COOLANT_TEMP 2
#define ADC_POWER_USAGE  3

/* Ambient is on PC4 */
#define ADC_AMBIENT_REG (1 << MUX2)
/* Coolant is on PC5 */
#define ADC_COOLANT_REG ((1 << MUX2) | (1 << MUX0))

extern volatile uint8_t adc_current_target;

/* Fan interrupt counter */
extern volatile uint8_t fan_int_ctr;
extern volatile uint8_t fan_int_oflow;
extern volatile uint8_t fan_int_ticks;
extern volatile uint16_t fan_rpm_1;
extern volatile uint16_t fan_rpm_2;

/* Status and config bits */
extern volatile uint8_t status_flag_1;

/* 
 * x - the byte we are testing
 * y - the bit we are interested in
 * */

#define SET_BIT(x,y) (x) = (x) | (y)
#define CLEAR_BIT(x,y) (x) = (x) & ~(y)
#define TEST_BIT(x,y) ((x) & (y))

#define SF1_SET_BIT(z)   SET_BIT(status_flag_1,(z))
#define SF1_CLEAR_BIT(z) CLEAR_BIT(status_flag_1,(z))
#define SF1_TEST_BIT(z)  TEST_BIT(status_flag_1,(z))

/* The serial port has something waiting */
#define SF1_RX_WAITING 0x01
/* The system heartbeat is enabled */
#define SF1_HB_ENABLE  0x02
/* The system is under automated control */
#define SF1_AUTOMATED  0x04
/* Rotate the display */
#define SF1_ROTATE 0x08
/* The result of a fan RPM calculation is waiting to be processed*/
#define SF1_FAN_SPEED 0x10

/* Never go any slower than 30% */
#define DUTY_CYCLE_MIN 77
/* The hardcoded maximum temp we can never go over */
#define GLOBAL_MAX_TEMP ((uint8_t)60)
#define MAX_TEMP() MIN(eeprom_read_byte(&eeprom_max_temp), GLOBAL_MAX_TEMP)

/* Minumum and max
 * */

#endif
