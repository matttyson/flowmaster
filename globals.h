#ifndef GLOBALS_H
#define GLOBALS_H

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

#define FLOWMASTER_VERSION 2

#define FAN_TABLE_SIZE 65
extern uint16_t fan_table[FAN_TABLE_SIZE] EEMEM;

/* Timer registers for the fan and pumps */
#define FAN_REGISTER OCR1A
#define PUMP_REGISTER OCR1B

#define PUMP_SPEED ((uint16_t)(TIMER1_TOP * 0.215))
#define FAN_SPEED 0

#if F_CPU == 14745600UL
/*
 * See the AVR datasheet about Phase Correct PWM
 * No prescaler (ie, 1)
 * (14745600)/(2*1*288) = 25600khz
 * */
	#define TIMER1_TOP ((uint16_t)288)
	#define OFLOW (225)
#elif F_CPU == 18432000UL
	#define TIMER1_TOP ((uint16_t)360)
	#define OFLOW (300)
#else
	#error define these values for F_CPU
#endif

/* Fan and pump speeds (<<5 to get RPMs) */
extern volatile uint8_t fan_rpm;
extern volatile uint8_t pump_rpm;

/* Temperatures (what unit?) */
extern volatile uint16_t ambient_temp;
extern volatile uint16_t coolant_temp;

/* Where the result of the current ADC conversion is going */
#define ADC_AMBIENT_TEMP 1
#define ADC_COOLANT_TEMP 2
#define ADC_POWER_USAGE  3
#define ADC_DONE         4


/* update init_thermistors() in init.c if you change this! */

/* The various ADC Channels */
#define ADC5 ((1 << MUX2) | (1 << MUX0))
#define ADC4 (1 << MUX2)
#define ADC3 ((1 << MUX1) | (1 << MUX0))
#define ADC2 (1 << MUX1)
#define ADC1 (1 << MUX0)
#define ADC0 (0)

/* The pins that we are using fo the thermistors */
#define ADC_AMBIENT_REG ADC3
#define ADC_COOLANT_REG ADC4

/* Disable digital on these pins */
#define INIT_AMBIENT_ADC ADC3D
#define INIT_COOLANT_ADC ADC4D

extern volatile uint8_t adc_current_target;

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
/* update the displays */
#define SF1_UPDATE_READY 0x20
/* did the system restart under a watchdog condition */
#define SF1_WDT_RESET 0x40

/* Never go any slower than 20% */
#define DUTY_CYCLE_MIN (TIMER1_TOP * 0.20)

#endif
