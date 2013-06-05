#include "init.h"
#include "vfd.h" /* For the chip select pin */
#include "globals.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

static void
init_serial()
{
#define BAUD_PRESCALE(x) (F_CPU / ((x) * 16UL) - 1UL)
	UBRR0H = (uint8_t) (BAUD_PRESCALE(USART_BAUD) >> 8);
	UBRR0L = (uint8_t) BAUD_PRESCALE(USART_BAUD);
#undef BAUD_PRESCALE

	// Enable TX and RX
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << TXCIE0);

	/* TODO: Pick two wires to use as flow control? */
}


static void
init_tach_int()
{
	/* Configure interrupt pins as inputs */
	/* PD4 PD3 - set to 0*/
	DDRD = DDRD & ~((1 << PD2) | (1 << PD3));
	/* PORTD to logic 1 (pull up enabled) - (fan tach is open collector) */
	PORTD = PORTD | ((1 << PD2) | (1 << PD3));
	
	/* Pull up disabled, use external pullup resistor */
	//PORTD = PORTD & ( ~ ((1 << PD2) | (1 << PD3)));

	EICRA = (1 << ISC11) | (0 << ISC10) | (1 << ISC01) | (0 << ISC00);

	/* Activate interrupt handler. */
	EIMSK = (1 << INT1) | (1 << INT0);
}

/*
 *	This timer controls the PWM frequncy for the pump and fans.
 *	We need to generate a PWM pulse at 25khz.
 *
 *	To do this we use mode 10 of the 16 bit timer 1.
 *	
 *	TOP is stored in the ICR1 register.
 *
 *	ATMEGA88
 *	
 * */
static void
init_pwm_timer()
{

	/* Enable PWM pins as output */
	DDRB = DDRB | (1 << PB2) | (1 << PB1);

	/* Set waveform generation, mode 10 */
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << CS10) | (1 << WGM13);

	/* Set top value of 25600hz at 14.7mhz */
	ICR1 = TIMER1_TOP;

	PUMP_REGISTER = PUMP_SPEED;
	FAN_REGISTER = FAN_SPEED;

	/* Enable the output pins, non inverting mode */
	TCCR1A |= (1 << COM1A1)| (0 << COM1A0) | (1 << COM1B1) | (0 << COM1B0);
}

static void
init_misc()
{
	/* Disable analog comparitor */
	ACSR = (1 << ACD);

	/* Shut down timer/counter 0 */
	/* Shut down two wire interface */
	PRR = (1 << PRTWI) | (1 << PRTIM0);

}

/*
 * 	This timer is used to generate a second delay counter for updating
 * 	the screen.
 *
 * 	The 16 bit timer is used for PWM generation and even with a 1024
 * 	prescaler that's not enough for a one second delay.
 *
 * 	We need to do a bit of a dirty hack and use a variable to hold another
 * 	prescaler.
 *
 * 	We'll use Timer/Counter 2, as it's output pins can't be used due to
 * 	a conflict with the SPI port.
 *
 * 	CTC mode.
 * */
static void
init_timer()
{
	OCR2A = 0xFF;

	TCCR2A = (1 << WGM21);
	TCCR2B = (1 << CS22) | (1 << CS21);
	
	TIMSK2 = (1 << OCIE2A);
}

#ifdef FM_DISPLAY
static void
init_spi()
{
	/* SS, MISO, MOSI, SCK - set as outputs  */
	/* SS must be an output or SPI master mode will break */
	DDRB = DDRB | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5);
	/* Configure user defined chip select pin */
	VFD_CS_DDR = VFD_CS_DDR | (1 << VFD_CS_PIN);

	/* Enable SPI, Master, Mode 3 */
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA) | (0 << SPR1) | (0 << SPR0);
	/* Double speed SPI */
	SPSR = SPSR | (1 << SPI2X);
}
#endif

/*
 * Thermistors are on ADC5 and ADC4 (port C, pins 5,4)
 *
 * Currently using pin4
 * */

static void
init_thermistors()
{
	/* DDRC pins 4 and 5 as inputs */
	DDRC = DDRC & ~((1 << PC5) | (1 << PC4));
	/* Pull ups off */
	PORTC = PORTC & ( ~ ((1 << PC5) | (1 << PC4)));

	/* Disable digital inputs */
	DIDR0 = (1 << ADC5D) | (1 << ADC4D);

	/* AVcc with capacitor on AREF */
	ADMUX = (0 << REFS1) | (1 << REFS0);

	/* Pin 4 */
	ADMUX |= (1 << MUX2);

	/* Enable ADC, Enable interrupts */
	/* ADC divisor of 128 */
	ADCSRA = (0 << ADATE) | (1 << ADEN) | ( 1 << ADIE) | (1 << ADSC) |
			(1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	/* 
	 * 23.3
	 * Change ADC to individual conversions.
	 * constantly switch between the two sensors.
	 * */
}

void
init_micro()
{
	init_pwm_timer();
	init_misc();
	init_serial();
	init_tach_int();
#ifdef FM_DISPLAY
	init_spi();
#endif
	init_timer();
	init_thermistors();

	SF1_CLEAR_BIT(SF1_WDT_RESET);
}
