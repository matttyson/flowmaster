#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#ifdef FM_DISPLAY
	#include "vfd.h"
	#include "display.h"
#endif

#include "init.h"
#include "globals.h"
#include "serial.h"

volatile uint8_t update = 0;

int
main(void)
{
#ifdef FM_DISPLAY
	uint8_t current_display = DISP_MENU_TEMP;
	uint8_t rotation_counter = 0;
#endif
	MCUSR = 0;
	wdt_disable();
	cli();

	init_micro();

#ifdef FM_DISPLAY
	vfd_init();
	display_switch(current_display);
#endif

	/* Set a watchdog time of 500ms. WDTO_500MS */
	wdt_enable(WDTO_500MS);

	/* Enable interrupts */
	sei();

	while(1){
		wdt_reset();

		if(update){
			if(SF1_TEST_BIT(SF1_HB_ENABLE)){
				serial_send_status();
			}
			update = 0;

#ifdef FM_DISPLAY
			display_update();
			//rotation_counter++;
			if(SF1_TEST_BIT(SF1_ROTATE) && (rotation_counter == 5)){
				rotation_counter = 0;
				if(current_display == DISP_MENU_MAX){
					current_display = 1;
				}
				display_switch(current_display++);
			}
#endif
		}

		if(SF1_TEST_BIT(SF1_RX_WAITING)){
			/* Serial data has been received.  process it.*/
			serial_process_input();
		}

	}
}
