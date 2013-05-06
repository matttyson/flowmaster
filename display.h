#ifndef DISPLAY_H
#define DISPLAY_H


/*
 * This is the high level display driver.  It sits on top of the basic IO library
 *
 * It is assumed that the display is a 20 x 2 display.
 *
 *
 * */

#define DISP_MENU_RPM 1
/*
 Displays:
*--------------------*
|Fans: 100% 1234 rpm |
|Pump: 100% 1234 rpm |
*--------------------*
*/

#define DISP_MENU_FLOW 2
/*
*--------------------*
|Flow: 99 lph        |
|12v * 9.99a = 99w   |
*--------------------*
*/

#define DISP_MENU_TEMP 3
/*
Temp
*--------------------*
|Coolant: 99c  Delta |
|Ambient: 99c   99c  |
*--------------------*
*/

#define DISP_MENU_MAX 4

/* Update the current display with new values */
void display_update();

/* Switch to a new display */
void display_switch(uint8_t display);

#endif
