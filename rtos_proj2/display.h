#ifndef _DISPLAY_H
#define _DISPLAY_H

/*
display.h
ECE544 Project 1 - Robert Holt
April 10, 2020
Created for NEXYSA7 
Encompases header definitions for all things related to the display.
This includes:
- OLED Color Box Drawing
- OLED H, S, V Numeric Display
- Seven Segment display
    - Digits[3:2] are calculated duty cycle rounded to
        integer percentages (i.e. 0d00 - 0d99)
    - Digits[1:0] are detected duty cycle rounded to
        integer percentages (i.e. 0d00 - 0d99)
    - DP[1] is periodically blinking with the FIT interrupt handler
- Green LEDS
*/
#include "xil_types.h"
#include "stdbool.h"
#include "nexys4IO.h"
//#include "pid_cont.h"

// General Setup
extern int8_t setup_displays();
extern int8_t teardown_displays();

extern XStatus set_oled_disp_pidgains_numeric(uint16_t kp_fp1000, uint16_t ki_fp1000, uint16_t kd_fp1000);
extern XStatus set_oled_disp_meas_mot_speed_numeric(uint16_t meas_speed);

extern XStatus disp_meas_speed(uint16_t rpm);
extern XStatus disp_set_speed(uint16_t rpm);

extern void set_disp_gains_applied_fb_leds(uint8_t en_gains);

#endif //_DISPLAY_H
