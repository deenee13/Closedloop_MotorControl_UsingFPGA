/*
display.c
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
//Includes shared by display.h
#include "stdbool.h"
#include "xil_types.h"
#include "nexys4IO.h"
#include "xparameters.h"
#include "display.h"
#include "PmodOLEDrgb.h"
#include "microblaze_sleep.h"


// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR
#define LEDMSK_GAINSDISP_MODE 0x00000007

// Internal function prototypes (not exposed via the header)
static void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
static void PMDIO_itoa(int32_t value, char *string, int32_t radix);
static void bin2bcd8(uint8_t bin, uint8_t bcd[]);
static void bin2bcd16(uint16_t bin, uint8_t bcd[]);
static void format_bcd16_fp1000(char* bcd16, char* result7);

static volatile PmodOLEDrgb pmodOLEDrgb_inst;

/*
* Function:  setup_displays 
* --------------------
*	Description
*       Handles initialization of Green LEDs, Seven seg display,
*       OLED display, and Nexys4IO RGBLED duty cycles and enables
*	Parameters:
*		None
*
*	returns: Xilinx Status Code
*/
int8_t setup_displays() {
    // Turn all of the LEDs off
    NX4IO_setLEDs(0);

    // Turn off the high seven seg
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	
    // Set the PWM calc and detect digits to zeros
    NX410_SSEG_setAllDigits(SSEGLO, CC_0, CC_0, CC_0, CC_0, DP_NONE);

    // Initialize OLED and make font white

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);
    usleep(1000000);
    OLEDrgb_SetFontColor(&pmodOLEDrgb_inst, 0xFFFF);
    usleep(500000);

    // Ensure RGBLEDs are off
    NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0);
    NX4IO_RGBLED_setChnlEn(RGB1, false, false, false);
    NX4IO_RGBLED_setDutyCycle(RGB2, 0, 0, 0);
    NX4IO_RGBLED_setChnlEn(RGB2, false, false, false);

    return XST_SUCCESS;
}

/*
* Function:  teardown_displays 
* --------------------
*	Description
*       Sets displays to clear state that the program is ending.  Specifically:
*           - Sets the OLED to state only "BYE BYE"
*           - Sets the SevenSeg display to show "Bye Bye"
*           - Turns off the OLEDrgb
*	Parameters:
*		None
*
*	returns: XST_SUCCESS
*/
int8_t teardown_displays() {

    OLEDrgb_Clear(&pmodOLEDrgb_inst);

    OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 2);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst ,OLEDrgb_BuildRGB(0, 0, 255));  // blue font
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"BYE BYE"); 
	
	usleep(1000 * 1000);
    // clear the displays and power down the pmodOLEDrbg
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);

	return XST_SUCCESS;
}

/*
* Function:  format_bcd16_fp1000 
* --------------------
*	Description:
*       Formats a fixed point number stored as BCD into
*       a character array representing the number with a
*       decimal point.  The number in bcd16 is treated as
*       a value multiplied by 1000.
*
*	Parameters:
*		bcd16: The fixed point number stored as BCD. Numbers
*           up to "99.999" can be represented.
*       result7: Output character string with the most
*           significant digits in the lower indices. The
*           number is formatted to 3 digits after the
*           decimal point.
*
*	returns: None
*/
void format_bcd16_fp1000(char* bcd16, char* result7) {
    // This whole function is kinda hacky but it works
    result7[0] = 48 + bcd16[0];
    result7[1] = 48 + bcd16[1];
    result7[2] = '.';
    result7[3] = 48 + bcd16[2];
    result7[4] = 48 + bcd16[3];
    result7[5] = 48 + bcd16[4];
    result7[6] = '\0';
    return;
}

/*
* Function:  set_oled_disp_pidgains_numeric 
* --------------------
*	Description
*       Sets the pmodOLEDrgb to display the floating point gains
*       on the display.  Accepts fixed point inputs for gains.
*	Parameters:
*		kp_fp1000: Kp gain multiplied by 1000 and integer truncated
*       ki_fp1000: Ki gain multiplied by 1000 and integer truncated
*       kd_fp1000: Kd gain multiplied by 1000 and integer truncated
*
*	returns: XST_SUCCESS
*/
XStatus set_oled_disp_pidgains_numeric(uint16_t kp_fp1000, uint16_t ki_fp1000, uint16_t kd_fp1000) {
    static uint16_t kp_l = 0xFFFF;
    static uint16_t ki_l = 0xFFFF;
    static uint16_t kd_l = 0xFFFF;
    
    char kpbcd[5];
    char kibcd[5];
    char kdbcd[5];

    char temp_finalfmt_gain[] = "00.000";

    if(kp_l != kp_fp1000) {
        // Write Kp
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 1);
        OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kp:");
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
        bin2bcd16(kp_fp1000, kpbcd);
        format_bcd16_fp1000(kpbcd, temp_finalfmt_gain);
        OLEDrgb_PutString(&pmodOLEDrgb_inst, temp_finalfmt_gain);
    }
    if(ki_l != ki_fp1000) {
        // Write Ki
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 2);
        OLEDrgb_PutString(&pmodOLEDrgb_inst,"Ki:");
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 2);
        bin2bcd16(ki_fp1000, kibcd);
        format_bcd16_fp1000(kibcd, temp_finalfmt_gain);
        OLEDrgb_PutString(&pmodOLEDrgb_inst, temp_finalfmt_gain);
    }
    if(kd_l != kd_fp1000) {
        // Write Kd
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 3);
        OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kd:");
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
        bin2bcd16(kd_fp1000, kdbcd);
        format_bcd16_fp1000(kdbcd, temp_finalfmt_gain);
        OLEDrgb_PutString(&pmodOLEDrgb_inst, temp_finalfmt_gain);
    }

    kp_l = kp_fp1000;
    ki_l = ki_fp1000;
    kd_l = kd_fp1000;

	return XST_SUCCESS;
}

/*
* Function:  set_oled_disp_meas_mot_speed_numeric 
* --------------------
*	Description
*       Displays an integer motor speed on the display in the fourth
*       row.  Motor speeds up to 65535 RPM can be displayed.
*
*	Parameters:
*		meas_speed: The speed of the motor to be displayed (in RPM)
*
*	returns: XST_SUCCESS
*/
XStatus set_oled_disp_meas_mot_speed_numeric(uint16_t meas_speed) {
    static uint16_t meas_speed_l;
    if(meas_speed_l != meas_speed) {
        // Write Measured speed
    	//xil_printf("writing motor speed: %d\n", meas_speed);
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 4);
        OLEDrgb_PutString(&pmodOLEDrgb_inst,"M_RPM:         ");
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 7, 4);
        PMDIO_putnum(&pmodOLEDrgb_inst, (int32_t) meas_speed, 10);

    }

    meas_speed_l = meas_speed;

	return XST_SUCCESS;
}

/*
* Function:  disp_meas_speed 
* --------------------
*	Description
*       Displays the motor speed (up to 9999 RPM) on the high
*       seven segments.
*	Parameters:
*       rpm: The motor speed to display, in rotations per minute.
*
*	returns: XST_SUCCESS
*/
XStatus disp_meas_speed(uint16_t rpm) {
    uint8_t bcd[5];
    uint16_t clamped_rpm;
    clamped_rpm = (rpm > 9999) ? 9999 : rpm;
    bin2bcd16(clamped_rpm, bcd);

    // Note: BCD is coded such that most significant digit is in lower index.
    // Therefore least significant digit is in fifth byte (index 4) since a 16bit
    // value could be up to 65535.  Our display only shows 4 digits of speed, so
    // we always throw away bcd[0].

    NX410_SSEG_setAllDigits(SSEGHI, bcd[1], bcd[2], bcd[3], bcd[4], DP_NONE);

	return XST_SUCCESS;
}

/*
* Function:  disp_set_speed 
* --------------------
*	Description
*       Displays the motor set speed (up to 9999 RPM) on the low
*       seven segments.
*	Parameters:
*       rpm: The motor speed to display, in rotations per minute.
*
*	returns: XST_SUCCESS
*/
XStatus disp_set_speed(uint16_t rpm) {
    uint8_t bcd[5];
    uint16_t clamped_rpm;
    clamped_rpm = (rpm > 9999) ? 9999 : rpm;
    bin2bcd16(clamped_rpm, bcd);

    // Note: BCD is coded such that most significant digit is in lower index.
    // Therefore least significant digit is in fifth byte (index 4) since a 16bit
    // value could be up to 65535.  Our display only shows 4 digits of speed, so
    // we always throw away bcd[0].

    NX410_SSEG_setAllDigits(SSEGLO, bcd[1], bcd[2], bcd[3], bcd[4], DP_NONE);

	return XST_SUCCESS;
}

/*
* Function:  set_disp_gains_applied_fb_leds 
* --------------------
*	Description
*       Sets the LEDs values which indicate which gains are
*       being applied in the PID controller
*	Parameters:
*		en_gains: Bitfield indicating which gains are enabled
*           [D, I, P] where P is LSB and D is MSB.
*
*	returns: Void
*/

void set_disp_gains_applied_fb_leds(uint8_t en_gains) {
    u32 leds = NX4IO_getLEDS_DATA();
    leds &= ~LEDMSK_GAINSDISP_MODE;
    leds |= en_gains;
    NX4IO_setLEDs(leds);
    return;
}

// Helper functions

/**
* Converts an unsigned char to BCD so that it can be displayed
* on the Seven Segment display
*
* @param	bin is the unsigned char to convert.
*
* @param	*bcd is a pointer to the buffer holding the result. The buffer
*			should be at least 3 bytes long. the BCD digits are returned
*			with the largest digit being in bcd[0] and the least signifcant
*			digit being in bcd[2]
*
* @return	NONE
*
* @note
*	Source:  http://www.keil.com/forum/14621/
*/

void bin2bcd16(uint16_t bin, uint8_t bcd[])
{
	#define DIM(a)  (sizeof(a) / sizeof(a[0]))

    static const uint16_t pow_ten_tbl[] = {
        10000,
        1000,
        100,
        10,
        1
    };
    uint16_t pow_ten;
    uint8_t digit;
    uint8_t i;

    for (i = 0; i != DIM(pow_ten_tbl); i++) {
        digit = 0;
        pow_ten = pow_ten_tbl[i];

        while (bin >= pow_ten) {
            bin -= pow_ten;
            digit++;
        }
        *bcd++ = digit;
    }
}

/**
* Converts an unsigned char to BCD so that it can be displayed
* on the Seven Segment display
*
* @param	bin is the unsigned char to convert.
*
* @param	*bcd is a pointer to the buffer holding the result. The buffer
*			should be at least 3 bytes long. the BCD digits are returned
*			with the largest digit being in bcd[0] and the least signifcant
*			digit being in bcd[2]
*
* @return	NONE
*
* @note
*	Source:  http://www.keil.com/forum/14621/
*/

void bin2bcd8(uint8_t bin, uint8_t bcd[])
{
	#define DIM(a)  (sizeof(a) / sizeof(a[0]))

    static const uint8_t pow_ten_tbl[] = {
        100,
        10,
        1
    };
    uint8_t pow_ten;
    uint8_t digit;
    uint8_t i;

    for (i = 0; i != DIM(pow_ten_tbl); i++) {
        digit = 0;
        pow_ten = pow_ten_tbl[i];

        while (bin >= pow_ten) {
            bin -= pow_ten;
            digit++;
        }
        *bcd++ = digit;
    }
}

/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[4];
  
  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion, 
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}
	
  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;
	
	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;
	
  	return;
}
