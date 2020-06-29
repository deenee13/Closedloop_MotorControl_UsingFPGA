/*
input.c
ECE544 Project 1 - Robert Holt
April 10, 2020
Created for NEXYSA7 
Encompases header definitions for all things related to abstracting inputs.
This includes:
    - Slide switches
    - Push buttons
    - Encoder state
*/

#include "input.h"
#include "xstatus.h"
#include "PmodENC.h"
#include "nexys4IO.h"
#include "stdbool.h"
#include "xil_types.h"
#include "xparameters.h"
#include "microblaze_sleep.h"

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

static EncoderState _lastencstate;
static PmodENC pmodENC_inst;

/*
* Function:  setup_inputs 
* --------------------
*	Description
*       Sets up the encoder and initializes it's internal state.
*	Parameters:
*		None
*
*	returns: XST_SUCCESS if successful.
*/
int8_t setup_inputs() {
    // initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);
    //usleep(100);

    // Initialize the last encoderstate
    uint32_t setup_enc_state = ENC_getState(&pmodENC_inst);

    _lastencstate.button_pressed = (bool) ENC_buttonPressed(setup_enc_state);
    _lastencstate.switch_up = (bool) ENC_switchOn(setup_enc_state);
    // Cant know direction because it hasnt been established yet
    // This will be defined in future calls to get_enc_state
    _lastencstate.rot_dir = 0;
    _lastencstate.raw_state = setup_enc_state;

    return XST_SUCCESS;
}

/*
* Function:  get_switch_inputs 
* --------------------
*	Description
*       Reads the slide switches on the NexyA7 board in order to
*       determine the semantic meanings of the following user requests:
*           - Hardware PWD requested mode
*           - PWM Channel display request (Red, Green, Blue)
*           - SW PWD interrupt rate request
*	Parameters:
*		None
*
*	returns: SwitchInputs struct with the requested modes.
*/
SwitchInputs get_switch_inputs() {
    SwitchInputs _sw_ins;
    uint16_t raw_switches = NX4IO_getSwitches();
    for(int i = 0; i < N_SWITCHES; i++) {
        _sw_ins.sws[i] = (raw_switches >> i) & 0x1;
    }
    _sw_ins.raw_state = raw_switches;
    return _sw_ins;
}

/*
* Function:  get_enc_state 
* --------------------
*	Description
*       Uses the encoder API to query and interpret the state of the
*       encoder, thus abstracting several calls and only providing a
*       struct response.  This assumes there is only a single instance
*       of the encoder connected.  The fields in the struct correspond to:
*           - Button Pressed - whether the encoder shaft is depressed in
*           - Switch up - whether the slide switch on the encoder is "up"
*           - Rotation direction - Uses -1 for CCW rotation and 1 for CW rotation
*           - Raw State - The actual raw state from the ENC_getState.  This
*               is redundant information, but is used internally to the input.c
*               file and gives the user the option to use it.
*	Parameters:
*		None
*
*	returns: EncoderState struct with the state of the encoder
*/
EncoderState get_enc_state() {
    EncoderState _encstate;
    uint32_t raw_state = ENC_getState(&pmodENC_inst);

    _encstate.button_pressed = (bool) ENC_buttonPressed(raw_state);
    _encstate.switch_up = (ENC_switchOn(raw_state) == 0);
    //xil_printf("%d, switchup: %d\n", ENC_switchOn(raw_state), _encstate.switch_up);
    _encstate.switch_dir_change = (int8_t) _encstate.switch_up - (int8_t) _lastencstate.switch_up;
    _encstate.rot_dir = (int8_t) ENC_getRotation(raw_state, _lastencstate.raw_state);
    _encstate.raw_state = raw_state;

    _lastencstate = _encstate;
    return _encstate;
}

/*
* Function:  get_push_buttons 
* --------------------
*	Description
*       Determines the status of the pushbuttons on the NexysA7 board.  This
*       does not read the status of the reset of other buttons outside of the:
*           - Left
*           - Right
*           - Center
*           - Up
*           - Down
*       Additionally determines a "increment" or "decrement" direction for
*       both the Up/Down and Left/Right button presses.  When a user presses
*       up or right, the corresponding increment/decrement signal reflects this
*       by setting the value to +1.  When a user  pressses left or down, the corresponding
*       increment/decrement signal reflects this by setting the value to -1.  If
*       neither or both are pressed, the individual button statuses reflect the
*       actual states, but the increment/decrement signals will report 0.
*       This is used primarily in the colorwheel application for changing the
*       saturation and value.
*	Parameters:
*		None
*
*	returns: PushButtons struct which indicates the state of the push buttons
*/
PushButtons get_push_buttons() {
    PushButtons buttons;
    uint8_t raw_push_buttons = NX4IO_getBtns();
    buttons.raw_buttons = raw_push_buttons;
    buttons.center = (bool)((NEXYS4IO_BTNC_MASK >> 16) & raw_push_buttons);
    buttons.up = (bool)((NEXYS4IO_BTNU_MASK >> 16) & raw_push_buttons);
    buttons.left = (bool)((NEXYS4IO_BTNL_MASK >> 16) & raw_push_buttons);
    buttons.right = (bool)((NEXYS4IO_BTNR_MASK >> 16) & raw_push_buttons);
    buttons.down = (bool)((NEXYS4IO_BTND_MASK >> 16) & raw_push_buttons);

    if(buttons.up != buttons.down) {
        buttons.ud = buttons.up ? 1 : -1;
    }
    else {
        buttons.ud = 0;
    }

    if(buttons.left != buttons.right) {
        buttons.lr = buttons.right ? 1 : -1;
    }
    else {
        buttons.lr = 0;
    }

    return buttons;
}



