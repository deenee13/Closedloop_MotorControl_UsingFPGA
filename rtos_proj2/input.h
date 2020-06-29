#ifndef _INPUT_H
#define _INPUT_H
#include "xil_types.h"
#include "stdbool.h"

#define N_SWITCHES 16

typedef struct {
    bool switch_up;
    int8_t switch_dir_change;
    bool button_pressed;
    int8_t rot_dir;
    uint32_t raw_state;
} EncoderState;

typedef struct {
    bool up;
    bool down;
    bool left;
    bool right;
    bool center;
    // Will either be -1 to indicate left, 1 for right
    // or 0 for neither (or both)
    int8_t lr;
    // Will either be -1 to indicate down, 1 for up
    // or 0 for neither (or both)
    int8_t ud;
    uint8_t raw_buttons;
} PushButtons;

typedef struct {
    bool sws[N_SWITCHES];
    uint16_t raw_state;
} SwitchInputs;

// TODO Rework Switch Inputs to be what we want, not tied
// to project 1
extern SwitchInputs get_switch_inputs();
extern PushButtons get_push_buttons();
extern EncoderState get_enc_state();

extern int8_t setup_inputs();

#endif // _INPUT_H