#include "input.h"
#include "pid_cont.h"
#include "xil_printf.h"


#define MOTOR_SPEED_SELECT_LSB 0
#define MOTOR_SPEED_SELECT_MSB 1
#define PID_SELECT_LSB  2
#define PID_SELECT_MSB  3
#define INCR_SELECT_LSB 4
#define INCR_SELECT_MSB 5


typedef struct {
    float int_windup;
    float last_windup;
    float kp, ki, kd;
    bool kp_en, ki_en, kd_en;
    float dt;
    float max, min;
} PidState;

static float p_term(float error);
static float i_term(float error);
static float d_term(float error);

static PidState pidstat;

/*
* Function:  p_term 
* --------------------
*	Description:
*       Calculates the contribution of the P term.
*
*	Parameters:
*		error: The difference between the motor speed
*           setpoint and measured motor speed.
*
*	returns: Contribution due to proportional control
*/
float p_term(float error) {
    float term = pidstat.kp * error;
    return term;
}

/*
* Function:  i_term 
* --------------------
*	Description:
*       Calculates the contribution of the I term.  Integrates
*       the error and stores as the integral windup term in
*       the PID state (pidstat).
*
*	Parameters:
*		error: The difference between the motor speed
*           setpoint and measured motor speed.
*
*	returns: Contribution due to integral control
*/
float i_term(float error) {
    pidstat.last_windup = error * pidstat.dt;
    pidstat.int_windup += pidstat.last_windup;
    return pidstat.ki * pidstat.int_windup;
}

/*
* Function:  d_term 
* --------------------
*	Description:
*       Calculates the contribution of the D term.
*
*	Parameters:
*		error: The difference between the motor speed
*           setpoint and measured motor speed.
*
*	returns: Contribution due to derivative control
*/
float d_term(float error) {
    static float error_last = 0;
    float de = error - error_last;
    error_last = error;
    return pidstat.kd * (de / pidstat.dt);
}

/*
* Function:  pid_init 
* --------------------
*	Description:
*       Initializes the internal state of the PID algorithm.
*       Gains are set and the integral windup is initialized.
*       The time step for the control is initialized, and the
*       output saturation limits are specified.
*
*	Parameters:
*		windup_init: The amount of integral windup to begin with.
*           useful for estimated steady state error if available
*           at initialization time.
*		kp_init: Kp gain to use
*		ki_init: Ki gain to use
*		kd_init: Kd gain to use
*		dt: Time difference in seconds between consecutive control steps
*		max: Maximum value that the output should be saturated to
*		min: Minimum value that the output should be saturated to
*
*	returns: None
*/
void pid_init(float windup_init, float kp_init, float ki_init, float kd_init, float dt, float max, float min) {
    pidstat.int_windup = windup_init;

    pid_set_gain(P, kp_init);
    pid_set_gain(I, ki_init);
    pid_set_gain(D, kd_init);

    pidstat.max = max;
    pidstat.min = min;

    pidstat.dt = dt;

    return;
}

/*
* Function:  pid_enable_gains 
* --------------------
*	Description
*       Function to enable or disable the contribution from various terms.
*       Effectively allows for testing P, PI, PID, or any other combination
*       of PID control.
*	Parameters:
*		proportional: Boolean high enables the proportional contribution
*       integral: Boolean high enables the integral contribution
*       derivative: Boolean high enables the derivative contribution
*
*	returns: None
*/
void pid_enable_gains(bool proportional, bool integral, bool derivative) {
    pidstat.kp_en = proportional;
    pidstat.ki_en = integral;
    pidstat.kd_en = derivative;
    return;
}

/*
* Function:  get_enabled_gains_bitfield 
* --------------------
*	Description
*       Gets an integer representation of which gains are currently
*       enabled.  Stored in [D, I, P] where P is the LSB of the bitfield.
*       Useful for displaying information easily on the LEDs.
*	Parameters:
*		None
*
*	returns: Bitfield representation of which gains are enabled
*/
uint8_t get_enabled_gains_bitfield() {
    uint8_t en_gains = 0;
    en_gains |= pidstat.kp_en;
    en_gains |= pidstat.ki_en << 1;
    en_gains |= pidstat.kd_en << 2;
    return en_gains;
}

/*
* Function:  pid_set_gain 
* --------------------
*	Description
*       Change the value of the P, I, or D gain to the
*       value specified by f_gain.
*	Parameters:
*		term: Enumeration for gain to change
*       f_gain: Value to change the gain to
*
*	returns: None
*/
void pid_set_gain(enum _PID_TERMS term, float f_gain) {
    switch (term)
    {
    case P:
        pidstat.kp = f_gain;
        break;
    case I:
        pidstat.ki = f_gain;
        break;
    case D:
        pidstat.kd = f_gain;
        break;
    }
    return;
}

/*
* Function:  pid_perform_step 
* --------------------
*	Description:
*       Calculates the control output for the
*       controller given the enabled gains,
*       current gain values, and stored state.  Output
*       limits are saturated to the min, max values.
*	Parameters:
*		sp: Set Point, or reference term
*       pv: Process variable, or feedback term
*
*	returns: Output of controller for this step.
*/
float pid_perform_step(uint16_t sp, uint16_t pv) {
    float error = ((float) sp) - ((float) pv);
    float output = 0.0;

    output += pidstat.kp_en ? p_term(error) : 0.0;
    output += pidstat.ki_en ? i_term(error) : 0.0;
    output += pidstat.kd_en ? d_term(error) : 0.0;

    if(output > pidstat.max) {
        output = pidstat.max;
        pidstat.int_windup -= pidstat.last_windup;
    } else {
        output = output;
    }
    output = (output < pidstat.min) ? pidstat.min : output;
    if(output < pidstat.min) {
        output = pidstat.min;
        pidstat.int_windup -= pidstat.last_windup;
    } else {
        output = output;
    }

    return output; 

}

/*
* Function:  pid_get_gain_u16fp 
* --------------------
*	Description:
*       Gets the gain as a fixed point representation.  For
*       instance a gain of 0.510 would be represented as a
*       value of 510.  A gain of 0.0001 would be truncated to
*       a value of 0.
*	Parameters:
*		term: Enumeration for PID term to get the gain for
*
*	returns: Fixed point gain
*/
uint16_t pid_get_gain_u16fp(enum _PID_TERMS term) {
    uint16_t gain_fp;
    switch (term)
    {
    case P:
        gain_fp = (uint16_t) (pidstat.kp * 1000);
        break;
    case I:
        gain_fp = (uint16_t) (pidstat.ki * 1000);
        break;
    case D:
        gain_fp = (uint16_t) (pidstat.kd * 1000);
        break;
    }

    return gain_fp;
}

void update_pid_value(enum _PID_TERMS term, float increment_value ) {
    switch (term)
    {
    case P:
        pidstat.kp = pidstat.kp + increment_value;
        break;
    case I:
        pidstat.ki = pidstat.ki + increment_value;
        break;
    case D:
        pidstat.kd = pidstat.kd + increment_value;
        break;    
    }

}

void update_pid_value_sw(SwitchInputs *current_sw_value,   PushButtons *current_pb_value) {
    float increment_value;
    uint8_t gain_step_sel;
    uint8_t gain_sel;
  
    gain_step_sel = encoder_value (current_sw_value->sws[INCR_SELECT_MSB], current_sw_value->sws[INCR_SELECT_LSB]);
    gain_sel = encoder_value (current_sw_value->sws[PID_SELECT_MSB], current_sw_value->sws[PID_SELECT_LSB]);

    switch(gain_step_sel)
    {
    case 0:
        increment_value = 0.001;
        break;
    case 1:
        increment_value = 0.005;
        break;
    case 2:
    case 3:
        increment_value = 0.010;
        break;
    default:
        xil_printf("Invalid increment selection, not changing PID gain\n");
    }

    update_pid_value(gain_sel, increment_value * current_pb_value->ud);

    return;
}

uint16_t update_motor_speed_sw(SwitchInputs *current_sw_value, EncoderState *rotenc){
    uint8_t switch_sel;
    int8_t motor_speed;
    switch_sel = encoder_value (current_sw_value->sws[MOTOR_SPEED_SELECT_MSB], current_sw_value->sws[MOTOR_SPEED_SELECT_LSB]);

    switch (switch_sel)
    {
     case 0:
     motor_speed = rotenc->rot_dir;
        break;
     case 1:
     motor_speed = (rotenc->rot_dir)*(5);
        break;
     case 2:
     case 3:
     motor_speed = (rotenc->rot_dir)*(10);
        break;
    }

    return motor_speed;
}


uint8_t encoder_value (bool msb_bit, bool lsb_bit ) {

    uint8_t ret = 0;
     ret |= (msb_bit << 1);
     ret |= (lsb_bit << 0);

     return ret;
}











