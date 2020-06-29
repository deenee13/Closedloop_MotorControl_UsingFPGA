#ifndef _PIDCONT_H
#define _PIDCONT_H

#include "xil_types.h"
#include "stdbool.h"

enum _PID_TERMS {P = 0, I = 1, D = 2};

extern void pid_init(float windup_init, float kp_init, float ki_init, float kd_init, float dt, float max, float min);
extern void pid_enable_gains(bool proportional, bool integral, bool derivative);
extern void pid_set_gain(enum _PID_TERMS term, float f_gain);
extern uint16_t pid_get_gain_u16fp(enum _PID_TERMS term);
extern float pid_perform_step(uint16_t sp, uint16_t pv);
extern uint8_t encoder_value (bool msb_bit, bool lsb_bit );
extern uint16_t update_motor_speed_sw(SwitchInputs *current_sw_value, EncoderState *rotenc);
extern void update_pid_value_sw( SwitchInputs *current_sw_value, PushButtons *current_pb_value);
extern void update_pid_value(enum _PID_TERMS term, float increment_value);
extern uint8_t get_enabled_gains_bitfield();

#endif