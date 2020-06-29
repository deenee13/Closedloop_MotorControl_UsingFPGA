#ifndef MOTOR_ENCODER_CONTROLLER_H
#define MOTOR_ENCODER_CONTROLLER_H

#include "motor_encoder_controller_l.h"
#include "xil_types.h"
#include "xstatus.h"
#include "stdbool.h"

enum _MOTORDIR {CW = true, CCW = false};

extern XStatus MotorEncCont_initialize(u32 baseaddress);

extern XStatus MotorEncCont_set_direction(bool cw);

extern void MotorEncCont_set_dutycycle(u8 dc);
extern void MotorEncCont_set_dutycycle_pct(u8 dc_pct);

extern u16 MotorEncCont_get_rps_raw();
extern u16 MotorEncCont_get_rpm_raw();

extern u16 MotorEncCont_get_rps();
extern u16 MotorEncCont_get_rpm();

extern void MotorEncCont_set_output_ratio(float ratio);

#endif // MOTOR_ENCODER_CONTROLLER_H