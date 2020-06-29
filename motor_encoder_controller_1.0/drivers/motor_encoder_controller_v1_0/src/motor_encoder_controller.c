

/***************************** Include Files *******************************/
#include "motor_encoder_controller.h"
#include "xil_io.h"

/************************** Constant Definitions ***************************/
#define MOTORENCCONT_DIR_MASK       0x00000100
#define MOTORENCCONT_DC_MASK        0x000000FF
#define MOTORENCCONT_RPS_MASK       0x0000FFFF
#define MOTOR_STOPPED_THRESH_RPS    3
/**************************** File Static Vars *****************************/
static u32 MotorEncContBaseAddr;
u16 c_fb_rps = 0; // Current motor feedback speed in rotations per second
float inv_output_ratio = 1.0;
/************************** Function Definitions ***************************/

XStatus MotorEncCont_initialize(u32 BaseAddr)
{
	MotorEncContBaseAddr = BaseAddr;
    return MOTOR_ENCODER_CONTROLLER_Reg_SelfTest(MotorEncContBaseAddr);
}

// TODO: Include code in here to enforce that the direction cannot be changed when moving
XStatus MotorEncCont_set_direction(bool cw) {
    //RMW sequence
    u32 dirmask = (cw << 8) & MOTORENCCONT_DIR_MASK;
    u32 reg;
    
    // Update the internal state for motor speed
    MotorEncCont_get_rps_raw();

    if(c_fb_rps > MOTOR_STOPPED_THRESH_RPS) {
        xil_printf("Cannot change direction, motor speed %d > %d\n", c_fb_rps, MOTOR_STOPPED_THRESH_RPS);
        return XST_FAILURE;
    }

    reg = MOTOR_ENCODER_CONTROLLER_mReadReg(MotorEncContBaseAddr,
                                            MOTOR_ENCODER_CONTROLLER_MOVEMENTCTRL_REG_OFFSET);
    reg &= ~MOTORENCCONT_DIR_MASK;
    reg |= dirmask;
    MOTOR_ENCODER_CONTROLLER_mWriteReg(MotorEncContBaseAddr,
                                        MOTOR_ENCODER_CONTROLLER_MOVEMENTCTRL_REG_OFFSET,
                                        reg);
    return XST_SUCCESS;
}

void MotorEncCont_set_dutycycle(u8 dc) {
    u32 dcmask = ((u32) dc) & MOTORENCCONT_DC_MASK;
    u32 reg = MOTOR_ENCODER_CONTROLLER_mReadReg(MotorEncContBaseAddr,
                                            MOTOR_ENCODER_CONTROLLER_MOVEMENTCTRL_REG_OFFSET);
    reg &= ~MOTORENCCONT_DC_MASK;
    reg |= dcmask;
    MOTOR_ENCODER_CONTROLLER_mWriteReg(MotorEncContBaseAddr,
                                        MOTOR_ENCODER_CONTROLLER_MOVEMENTCTRL_REG_OFFSET,
                                        reg);

    return;
}

void MotorEncCont_set_dutycycle_pct(u8 dc_pct) {
    u8 dc = (dc_pct * 255) / 100;
    MotorEncCont_set_dutycycle(dc);
    return;
}

u16 MotorEncCont_get_rpm() {
    return (u16) (inv_output_ratio * (float) MotorEncCont_get_rpm_raw());
}

u16 MotorEncCont_get_rps() {
    return (u16) (inv_output_ratio * (float) MotorEncCont_get_rps_raw());
}

void MotorEncCont_set_output_ratio(float ratio) {
    inv_output_ratio = 1.0 / ratio;
    return;
}

u16 MotorEncCont_get_rps_raw() {
    u32 reg = MOTOR_ENCODER_CONTROLLER_mReadReg(MotorEncContBaseAddr,
                                            MOTOR_ENCODER_CONTROLLER_MOVEMENTFB_REG_OFFSET);
    c_fb_rps = (u16) (reg & MOTORENCCONT_RPS_MASK);
    return c_fb_rps;
}

// Convenience function to get speed as rotations per minute instead of
// rotations per second
u16 MotorEncCont_get_rpm_raw() {
    u16 rps = MotorEncCont_get_rps_raw();
    return rps * 60;
}