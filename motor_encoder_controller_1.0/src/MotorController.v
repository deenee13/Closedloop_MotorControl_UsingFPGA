`timescale 1ns / 1ps

module MotorController
#(
	//parameter declarations
	parameter 			RESET_POLARITY_LOW = 1,				// Reset is active-low?  (default is yes)
	parameter 			PWM_DC_WIDTH = 8					// Number of duty cycle bits for each channel (Default = 9-bit or 1/512 resolution)
)
(
	// port declarations
	input 						CLK, 						// 100MHz clock
	input						RESET,						// system reset
    // Application Inputs
    input                       DIR_SEL_RQ,                    // Rq from Software 1 is CW, 0 is CCW
	input   [7:0]               MOTOR_DC,                   // Duty cycle for motor output [0 - 255]
	// Application Outputs
    output	    [15:0]           MOTOR_FB_RPS,                // Motor Feedback rotations per second
    // Motor Inputs
    input                       MOTOR_ENC,                  // Raw encoder input for calculating motor speed
    // Motor Outputs
    output                      DIR_SEL_OUT,
    output                      MOTOR_EN
);

localparam LOG2_N_ENC_SAMPLING_SUBSEGS = 4; // log2(16) = 4 sampling sub-intervals
localparam CLOCK_HZ = 100000000; // 100MHz clock
localparam MAX_COUNT_NEXT_SUBSEG = CLOCK_HZ / (2**LOG2_N_ENC_SAMPLING_SUBSEGS);

// use the RESET_POLARITY_LOW parameter to set the RESET_Int slevel
wire RESET_Int = RESET_POLARITY_LOW ? ~RESET : RESET;

wire [15:0] total_enc_ticks_s;

// (ticks / s) * (1 rot / 12 ticks) = (rot / s)
// TODO: fix this.  Is division by 12 causing a problem?  UG901 would indicate so
//assign MOTOR_FB_RPS = total_enc_ticks_s / 12;
//Instead of dividing by 12, try multiplying ...
// total_enc_ticks_s * (2^8 / 12) >> 8
wire [31:0] fb_rps_intermediate = (total_enc_ticks_s * ((2**8) / 12)) >> 8; 
assign MOTOR_FB_RPS = fb_rps_intermediate[15:0];
//assign MOTOR_FB_RPS = 16'hF0F0;


// Synchronizing Reg's for incoming Motor Encoder signal
reg motor_enc_s1 = 0;
reg motor_enc_sync = 0;
reg dir_sel_sync = 0;
reg [7:0] motor_dc_sync;

reg [31:0] count = 32'h00000000;

// This is the signal that indicates that the next subcounter should start running
// the next subcounter is clocked based on positive and negative edges of this signal.
reg [LOG2_N_ENC_SAMPLING_SUBSEGS - 1:0] subseg_sel = 0;

assign DIR_SEL_OUT = dir_sel_sync;

// Synchronize inputs to clock
always @(posedge CLK) begin
    motor_enc_s1 <= MOTOR_ENC;
    motor_enc_sync <= motor_enc_s1;
    motor_dc_sync <= MOTOR_DC;
    dir_sel_sync <= DIR_SEL_RQ;
end

// Generates select for which subsegment counter to increment
// If we've counted high enough, then select the next subsegment
// Need to make sure we get through each subsegment per second
// That's why MAX_COUNT_NEXT_SUBSEG is based on the clock frequency
// and the number of sub-segments
// e.g. (clock_ticks / s) / (1s / 16 segments) = clock_ticks / segment = MAX_COUNT_NEXT_SUBSEG
always @(posedge CLK) begin
    if(count >= MAX_COUNT_NEXT_SUBSEG) begin
        subseg_sel <= subseg_sel + 1;
        count <= 32'h00000000;
    end
    else
        count <= count + 1;
end

// Module to clock PWM output
PwmGen
#(
    .RESET_POLARITY_LOW(1),
    .PWM_DC_WIDTH(8)
) MOTOR_PWM_GEN
(
    .CLK(CLK),
    .RESET(RESET),
    .IN_DC(motor_dc_sync),
    .OUT(MOTOR_EN)
);

// Module to count encoder ticks with sub intervals and rolling buffer concept
AddMuxCtr
#(
    .SUBCTR_WIDTH(8),
    .LOG2_N_SUB_CTRS(LOG2_N_ENC_SAMPLING_SUBSEGS)
) MOTOR_SUB_SEG_CTRS
(
    .CLK(CLK),
    .INC(motor_enc_sync),
    .SEL(subseg_sel),
    .SUM(total_enc_ticks_s)
);

// initialize everything.  Synthesis tool uses initial block for this
initial begin
    count <= 32'h00000000;
    motor_enc_s1 <= 1'b0;
    motor_enc_sync <= 1'b0;
    dir_sel_sync <= 1'b0;
    subseg_sel <= {LOG2_N_ENC_SAMPLING_SUBSEGS{1'b0}};
end  // initialization block
endmodule
