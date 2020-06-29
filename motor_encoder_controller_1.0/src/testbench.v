`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/07/2020 08:24:47 PM
// Design Name: 
// Module Name: testbench
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module testbench;

    reg S_AXI_ACLK;
    reg S_AXI_ARESETN;
    reg [31:0] slv_reg0;
    
    wire [15:0] motor_fb_wire;
    
    reg MOTOR_ENC;
    
    wire DIR_SEL_OUT;
    wire MOTOR_EN;
    
    initial begin
        S_AXI_ACLK = 1'b0;
        S_AXI_ARESETN = 1'b1;
        slv_reg0 = {23'd0, 1'b1, 8'd204};
        MOTOR_ENC = 1'b0;        
    end
    
    always
        #275000 MOTOR_ENC = !MOTOR_ENC;
    
    always
        #5 S_AXI_ACLK = !S_AXI_ACLK;

	// UUT
		MotorController
    #(
        .RESET_POLARITY_LOW(1),                // Reset is active-low?  (default is yes)
        .PWM_DC_WIDTH(8)                    // Number of duty cycle bits for each channel (Default = 9-bit or 1/512 resolution)
    ) uut
    (
        // port declarations
        .CLK(S_AXI_ACLK),                         // 100MHz clock
        .RESET(S_AXI_ARESETN),                        // system reset
        // Inputs from Application via GPIO
        .DIR_SEL_RQ(slv_reg0[8]),                    // Rq from Software 1 is CW, 0 is CCW
        .MOTOR_DC(slv_reg0[7:0]),                   // Duty cycle for motor output [0 - 255]
        // Outputs to Application via GPIO
        .MOTOR_FB_RPS(motor_fb_wire),                // Motor Feedback rotations per second
        // Inputs from Motor
        .MOTOR_ENC(MOTOR_ENC),                  // Raw encoder input for calculating motor speed
        // Outputs to Motor
        .DIR_SEL_OUT(DIR_SEL_OUT),
        .MOTOR_EN(MOTOR_EN)
    );
    
endmodule
