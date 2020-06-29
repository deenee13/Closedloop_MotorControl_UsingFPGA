module PwmGen
#(
    parameter RESET_POLARITY_LOW = 1,
    parameter PWM_DC_WIDTH = 8
)
(
// port declarations
    input               CLK,
    input               RESET,
    input   [7:0]       IN_DC,
    output  reg         OUT
);

// use the RESET_POLARITY_LOW parameter to set the RESET_Int slevel
wire RESET_Int = RESET_POLARITY_LOW ? ~RESET : RESET;

reg [PWM_DC_WIDTH - 1:0] pwm_cntr;

// PWM channel counters
// counters overflow to restart PWM period
// If reset, make the counter max value so that the output stays low
always @(posedge CLK) begin
    if (RESET_Int) begin
    	pwm_cntr <= {PWM_DC_WIDTH{1'b1}};
    end
    else begin
    	pwm_cntr <= pwm_cntr + 1'b1;	
    end
end // pwm counters

// PWM output generation
// Block can be combinational because the counters are synchronized to the clock
always @* begin
    // control the red PWM channel
    if (pwm_cntr < IN_DC) 
        OUT = 1'b1;
    else
        OUT = 1'b0;
end // PWM output generation

initial begin
    OUT <= 1'b0;
    pwm_cntr <= {PWM_DC_WIDTH{1'b1}};
end  // initialization block
endmodule