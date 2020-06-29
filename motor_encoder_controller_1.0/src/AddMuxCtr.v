module AddMuxCtr
#(
    parameter SUBCTR_WIDTH = 8,
    parameter LOG2_N_SUB_CTRS = 4
)
(
    input                           CLK,
    input                           INC,
    input [LOG2_N_SUB_CTRS - 1:0]    SEL,
    output reg [15:0]               SUM
);

localparam N_CTRS = 2**LOG2_N_SUB_CTRS;

integer i;

reg inc_last;

reg [15:0] SUM;

reg [SUBCTR_WIDTH - 1:0] cntrs[N_CTRS - 1:0];
reg [SUBCTR_WIDTH - 1:0] working_ctr = 0;
wire [LOG2_N_SUB_CTRS - 1:0] cur_sel;
reg [LOG2_N_SUB_CTRS - 1:0] last_sel = 0;
reg [15:0] sum = 0;

reg wkctr_reset = 0;

assign cur_sel = SEL;

    // This always block ensures that sig_dly is exactly 1 clock behind sig
always @ (posedge CLK) begin
    inc_last <= INC;
end
 
    // Combinational logic where sig is AND with delayed, inverted version of sig
    // Assign statement assigns the evaluated expression in the RHS to the internal net pe
assign inc_pe = INC & ~inc_last;    

// Whenever the increment signal has a positive edge,
// or when the working counter should be reset, set the
// working counter to 0 (if reset) or if not reset
// increment the working counter.
always@(posedge CLK or posedge wkctr_reset) begin
    if(wkctr_reset)
        working_ctr = 0;
    else begin
        if(inc_pe)
            working_ctr = working_ctr + 1;
    end
end

// On Clock, check if the current selected counter is different from
// the last selected counter.  This indicates a change in SEL, which
// means that we need to save the working counter's value into the
// array of counters.  Save it at the index as defined by last_sel.
// Then signal the working counter to be reset.
always@(posedge CLK) begin
    if(cur_sel != last_sel) begin
        cntrs[last_sel] <= working_ctr;
        wkctr_reset <= 1'b1;
    end
    else
        wkctr_reset <= 1'b0;
    last_sel = cur_sel;
end

// Combinational block to sum up the counters. Sensitivity list should
// include just the counters (cntrs).
always @* begin
    sum = 16'h0000; // all zero
    for(i = 0; i < N_CTRS; i=i+1)
        sum = sum + cntrs[i];
    SUM = sum;
end



initial begin
    //cur_sel <= {LOG2_N_SUB_CTRS{1'b0}};
    working_ctr <= 0;
    last_sel <= {LOG2_N_SUB_CTRS{1'b0}};
    for(i = 0; i < N_CTRS; i=i+1)
            cntrs[i] = {SUBCTR_WIDTH{1'b0}};
end  // initialization block

endmodule