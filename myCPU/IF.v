module IF_stage(
	input wire		clk,
	input wire		resetn,

	input  wire [31:0]	irom_inst_i,
	input  wire    [31:0]  irom_pc_i,
	input  wire        inst_valid,
	input  wire        PC_ready,
	output wire        inst_ready,
	output reg        PC_valid,
	output wire [31:0]	irom_pc_o,
	
	output wire [31:0]	pc_o,
	output wire [31:0]	pc_4_o,
	output wire [31:0]	inst_o,

	input wire [31:0]	wb_pc,
	input wire 		PCWriteCond,
	input wire         stop,
	
	input  wire    token,
	input  wire		is_int,
	input  wire [31:0]	int_pc,
	input  wire    is_b
);
wire    wb_pc_ack;
reg [31:0] pc, inst,PC_b,wb_pc_hold;
reg token_hold,pc_w;
wire [31:0] pc_i,pc_4;
wire [31:0] new_pc;
always @(posedge clk)
begin
	pc <= resetn ? (is_int ? new_pc :((PC_valid & PC_ready) ?  new_pc : pc)) : 32'hbfc00000; 
	//inst <= resetn ?  irom_inst_i : 32'd0;
	PC_b <= resetn ?  (is_b ? wb_pc : PC_b) : 32'd0;
end

always @(posedge clk)
begin
    if(~resetn)
        PC_valid <= 1'b0;
    else if(~PC_valid )
        PC_valid <= 1'b1;
    else if(PC_ready)
        PC_valid <= 1'b0;
    else
        PC_valid <= PC_valid;  
end
always @(posedge clk)
begin
    if(~resetn)
            token_hold <= 1'b0;
    else if(~(PC_valid & PC_ready) & token)
            token_hold <= token;
    else if(wb_pc_ack)
            token_hold <= 1'b0;
    else
            token_hold <= token_hold;
end
always @(posedge clk)
begin
    if(~resetn)
    begin
             pc_w <= 1'b0;
             wb_pc_hold <= 'd0;
    end
    else if(~(PC_valid & PC_ready) & PCWriteCond)
    begin
            pc_w <= PCWriteCond;
            wb_pc_hold <= wb_pc;
    end
    else if(wb_pc_ack)
    begin
            pc_w <= 1'b0;
            wb_pc_hold <= 'd0;
    end
    else
    begin
            pc_w<= pc_w;
            wb_pc_hold <= wb_pc_hold;
    end
end
adder adder_0 (
	.A		(pc_o		),
	.B		(32'd4		),
	.add		(pc_4_o		)
);

adder adder_1 (
	.A		( pc		),
	.B		(32'd4		),
	.add		(pc_4		)
);

mux4 #(32) mux_pc (
	.mux4_out	(pc_i		),
	.m0_in		(pc_4  	),
	.m1_in		(pc    		),
	.m2_in		((wb_pc | wb_pc_hold)		),
	.m3_in      	(pc         ),
	.sel_in		({(PCWriteCond | pc_w),1'b0}	)
);

assign new_pc = is_int ? int_pc : ((token|token_hold) ? PC_b : pc_i);
assign irom_pc_o = resetn ?  pc: 32'hbfc00000;
// can not use reg pc here because of the latency of i-rom
assign pc_o = (inst_valid & inst_ready & ~is_int) ? irom_pc_i : 32'd0;
assign inst_o = (inst_valid & inst_ready & ~is_int) ? irom_inst_i : 32'h00000000;
assign  inst_ready = ~stop;
assign  wb_pc_ack = (PC_valid & PC_ready);
endmodule

// actually is PC register
