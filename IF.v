module IF_stage(
	input wire		clk,
	input wire		resetn,

	input wire [31:0]	irom_inst_i,
	output wire [31:0]	irom_pc_o,
	
	output wire [31:0]	pc_o,
	output wire [31:0]	pc_4_o,
	output wire [31:0]	inst_o,

	input wire [31:0]	wb_pc,
	input wire 		PCWriteCond,
	input wire         stop,
	
	input wire [31:0]  PC_from_buf,
	input  wire    token,
	input wire		is_int,
	input wire [31:0]	int_pc
);

reg [31:0] pc, inst;
wire [31:0] pc_i;
wire [31:0] new_pc;

always @(posedge clk)
begin
	pc <= resetn ? new_pc : 32'hbfc00000; 
	inst <= resetn ?  irom_inst_i : 32'b0;
end

adder adder_0 (
	.A		(pc_o		),
	.B		(32'd4		),
	.add		(pc_4_o		)
);

mux4 #(32) mux_pc (
	.mux4_out	(pc_i		),
	.m0_in		(pc_4_o  	),
	.m1_in		(pc    		),
	.m2_in		(wb_pc		),
	.m3_in      	(pc         ),
	.sel_in		({PCWriteCond,stop}	)
);

assign new_pc = is_int ? int_pc : (token ? PC_from_buf : pc_i);
assign irom_pc_o = resetn ? new_pc : 32'hbfc00000;
// can not use reg pc here because of the latency of i-rom
assign pc_o = pc;
assign inst_o = resetn ? irom_inst_i : 32'h00000000;

endmodule

// actually is PC register
