`timescale 1ns/1ns

module branch_ctrl(
	input wire [31:0] pc, inst, data1, branch_pc,
	output reg [31:0] wb_pc,
	output wire branch
);

wire        taken;     //1: branch taken, go to the branch target
wire        is_br;     //1: target is PC+offset
wire	    is_bn;
wire        is_j;      //1: target is PC||offset
wire        is_jr;     //1: target is GR valu
    
wire [5:0]  opcode;
wire [5:0]  funcode;
wire [4:0]  rs;
wire [4:0]  rt;

assign opcode = inst[31:26];
assign funcode = inst[5:0];
assign rs = inst[25:21];
assign rt = inst[20:16];
assign is_br = (opcode == `BEQ);
assign is_bn = (opcode == `BNE);
assign is_j = (opcode == `J);
assign is_jr = (opcode == `SPEC && funcode == `JR) | (opcode == `JAL);
assign taken = (is_br & (rs == rt)) | (is_bn & (rs != rt));
assign branch = is_j | is_jr | taken;

always @(*)
begin
	if(taken)
	begin
		wb_pc = branch_pc;
	end
	else if(is_j)
	begin
		wb_pc = {pc[31:28], inst[25:0], 2'b0};
	end
	else if(is_jr)
	begin
		wb_pc = data1;
	end
	else
	   wb_pc  = 'd0;
end

endmodule
