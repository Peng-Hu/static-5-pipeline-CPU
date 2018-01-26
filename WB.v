module WB_stage (
	input wire	clk,
	input wire	resetn,
	
	/*MEM-WB Reg*/
	input wire [31:0]	pc_i,		//debug
	input wire [31:0]	inst_i,		//debug
	input wire [9:0]	wb_ctrl_i,
	input wire [31:0]	rdata_i,	//MemRdata
	input wire [31:0]	ALUOut_i,	//ALUOut
	output wire [31:0]	pc_o,
	output wire [31:0]	inst_o,
	input  wire [4:0]   	db_dest_i,

	/*to RF(ID stage)*/
	output wire [31:0]	wd1_o,		//debug
	output wire [4:0]	wa1_o,		//debug
	output wire		RegWrite_o	//debug	
);

reg [31:0] pc, inst, rdata, ALUOut;
reg [9:0] wb_ctrl;
reg [4:0] db_dest;
wire Mem2Reg;
wire [31:0] rdata_o;
wire [31:0] ALUOut_o;
wire        RegWrite;

always @(posedge clk)
begin
    if(~resetn)
    begin
        pc <= 'd0;
        inst <= 'd0;
        wb_ctrl <= 'd0;
        rdata <= 'd0;
        ALUOut <= 'd0;
        db_dest <= 'd0;
    end
    else
    begin
        pc <= pc_i;
        inst <= inst_i;
        wb_ctrl <= wb_ctrl_i;
        rdata <= rdata_i;
        ALUOut <= ALUOut_i;
        db_dest <= db_dest_i;
    end
end

assign pc_o = pc;
assign inst_o = inst;
assign rdata_o = rdata;
assign ALUOut_o = ALUOut;
assign RegWrite_o = RegWrite;
assign wa1_o = db_dest;
assign Mem2Reg = wb_ctrl[1];
assign RegWrite = wb_ctrl[0];

mux2 #(32) mux_wd1 (
	.mux_out	(wd1_o		),
	.m0_in		(rdata_o	),
	.m1_in		(ALUOut_o	),
	.sel_in		(Mem2Reg	)
);

endmodule
