module EX_stage(
	input wire	clk,
	input wire	resetn,

	/*ID-EXE Reg*/
	input wire [31:0]	pc_i,
	input wire [31:0]	inst_i,
	input wire [31:0]	rd1_i,
	input wire [31:0]	rd2_i,
	input wire [31:0]	extend_i,	
	input wire [9:0]	wb_ctrl_i,
	input wire [18:0]	mem_ctrl_i,
	input wire [12:0]	exe_ctrl_i,

	output wire [31:0]	pc_o,
	output wire [31:0]	inst_o,
	output wire [31:0]	rd2_o,
	output	wire [31:0]	rd1_o,
	output wire [9:0]	wb_ctrl_o,
	output wire [16:0]	mem_ctrl_o,
	output wire [4:0]  db_dest,

	/*Output to MEM stage*/
	output wire [31:0]	ALUOut,
	input  wire [31:0] ALUout_i,
	
	/*Output to mul&div*/
	output	wire [1:0]	mul_con_o,
	output wire [1:0]	div_con_o,
	/*input from WB stage*/
	input  wire [31:0] wb_i,
	
	/*input from forward unit*/
	output wire [1:0]  ALUSrcB,
	output wire        ALUSrcA,
	input  wire        stop,
	input  wire [1:0]  ForwardA,
	input  wire [1:0]  ForwardB,
	input  wire	ex_btaken_i,
	output wire	ex_btaken_o,
	
	input  wire    [6:0]   exe_tag_i,
	output wire    [6:0]   exe_tag_o,
	input  wire    [31:0]  exe_badvaddr_i_i,
	output wire    [31:0]  exe_badvaddr_d_o,
	output wire    [31:0]  exe_badvaddr_i_o
);

reg [12:0]	exe_ctrl;
reg [18:0]	mem_ctrl;
reg [9:0]	wb_ctrl;
reg [31:0]	pc, inst;
reg [31:0]	rd1, rd2;
reg [31:0]	extend;
reg [31:0] wb_hold_reg;
reg         rdata1_hold;
reg         rdata2_hold;
reg		ex_btaken;
reg [31:0]  exe_badvaddr_i;

wire    is_ades;
wire    is_Ov;
wire    is_adel;
reg [6:0]   tag;
assign  is_adel = (mem_ctrl[18]&(ALUOut[0]|ALUOut[1])) | (mem_ctrl[17]&ALUOut[0]);
assign  is_ades = (mem_ctrl[16]&(ALUOut[0]|ALUOut[1])) | (mem_ctrl[15]&ALUOut[0]);
assign  exe_tag_o = {tag[6],tag[5:3],is_Ov & tag[3],is_adel,is_ades} ;
assign  exe_badvaddr_d_o = {{32{is_adel | is_ades}} & ALUOut};
assign  exe_badvaddr_i_o = exe_badvaddr_i;
always @(posedge clk)
begin
        if(~resetn)
        begin
            tag <= 'd0;
            exe_badvaddr_i <= 'd0;
        end
        else
        begin
            tag <= exe_tag_i;
            exe_badvaddr_i <= exe_badvaddr_i_i;
        end
end

always @(posedge clk)
begin
        if(ForwardA == 'd2 && stop == 1'b1)
                wb_hold_reg <= wb_i;
        else if(ForwardB == 'd2 && stop == 1'b1)
                wb_hold_reg <= wb_i;
        else
                wb_hold_reg <= 'd0;
end

always @(posedge clk)
begin
        if(ForwardA == 'd2 && stop == 1'b1)
                rdata1_hold <= 1'b1;
        else
                rdata1_hold <= 1'b0;
end

always @(posedge clk)
begin
        if(ForwardB == 'd2 && stop == 1'b1)
                rdata2_hold <= 1'b1;
        else
                rdata2_hold <= 1'b0;
end
always @(posedge clk)
begin
    if(~resetn)
    begin
                exe_ctrl <= 'd0;
                mem_ctrl <= 'd0;
                wb_ctrl <= 'd0;
                pc <= 'd0;
                rd1 <= 'd0;
                rd2 <= 'd0;
                inst <= 'd0;
                extend <= 'd0;
                ex_btaken <= 'd0;
    end
    else if(stop == 1'b1)
    begin
            exe_ctrl <= exe_ctrl;
            mem_ctrl <= mem_ctrl;
            wb_ctrl <= wb_ctrl;
            pc <= pc;
            rd1 <= rd1;
            rd2 <= rd2;
            inst <= inst;
            extend <= extend;
	    ex_btaken <= ex_btaken;
    end
    else
    begin
	       exe_ctrl <= exe_ctrl_i;
	       mem_ctrl <= mem_ctrl_i;
	       wb_ctrl <= wb_ctrl_i;
	       pc <= pc_i;
	       rd1 <= rd1_i;
	       rd2 <= rd2_i;
	       inst <= inst_i;
	       extend <= extend_i;
	       ex_btaken <= ex_btaken_i;
	end
end
assign	mul_con_o = stop ? 'd0 : exe_ctrl[10:9];
assign	div_con_o = stop ? 'd0 : exe_ctrl[12:11];
assign	wb_ctrl_o = stop ? 'd0 : wb_ctrl;
assign	pc_o = pc;
assign	rd2_o = rdata2_hold ? wb_hold_reg : (ForwardB == 'd0 ? rd2 : (ForwardB == 'd1 ? ALUout_i : wb_i));
assign	rd1_o = rdata1_hold ? wb_hold_reg : (ForwardA == 'd0 ? rd1 : (ForwardA == 'd1 ? ALUout_i : wb_i));
assign inst_o = inst;

wire [1:0] byte_sel;
assign byte_sel = ALUOut[1:0];
assign mem_ctrl_o = stop ? 'd0 : {byte_sel, mem_ctrl[14:0]};
assign ex_btaken_o = ex_btaken;

wire [31:0] A, B;
wire [5:0] ALUop;
wire [4:0] sa;

assign sa = inst_o[10:6];
assign ALUSrcB = exe_ctrl[1:0];
assign ALUSrcA = exe_ctrl[2];
assign ALUop = exe_ctrl[8:3];

mux4 #(32) mux_B (
	.mux4_out	(B		),
	.m0_in		(rd2_o		),
	.m1_in		(extend	),
	.m2_in		(32'd8		),
	.m3_in		(32'd0		),
	.sel_in		(ALUSrcB	)
);

mux2 #(32) mux_A (
	.mux_out	(A		),
	.m0_in		(rd1_o		),
	.m1_in		(pc_o		),
	.sel_in		(ALUSrcA	)
);

alu alu (
	.ALUop		(ALUop		),
	.A		(A		),
	.B		(B		),
	.shamt		(sa		),
	.result		(ALUOut	),
	.is_Ov      (is_Ov)
);

mux4 #(5) mux_db_dest (
	.mux4_out	(db_dest		),
	.m0_in		(inst_o[20:16]	),	//rt
	.m1_in		(inst_o[15:11]	),	//rd
	.m2_in		(5'd31		),
	.m3_in		(5'd0		),
	.sel_in		(wb_ctrl[3:2]		)
);

endmodule
