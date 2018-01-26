module ID_stage(
    input wire		clk,
    input wire		resetn,

    /*IF-ID Reg*/
    input wire	[31:0]	pc_i,
    input wire	[31:0]	pc_4_i,
    input wire	[31:0]	inst_i,
    input   wire    pc_valid_i,

    /*RF & input from WB stage*/
    input wire [4:0]	wa1_i,
    input wire [31:0]	wd1_i,
    input wire		RegWrite_i,

    /*Output to EXE stage*/
    output wire [31:0]	rd1_o,
    output wire [31:0]	rd2_o,
    output wire [12:0]	wb_ctrl_o,
    output wire [18:0]	mem_ctrl_o,
    output wire [12:0]	exe_ctrl_o,
    output wire [31:0]	extend,
    output wire [31:0]	pc_o,
    output wire [31:0]  inst_o,

    /*Output to IF stage*/
    output reg [31:0]	new_pc,
    output reg		PCWriteCond,
    output wire     b_stop,
    output  wire    is_b,
    output wire     token,
    output wire		eret,
    output   wire    weap,
    /*input from forward unit */
    input  wire         stop,
    
    /*input to b forward unit*/
    input   wire	div_complete,
    input   wire    [4:0]   ex_db_dest,
    input   wire            ex_memread,
    input   wire    [4:0]   mem_db_dest,
    input   wire            memread,
    input   wire    [31:0]  mem_ALUout,
    input   wire    [4:0]   wb_dest,
    input   wire    [31:0]  wb_data,
    
    input   wire    [7:0]   id_tag_in,
    input   wire    [31:0]  id_badvaddr_i_i,
    output   wire    [31:0]  id_badvaddr_i_o,
    output  wire    [7:0]   id_tag_o,
    input   wire            sweap
);

/*IF-ID Reg start*/
reg [31:0]	pc, inst, pc_4;
reg [7:0]   tag;
reg [31:0]  badvaddr_i;
reg weap_hold;
wire [31:0]	pc_4_o;
wire    [31:0]  b_pc;
reg     pc_valid;

always @(posedge clk)
begin
    if(stop | b_stop)
    begin
            inst <= inst;
            pc  <=  pc;
            pc_4 <= pc_4;
    end
    else if(~resetn)
    begin
            pc <= 'd0;
            inst <= 'd0;
            pc_4 <= 'd0;
    end
    else
    begin
            pc <= pc_i;
            inst <= inst_i;
            pc_4 <= pc_4_i;
    end  
end
always @(posedge clk)
begin
    if(~resetn)
        pc_valid <= 1'b0;
    else if(pc_valid_i)
        pc_valid <= 1'b1;
    else
        pc_valid <= 1'b0;
end
assign pc_o = pc;
assign inst_o =  inst;
assign pc_4_o =  pc_4;
/*IF-ID Reg end*/


/*RF start*/
wire [4:0]	rs;
wire [4:0]	rt;
wire [4:0]	rd;
wire [31:0]	wd1_o, rd1_i, rd2_i;

assign rs = inst_o[25:21];
assign rt = inst_o[20:16];
assign rd = inst_o[15:11];

reg_file regfile_2r1w(
    .clk(clk),
	.rst(~resetn),
	.waddr(wa1_i), //写口
	.raddr1(rs), //读口
	.raddr2(rt),
	.wen(RegWrite_i), //写使能
	.wdata(wd1_i), //写数据
	.rdata1(rd1_i), //读数据
	.rdata2(rd2_i)
);
assign rd1_o = RegWrite_i ? ((rs == wa1_i && wa1_i != 'd0) ? wd1_i : rd1_i) : rd1_i ;
assign rd2_o = RegWrite_i ? ((rt == wa1_i && wa1_i != 'd0) ? wd1_i : rd2_i) : rd2_i;
assign wd1_o = wd1_i;
/*RF end*/

/*CU start*/
wire		b_hit;
wire		j_hit;
wire		jr_hit;
wire [5:0]	opcode;
wire [5:0]	funcode;
wire		ExtOp_o;
wire [1:0]  forward_rt;
wire [1:0]  forward_rs;
wire [31:0] rdata1;
wire [31:0] rdata2;
wire complete;
wire is_eret;
wire    is_RI;
wire    test_Ov;
reg    is_delayslot;

assign complete = div_complete;
assign opcode = inst_o[31:26];
assign funcode = inst_o[5:0];
assign rdata1 = (forward_rs == 'd0) ? rd1_i : ((forward_rs == 'd1) ? mem_ALUout : wb_data);
assign rdata2 = (forward_rt == 'd0) ? rd2_i : ((forward_rt == 'd1) ? mem_ALUout : wb_data);
assign eret = is_eret;
assign  id_tag_o = {tag[7:6],is_RI,is_delayslot,test_Ov,3'd0};
assign  id_badvaddr_i_o = badvaddr_i;
ctrl ctrl (
    .clk        (clk),
    .resetn     (resetn),
	.opcode		(opcode		),
	.funcode	(funcode	),
	.rs		(rs		),
	.rt		(rt		),
	.rdata1 (rdata1),
	.rdata2 (rdata2),
	.wb_ctrl	(wb_ctrl_o	),
	.mem_ctrl	(mem_ctrl_o	),
	.exe_ctrl	(exe_ctrl_o	),
	.ExtOp		(ExtOp_o	),
	.b_hit		(b_hit		),
	.is_b       (is_b      ),
	.token      (token),
	.j_hit		(j_hit		),
	.jr_hit		(jr_hit		),
	.complete	(complete	),
	.ex_db_dest    (ex_db_dest),
	.ex_memread    (ex_memread),
    .mem_db_dest    (mem_db_dest),
    .mem_wdata      (mem_ALUout),
    .memread        (memread),
    .wb_dest        (wb_dest),
    .wb_wdata       (wb_data),
    .b_stop         (b_stop),
    .forward_rt     (forward_rt),
    .forward_rs     (forward_rs),
    .is_eret		(is_eret),
    .is_RI          (is_RI),
    .test_Ov        (test_Ov),
    .sweap          (sweap)
);

always @(posedge clk)
begin
        if(~resetn | sweap)
        begin
            tag <= 'd0;
            badvaddr_i <= 'd0;
        end
        else if(pc_valid_i)
        begin
            tag <= id_tag_in;
            badvaddr_i <= id_badvaddr_i_i;
        end
        else
        begin
            tag <= 'd0;
            badvaddr_i <= 'd0;
        end
end

always @(posedge clk)
begin
        if(~resetn | sweap)
            is_delayslot <= 1'b0;
        else if((j_hit|jr_hit|is_b) & ~b_stop)
            is_delayslot <= 1'b1;
        else if(is_delayslot && (pc_valid))
            is_delayslot <= 1'b0;
        else
            is_delayslot <= is_delayslot;
end

always @(posedge clk)
begin
        if(~resetn | sweap)
            weap_hold <= 1'b0;
        else if(PCWriteCond | token)
            weap_hold <= 1'b1;
        else if(~is_delayslot)
            weap_hold <= 1'b0;
        else
            weap_hold <= weap_hold;
end

assign  weap = (weap_hold | token) & is_delayslot & (pc_valid);
/*CU end*/
wire [15:0]	imm;
assign imm = inst_o[15:0];

expander #(16) expander (
	.data_i		(imm		),
	.ExtOp		(ExtOp_o	),
	.data_o		(extend	)
);

/*Next PC generate start*/
wire [31:0]	SEI;
wire [31:0]	j_pc;
wire [31:0]	jr_pc;
assign SEI = {extend[29:0], 2'b00};
assign j_pc = {pc_4_o[31:28], inst_o[25:0], 2'b00};
assign jr_pc = rdata1;

adder adder_0 (
	.A		(pc_4_o		),
	.B		(SEI		),
	.add		(b_pc		)
);
always @(*)
begin
    if(~resetn | sweap)
            new_pc = 'd0;
    else if(is_b | j_hit | jr_hit)
            new_pc = ({32{is_b}} & b_pc) | ({32{j_hit}} & j_pc) | ({32{jr_hit}} & jr_pc);
    else
            new_pc = 'd0;
end
always@(*)
begin
    if(~resetn | sweap)
            PCWriteCond = 1'b0;
    else if((b_hit | j_hit | jr_hit) &~b_stop &~stop )
            PCWriteCond = 1'b1;
    else
            PCWriteCond = 1'b0;
end
/*Next PC generate end*/

endmodule //decode_stage
