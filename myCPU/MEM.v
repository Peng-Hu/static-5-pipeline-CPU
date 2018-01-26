module MEM_stage (
	input wire	clk,
	input wire	resetn,

	/*EXE-MEM Reg*/
	input wire [31:0]	pc_i,		//debug
	input wire [31:0]	inst_i,		//debug
	input wire [31:0]	wdata_i,	//rd2
	input wire [12:0]	wb_ctrl_i,
	input wire [16:0]	mem_ctrl_i,
	input wire [31:0]	ALUOut_i,	//Mem_waddr, wd1
	input  wire    [7:0]   mem_tag_i,
	output wire    [7:0]   mem_tag_o,
	output wire [31:0]	pc_o,
	output wire [31:0]	inst_o,
	output wire [31:0]	wdata_o,
	output wire [12:0]	wb_ctrl_o,
	input wire  [4:0]  db_dest_i,
	input  wire    [31:0]  badvaddr_i_i,
	input  wire    [31:0]  badvaddr_d_i,
	output reg    [31:0]  badvaddr_i_o,
	output reg    [31:0]  badvaddr_d_o,
	input  wire            is_read,
	input  wire            is_write,

	/*D-RAM & to WB stage*/
	output wire  is_mflo,
	output wire  is_mfhi,
	output wire        is_mfc0,
	input wire [31:0]	rdata_i,
	input  wire    data_ready,
	input  wire    rdata_valid,
	output reg     mem_data_valid,
	output wire		MemRead_o,
	output wire [3:0]	MemWrite_o,
	output wire [31:0]	rdata_o,
	output wire [31:0]	ALUOut_o,		//reg write data
	output wire [4:0] 	db_dest_o,
	output wire  mem_stop_o,
	output reg     [31:0]  rd2_o
);

reg [31:0] pc, inst, wdata, ALUOut;
reg [12:0] wb_ctrl;
reg [4:0] db_dest;
reg [16:0] mem_ctrl;
reg [7:0]   tag;
reg         mem_stop;
reg         r_a_w_wait;
assign  mem_stop_o = mem_stop | r_a_w_wait;
//12:11 sel, 10 unsign, 9:5 read, 4:0 write
wire r4, lwl, lwr, r2, r1;
wire w4, swl, swr, w2, w1;
wire unsign;
wire [1:0] w_sel;
wire [1:0] r_sel;
wire [35:0] swlr;
wire [31:0] lwlr;
wire [31:0] r2_data, r1_data, w1_data, w2_data;
wire [7:0] r1_data_mul;
wire [15:0] r2_data_mul;
wire [3:0] mw1, mw2;

always @(posedge clk)
begin
    if(~resetn | r_a_w_wait)
    begin
        pc <= 'd0;
        inst <= 'd0;
        wdata <= 'd0;
        ALUOut <= 'd0;
        wb_ctrl <= 'd0;
        mem_ctrl <= 'd0;
        db_dest  <= 'd0;
        tag <= 'd0;
        badvaddr_i_o <= 'd0;
        badvaddr_d_o <= 'd0;
        rd2_o   <= 'd0;
    end
    else if(mem_stop)
    begin
            pc <= pc;
            inst <= inst;
            wdata <= wdata;
            ALUOut <= ALUOut;
            wb_ctrl <= wb_ctrl;
            mem_ctrl <= mem_ctrl;
            db_dest  <= db_dest;
            tag <= tag;
            badvaddr_d_o <= badvaddr_d_o;
            badvaddr_i_o <= badvaddr_i_o;
            rd2_o   <=  rd2_o;
    end
    else
    begin
        pc <= pc_i;
        inst <= inst_i;
        wdata <= wdata_i;
        ALUOut <= ALUOut_i;
        wb_ctrl <= wb_ctrl_i;
        mem_ctrl <= mem_ctrl_i;
        db_dest  <= db_dest_i;
        tag <= mem_tag_i;
        badvaddr_i_o <= badvaddr_i_i;
        badvaddr_d_o <= badvaddr_d_i;
        rd2_o   <=  wdata_i;
    end	
end
always @(*)
begin
        if(~resetn)
            mem_stop = 1'b0;
        else if(~(data_ready) && (MemRead_o|(|MemWrite_o)))
                mem_stop = 1'b1;
        else if(MemRead_o && ~rdata_valid)
            mem_stop = 1'b1;
        else if(MemRead_o && rdata_valid)
            mem_stop = 1'b0;
        else
            mem_stop = 1'b0;
end

always @(*)
begin
        if(~resetn)
            r_a_w_wait = 1'b0;
        else if((is_read|is_write) && (MemRead_o|(|MemWrite_o)) && ~mem_stop)
            r_a_w_wait = 1'b1;
        else
            r_a_w_wait = 1'b0;
end
always @(posedge clk)
begin
        if(~resetn)
            mem_data_valid <= 1'b0;
        else if((|mem_ctrl_i[9:0]) && ~mem_tag_i[0]  && ~mem_data_valid && ~mem_stop)
            mem_data_valid <= 1'b1;
        //else if((|mem_ctrl[9:0]) && ~mem_tag_i[0] && ~mem_data_valid)
        //    mem_data_valid <= 1'b1;
        else if(data_ready && mem_data_valid)
            mem_data_valid <= 1'b0;
        else
            mem_data_valid <= mem_data_valid;
end

assign is_mflo = mem_ctrl[11];
assign  is_mfhi = mem_ctrl[12];
assign  is_mfc0 = wb_ctrl[7];
assign pc_o = pc;
assign inst_o = inst;
assign  mem_tag_o = tag;
//assign wdata_o = wdata_i;
//assign MemRead_o = mem_ctrl[1];
//assign MemWrite_o = mem_ctrl_i[0];
assign wb_ctrl_o = mem_stop ? 'd0 : wb_ctrl;
//assign rdata_o = rdata_i;
assign ALUOut_o = ALUOut;
assign db_dest_o = mem_stop ? 'd0 : db_dest;

assign {unsign,r4,lwl,lwr,r2,r1} = mem_ctrl[10:5];
assign {w4, swl,swr, w2, w1} = mem_ctrl[4:0];
assign r_sel = mem_ctrl[16:15];
assign w_sel = mem_ctrl[16:15];
assign MemRead_o = (|mem_ctrl[9:5]) ;
assign MemWrite_o = ({4{w4}} | mw2 | mw1 | ({4{swl|swr}} & swlr[35:32])) ;
assign wdata_o = ({32{w4}} & wdata) | ({32{w2}} & w2_data) | ({32{w1}} & w1_data) | ({32{swl|swr}} & swlr[31:0]);
assign rdata_o = ({32{r4}} & rdata_i) | ({32{lwl|lwr}} & lwlr) | ({32{r2}} & r2_data) | ({32{r1}} & r1_data);
assign r2_data = {{16{~unsign & r2_data_mul[15]}}, r2_data_mul};
assign r1_data = {{24{~unsign & r1_data_mul[7]}}, r1_data_mul};
assign w2_data = {2{wdata[15:0]}};
assign w1_data = {4{wdata[7:0]}};
assign mw1 = {4{w1}} & {(w_sel[1] & w_sel[0]), (w_sel[1] & ~w_sel[0]), (~w_sel[1] & w_sel[0]), (~w_sel[1] & ~w_sel[0])};
assign mw2 = {4{w2}} & {{2{w_sel[1]}}, {2{~w_sel[1]}}};

assign r2_data_mul = r_sel[1] ? rdata_i[31:16] : rdata_i[15:0];

mux4 #(8) mux_r1_data (
	.mux4_out	(r1_data_mul),
	.m0_in		(rdata_i[7:0]),
	.m1_in		(rdata_i[15:8]),
	.m2_in		(rdata_i[23:16]),
	.m3_in		(rdata_i[31:24]),
	.sel_in		(r_sel)
);

mux8 #(36) mux_swlr (
	.mux8_out	(swlr	),
	.m0_in		({4'b0001, 24'b0, wdata[31:24]}),
	.m1_in		({4'b0011, 16'b0, wdata[31:16]}),   
	.m2_in		({4'b0111, 8'b0, wdata[31:8]}),
	.m3_in		({4'b1111, wdata[31:0]}),
	.m4_in		({4'b1111, wdata[31:0]}),
	.m5_in		({4'b1110, wdata[23:0], 8'b0}),
	.m6_in		({4'b1100, wdata[15:0], 16'b0}),
	.m7_in		({4'b1000, wdata[7:0], 24'b0}),
	.sel_in		({swr, w_sel})
);

mux8 #(32) mux_lwlr (
	.mux8_out	(lwlr	),
	.m0_in		({rdata_i[7:0], wdata[23:0]}),
	.m1_in		({rdata_i[15:0], wdata[15:0]}),
	.m2_in		({rdata_i[23:0], wdata[7:0]}),
	.m3_in		(rdata_i),
	.m4_in		(rdata_i),
	.m5_in		({wdata[31:24], rdata_i[31:8]}),
	.m6_in		({wdata[31:16], rdata_i[31:16]}),
	.m7_in		({wdata[31:8], rdata_i[31:24]}),
	.sel_in		({lwr, r_sel})
);

endmodule
