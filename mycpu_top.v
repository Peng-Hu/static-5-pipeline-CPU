/*------------------------------------------------------------------------------
--------------------------------------------------------------------------------
Copyright (c) 2016, Loongson Technology Corporation Limited.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of Loongson Technology Corporation Limited nor the names of 
its contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL LOONGSON TECHNOLOGY CORPORATION LIMITED BE LIABLE
TO ANY PARTY FOR DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------------
------------------------------------------------------------------------------*/

module mycpu_top(
    input  wire        clk,
    input  wire        resetn,            //low active

    output wire        inst_sram_en,
    output wire [ 3:0] inst_sram_wen,
    output wire [31:0] inst_sram_addr,
    output wire [31:0] inst_sram_wdata,
    input  wire [31:0] inst_sram_rdata,
    output wire [31:0] inst_sram_addr2,
    input  wire [31:0] inst_sram_rdata2,
    
    output wire        data_sram_en,
    output wire [ 3:0] data_sram_wen,
    output wire [31:0] data_sram_addr,
    output wire [31:0] data_sram_wdata,
    input  wire [31:0] data_sram_rdata,

    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_wen,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,
    
    input   wire    [7:0]   int
);
// we only need an inst ROM now
assign inst_sram_wen   = 4'b0;
assign inst_sram_wdata = 32'b0;
wire    stop;
wire    [1:0]   ForwardA;
wire    [1:0]   ForwardB;
wire [4:0]  db_dest;
wire Memread;
wire [4:0]  wb_dest_i;
wire [31:0] mem_rdata_o, mem_ALUOut_o;
wire    sweap;
/****************** IF stage *****************/
wire [31:0]	if_irom_pc_o;
wire [31:0]	if_pc_o;
wire [31:0]	if_pc_4_o;
wire [31:0]	if_inst_o;
wire [31:0] id_wb_pc_o;
wire		id_wb_PCWC_o;
wire        b_stop;
wire [31:0] pc_from_buf;
wire        token;
wire        j_token;
wire		is_int;
wire [31:0] eret_pc;
wire [31:0] int_pc;
wire sr_exl, sr_bev;
wire is_eret;
wire [31:0] cp0_epc;
wire    syscall;
wire    break;
wire [9:0] id_wb_ctrl_o;
wire    is_adel;
wire    [6:0]   if_tag;
wire    [31:0]  if_badvaddr;
wire    is_exception;
wire    ENTR;

IF_stage IF_stage (
	.clk		(clk		),
	.resetn		(resetn		),

	.irom_inst_i	(inst_sram_rdata	),
	.irom_pc_o	(if_irom_pc_o	),
	.pc_o		(if_pc_o	),
	.pc_4_o		(if_pc_4_o	),
	.inst_o		(if_inst_o	),
	.wb_pc		(id_wb_pc_o	),
	.PCWriteCond	(id_wb_PCWC_o	),
	.stop          (stop | b_stop),
	.PC_from_buf   (pc_from_buf),
	.token         (j_token),
	.is_int		(is_int),
	.int_pc		(int_pc)
);
assign  syscall = id_wb_ctrl_o[8];
assign  break = id_wb_ctrl_o[9];
assign inst_sram_addr = if_irom_pc_o;
assign inst_sram_en = 1'b1;
assign is_int = is_eret | syscall | break |is_exception | ENTR;
assign eret_pc = cp0_epc;
assign int_pc = ({32{is_eret}} & cp0_epc) | ({32{((~sr_exl & ENTR) | is_exception | syscall | break) & sr_bev}} & 32'hbfc00380);

wire [31:0] token_inst;
wire    [31:0]  pc_to_id;
wire        is_b;
wire    [31:0]  b_pc;
inst_buf inst_buf(
    .clk(clk ),
	.resetn(resetn & ~sweap),
	
	.stop (stop),
	.j_token (j_token),
	.pc_to_ins(inst_sram_addr2),
	.pc_to_id(pc_to_id),
	.pc_to_if(pc_from_buf),
	.token_inst(token_inst),
	.j_pc(b_pc),
	.is_j(is_b),
	.token(token),
	.j_inst(inst_sram_rdata2)
);
assign  is_adel = (~(is_eret | syscall | break)) & ((j_token & (pc_to_id[1]|pc_to_id[0]))| (~j_token & (if_pc_o[1] | if_pc_o[0])));
assign  if_badvaddr = {{32{j_token}}&pc_to_id | {32{~j_token}}&if_pc_o};
assign  if_tag = {is_adel,6'd0};
// 读数据需要一个周期，最好别用PC reg里的PC
/********************* end ***********************/

/******************* ID stage ********************/
wire [31:0] id_pc_o, id_inst_o;
wire [4:0] wb_wa1_o;
wire	[31:0]	reg_wdata;
wire [31:0] wb_wd1_o;
wire wb_RegWrite_o;
wire [31:0] id_rd1_o, id_rd2_o, extend_o;
wire [18:0] id_mem_ctrl_o;
wire [12:0] id_exe_ctrl_o;
wire [31:0] id_extend_o;
wire	div_complete;
wire [31:0] ALU_final;
wire id_btaken_o;
wire    [6:0] id_tag;
wire   [31:0] id_badvaddr_i;

ID_stage ID_stage (
    .clk            (clk),
    .resetn         (resetn),
    /*IF-ID Reg*/
    .pc_i   		({{32{~j_token & ~is_int & ~sweap}} & if_pc_o}|{{32{~is_int & ~sweap}} & pc_to_id}	),
    .pc_4_i 		({{32{~j_token & ~is_int& ~sweap}} &if_pc_4_o}|{{32{~is_int & ~sweap}} &pc_from_buf}	),
    .inst_i 		({{32{~j_token & ~is_int& ~sweap}} &if_inst_o}|{{32{~is_int & ~sweap}} & token_inst}	),
	.pc_o		    (id_pc_o	),
	.inst_o		      (id_inst_o	),
	.b_stop         (b_stop    ),
	.is_b           (is_b),
	.token          (token),
	.b_pc            (b_pc),
    /*RF & input from WB stage*/
	.wa1_i		   (wb_wa1_o	),
	.wd1_i		   (reg_wdata	),
	.RegWrite_i	   (wb_RegWrite_o	),
    /*Output to EXE stage*/
	.rd1_o		   (id_rd1_o	),
	.rd2_o		   (id_rd2_o	),
	.wb_ctrl_o	    (id_wb_ctrl_o	),
	.mem_ctrl_o	   (id_mem_ctrl_o	),
	.exe_ctrl_o	   (id_exe_ctrl_o	),
    .extend	        (id_extend_o	),
    .new_pc         (id_wb_pc_o    ),
    .PCWriteCond    (id_wb_PCWC_o    ),
    .eret		(is_eret),
    .stop           (stop),
    .id_btaken_o	(id_btaken_o	),
    /*input to b forward unit*/
    .div_complete		(div_complete),
    .ex_db_dest         ({5{ex_wb_ctrl_o[0]}} & db_dest),
    .ex_memread         (|ex_mem_ctrl_o[9:5]),
    .mem_db_dest        ({5{mem_wb_ctrl_o[0]}} & wb_dest_i),
    .memread            (Memread),
    .mem_ALUout         (ALU_final),
    .wb_dest            ({5{wb_RegWrite_o}} & wb_wa1_o),
    .wb_data            (reg_wdata),
    .id_tag_in          (if_tag),
    .id_tag_o           (id_tag),
    .id_badvaddr_i_i    (if_badvaddr),
    .id_badvaddr_i_o    (id_badvaddr_i),
    .sweap              (sweap)
);
//s/.*\[.*\]//gi
//s/\t*\s*\(.*\),/\t\1\t\t(if_\1\t),/gi
//内部包含寄存器堆，控制单元，分支选择单元
/******************* end *********************/

/****************** EX stage ********************/
wire [31:0]	ex_pc_o, ex_inst_o, ex_rd2_o,ex_rd1_o;
wire [9:0]	ex_wb_ctrl_o;
wire [16:0] 	ex_mem_ctrl_o;
wire [31:0]	ex_ALUOut_o;
wire [1:0]	mul_con_o;
wire [1:0]	div_con_o;
wire [1:0] ALUSrcB;
wire        ALUSrcA;
wire	ex_btaken_o;
wire    [6:0] ex_tag;
wire    [31:0]  exe_badvaddr_i;
wire    [31:0]  exe_badvaddr_d;
EX_stage EX_stage (
	.clk		(clk		),
	.resetn		(resetn	& ~sweap	),

	.pc_i		(id_pc_o	),	//PC
	.inst_i		(id_inst_o	),
	.rd1_i	(id_rd1_o	),
	.rd2_i	(id_rd2_o	),
	.extend_i	(id_extend_o	),
	.wb_ctrl_i	(id_wb_ctrl_o	),
	.mem_ctrl_i	(id_mem_ctrl_o	),
	.exe_ctrl_i	(id_exe_ctrl_o	),

	.pc_o		(ex_pc_o	),	//PC
	.inst_o		(ex_inst_o	),
	.rd2_o	(ex_rd2_o	),
	.rd1_o	(ex_rd1_o),
	.wb_ctrl_o	(ex_wb_ctrl_o	),
	.mem_ctrl_o	(ex_mem_ctrl_o	),
	.ALUOut	(ex_ALUOut_o	),
	.db_dest   (db_dest),
	.mul_con_o	(mul_con_o),
	.div_con_o	(div_con_o),
	
	.ALUout_i(ALU_final),
     .wb_i(reg_wdata),
     .ALUSrcB(ALUSrcB),
     .ALUSrcA(ALUSrcA),
     .stop(stop),
     .ForwardA(ForwardA),
     .ForwardB(ForwardB),
     .ex_btaken_i	(id_btaken_o	),
     .ex_btaken_o	(ex_btaken_o	),
     .exe_tag_o(ex_tag),
     .exe_tag_i(id_tag),
     .exe_badvaddr_i_i(id_badvaddr_i),
     .exe_badvaddr_d_o(exe_badvaddr_d),
     .exe_badvaddr_i_o(exe_badvaddr_i)
);
/******************* end **********************/

/******************* int ***********************/
wire    [31:0] Cp0_data;
wire [31:0] mem_pc_o, mem_inst_o;
interupt_unit  in_unit(
    .clk(clk),
    .resetn(resetn),
    
    .datain(ex_rd2_o),
    .pc(ex_pc_o),
    .cp0_waddr({ex_inst_o[15:11], ex_inst_o[2:0]}),
    .cp0_raddr({mem_inst_o[15:11], mem_inst_o[2:0]}),
    .is_eret(is_eret),
    .is_mtc0(ex_wb_ctrl_o[6]),
    .is_delayslot(ex_tag[4]),
    .is_syscall(ex_wb_ctrl_o[8]),
    .is_break(ex_wb_ctrl_o[9]),
    .is_AdEL_i(ex_tag[6]),
    .is_AdEL_d(ex_tag[1]),
    .is_AdES(ex_tag[0]),
    .is_RI(ex_tag[5]),
    .is_Ov(ex_tag[2]),
    .int(int),
    .sr_exl(sr_exl),
    .sr_bev(sr_bev),
    .dataout(Cp0_data),
    .cp0_epc_o(cp0_epc),
    .badvaddr_i(exe_badvaddr_i),
    .badvaddr_d(exe_badvaddr_d),
    .sweap_o(sweap),
    .is_exception_o(is_exception),
    .ENTR(ENTR)
    );
/******************* end **********************/

/******************* Mem stage ***********************/
wire [31:0]  mem_wdata_o,rs_rdata_mo;
wire [9:0] mem_wb_ctrl_o;
wire is_mfhi,is_mflo;
wire [3:0] MemWrite;
wire mem_btaken_o;
wire    is_mfc0;

assign data_sram_wen = {MemWrite & {4{~ex_tag[0]}}};

MEM_stage MEM_stage (
	.clk		(clk		),
	.resetn		(resetn	& ~sweap	),

	.pc_i		(ex_pc_o	),
	.inst_i		(ex_inst_o	),
	.wdata_i	(ex_rd2_o	),
	.wb_ctrl_i	(ex_wb_ctrl_o	),
	.mem_ctrl_i	(ex_mem_ctrl_o	),
	.ALUOut_i	(ex_ALUOut_o	),
	.pc_o		(mem_pc_o	),
	.inst_o		(mem_inst_o	),
	.wdata_o	(data_sram_wdata	),
	.wb_ctrl_o	(mem_wb_ctrl_o	),
	.db_dest_i   (db_dest),

    	.is_mflo    (is_mflo),
    	.is_mfhi    (is_mfhi),
    	.is_mfc0(is_mfc0),
	.rdata_i	(data_sram_rdata),
	.MemRead_o	(Memread	),
	.MemWrite_o	(MemWrite	),
	.rdata_o    (mem_rdata_o   ),
	.ALUOut_o	(mem_ALUOut_o	),
	.db_dest_o   (wb_dest_i)
);
assign data_sram_en = 1'b1;
assign data_sram_addr = {{32{~(ex_tag[1]|ex_tag[0])}}&ex_ALUOut_o};	// for latency of sram
/**************** end *******************/

/*************** WB stage ***************/
wire [31:0] wb_pc_o, wb_inst_o,rs_rdata_wo;
wire [5:0] mul_reg_con;
//wire [7:0] wb_cp0_addr;
//wire [31:0] wb_cp0_wdata;
//wire [31:0] wb_epc;


reg	[31:0]	HI;
reg	[31:0]	LO;

assign  ALU_final = {{32{~is_mflo&~is_mfhi&~is_mfc0}}&mem_ALUOut_o}|{{32{is_mflo}}&LO}|{{32{is_mfhi}}&HI}|{{32{is_mfc0}}&Cp0_data};
WB_stage WB_stage (
	.clk		(clk		),
	.resetn		(resetn   	),
	.pc_i		(mem_pc_o	),
	.inst_i		(mem_inst_o	),
	.wb_ctrl_i	(mem_wb_ctrl_o	),
	.rdata_i	(mem_rdata_o	),	// ReadData from Mem
	.ALUOut_i	(ALU_final	),	// Result of ALU
	.pc_o		(wb_pc_o	),
	.inst_o		(wb_inst_o	),

	.db_dest_i  	(wb_dest_i	),

	.wd1_o		(wb_wd1_o	),
	.wa1_o		(wb_wa1_o	),
	.RegWrite_o	(wb_RegWrite_o	)
);

assign debug_wb_pc = wb_pc_o;
assign debug_wb_rf_wen = {4{wb_RegWrite_o}};
assign debug_wb_rf_wnum = wb_wa1_o;
assign debug_wb_rf_wdata = reg_wdata;

/*************** data forward unit ***************/
forward_unit forward_unit(
    .clk(clk),
    .rst(resetn ),
    .Asource({5{~ALUSrcA}}&ex_inst_o[25:21]),
    .Bsource(ex_inst_o[20:16]),
    .mem_dest({5{mem_wb_ctrl_o[0]}} & wb_dest_i),
    .mem_load(Memread),
    .wb_dest({5{wb_RegWrite_o}} & wb_wa1_o),
    
    .ForwardA(ForwardA),
    .ForwardB(ForwardB),
    .stop(stop)
);
/**************** end *******************/

/*************** muti&div unit ***************/

wire	[63:0]	mul_result;
wire	[31:0]	div_s;
wire	[31:0]	div_r;
wire    enable;
multi multi(
	.clk(clk),
	.resetn(resetn),
	.sig(mul_con_o[1]),
	.en(mul_con_o[0]),
	.S1(ex_rd1_o),
	.S2(ex_rd2_o),
	//input			sig,	//0 --> unsigned; 1 --> signed
	.C(mul_result),
	.enable(enable)
);

divider divider (
	.div_clk	(clk		),
	.resetn		(resetn		),
	.div		(div_con_o[0]	),
	.div_signed	(div_con_o[1]	),
	.x		(ex_rd1_o	),
	.y		(ex_rd2_o	),
	.s		(div_s		),
	.r		(div_r		),
	.complete	(div_complete	)
);

always @(posedge clk)
begin
     if(ex_mem_ctrl_o[14])
		HI <= ex_rd1_o;
	else if(enable)
		HI <= mul_result[63:32];
	else if(div_complete)
		HI <= div_r;
	else
		HI <= HI;
end

always @(posedge clk)
begin
     if(ex_mem_ctrl_o[13])
		LO <= ex_rd1_o;
	else if(enable)
		LO <= mul_result[31:0];
	else if(div_complete)
		LO <= div_s;
	else
		LO <= LO;
end

assign reg_wdata =  wb_wd1_o;
/**************** end *******************/
	

/*
cp0_reg cp0_reg(
	.clk		(clk		),
	.rst		(~resetn	),
	.waddr		(wb_cp0_waddr_o	),	//READ & WRITE addr of cp0 are the same
	.raddr		(wb_cp0_waddr_o	),
	.wen		(cp0_wen	),
	.wdata		(wb_cp0_wdata_o	),
	.rdata		(cp0_rdata	)
);
//we don;t really need so many cp0 reg now
*/



/**************** end *******************/

endmodule //mycpu_top
