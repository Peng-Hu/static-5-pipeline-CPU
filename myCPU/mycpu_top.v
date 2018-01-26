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

    output wire [31:0]      M_AXI_AWADDR,
    output wire [2:0] M_AXI_AWSIZE,
    output wire M_AXI_AWVALID,
    input  wire M_AXI_AWREADY,

    output wire [31:0]      M_AXI_WDATA,
    output wire [3:0]    M_AXI_WSTRB,
    output wire M_AXI_WVALID,
    input  wire M_AXI_WREADY,

    input  wire M_AXI_BVALID,
    output wire M_AXI_BREADY,

    output wire [3:0] M_AXI_ARID,
    output wire [31:0]      M_AXI_ARADDR,
    output wire [2:0] M_AXI_ARSIZE,
    output wire M_AXI_ARVALID,
    input  wire M_AXI_ARREADY,

    input wire [3:0] M_AXI_RID,
    input  wire [31:0]      M_AXI_RDATA,
    input  wire M_AXI_RVALID,
    output wire M_AXI_RREADY,

    output wire [31:0] debug_wb_pc,
    output wire [ 3:0] debug_wb_rf_wen,
    output wire [ 4:0] debug_wb_rf_wnum,
    output wire [31:0] debug_wb_rf_wdata,
    
    input   wire    [7:0]   int
);
// we only need an inst ROM now
wire    axi_PCvalid;
wire    axi_PCready;
wire    [31:0]  axi_PC;
wire    [63:0]  inst_sram_rdata;

wire    inst_valid;
wire    inst_ready;
wire    mem_stop;
wire [70:0] axi_data_pack;
wire        axi_data_valid;
wire        axi_data_ready;
wire [31:0] data_sram_rdata;
wire        MEM_rvalid;
wire		is_int;
wire    sweap;
wire        token;
wire		id_wb_PCWC_o;
wire    weap;
wire        is_b;
wire    TLBL_I;
wire    [7:0]   if_tag;
/****************** axi_slate *****************/
axi_master 
#(
    .C_M_AXI_DATA_WIDTH     (32)
)axi1
(
    .M_AXI_ACLK(clk),
    .M_AXI_ARESETN(resetn),

    .M_AXI_AWADDR(M_AXI_AWADDR),
    .M_AXI_AWSIZE(M_AXI_AWSIZE),
    .M_AXI_AWVALID(M_AXI_AWVALID),
    .M_AXI_AWREADY(M_AXI_AWREADY),

    .M_AXI_WDATA(M_AXI_WDATA),
    .M_AXI_WSTRB(M_AXI_WSTRB),
    .M_AXI_WVALID(M_AXI_WVALID),
    .M_AXI_WREADY(M_AXI_WREADY),

    .M_AXI_BVALID(M_AXI_BVALID),
    .M_AXI_BREADY(M_AXI_BREADY),

    .M_AXI_ARID(M_AXI_ARID),
    .M_AXI_ARADDR(M_AXI_ARADDR),
    .M_AXI_ARSIZE(M_AXI_ARSIZE),
    .M_AXI_ARVALID(M_AXI_ARVALID),
    .M_AXI_ARREADY(M_AXI_ARREADY),

    .M_AXI_RID(M_AXI_RID),
    .M_AXI_RDATA(M_AXI_RDATA),
    .M_AXI_RVALID(M_AXI_RVALID),
    .M_AXI_RREADY(M_AXI_RREADY),

    .PC(axi_PC),
    .PC_valid(axi_PCvalid),
    .ins_ready(inst_ready),
    .PC_ready(axi_PCready),
    .ins_back_pack(inst_sram_rdata),
    .ins_valid(inst_valid),
    .TLBL       (TLBL_I),
    .TLBL_i     (if_tag[7]),

    .weap(sweap | weap),
    .token_i(sweap |  id_wb_PCWC_o | token),
    .j_b(id_wb_PCWC_o | is_b),
    .is_int(is_int),

    .MEM_data_pack(axi_data_pack),
    .MEM_data_valid(axi_data_valid),
    .MEM_data_ready(axi_data_ready),
    .MEM_rdata(data_sram_rdata),
    .MEM_rvalid(MEM_rvalid)
);
/****************** IMMU *****************/
wire    PC_ready;
wire    PC_valid;
wire [31:0]	if_irom_pc_o;
wire [70:0] MEM_data_pack;
wire        MEM_data_valid;
wire        MEM_data_ready;
wire    [31:0]  CP0_index;
wire    [31:0]  CP0_mask;
wire    [31:0]  CP0_entryhi;
wire    [31:0]  CP0_entrylo0;
wire    [31:0]  CP0_entrylo1;
wire            tlbw;
wire    [89:0]  tlb_entry;
wire    [31:0]  entry_index;
wire    TLBS;
wire    TLBL_D;
wire    MOD;
wire    [31:0]  badvaddr_d;
wire    exception_sel;
MMU MMU(
        .clk                (clk),
        .resetn             (resetn),
//interface with IF
        .PC                 (if_irom_pc_o),
        .PC_valid_i         (PC_valid),
        .PC_ready_o         (PC_ready),
//interface with axi     
        .PC_valid_o         (axi_PCvalid),
        .irom_pc_o          (axi_PC),
        .PC_ready_i         (axi_PCready),
//interface with MEM
        .MEM_data_pack_i    (MEM_data_pack),
        .MEM_data_valid_i   (MEM_data_valid),
        .MEM_data_ready_o   (MEM_data_ready),
//interface with axi
        .MEM_data_pack_o    (axi_data_pack),
        .MEM_data_valid_o   (axi_data_valid),
        .data_ready         (axi_data_ready),
//interface with CP0
        .index              (CP0_index),
        .entry_hi           (CP0_entryhi),
        .entry_lo0          (CP0_entrylo0),
        .entry_lo1          (CP0_entrylo1),
        .mask               (CP0_mask),
        .tlbw               (tlbw),
        
        .tlb_entry          (tlb_entry),
        .entry_index        (entry_index),
 //exception
        .TLBS               (TLBS),
        .TLBL_D             (TLBL_D),
        .TLBL_I_o             (TLBL_I),
        .MOD                (MOD),
        .badvaddr_d         (badvaddr_d),
        .exception_sel      (exception_sel),       //0 --> bfc00380 1 --> bfc00200
        .sweap              (sweap)
);

/****************** IF stage *****************/

wire [31:0]	if_pc_o;
wire [31:0]	if_pc_4_o;
wire [31:0]	if_inst_o;
wire [31:0] id_wb_pc_o;
wire        b_stop;
wire [31:0] eret_pc;
wire [31:0] int_pc;
wire sr_exl, sr_bev;
wire is_eret;
wire [31:0] cp0_epc;
wire    syscall;
wire    break;
wire [12:0] id_wb_ctrl_o;
wire    is_adel;

wire    [31:0]  if_badvaddr;
wire    is_exception;
wire    ENTR;
wire    stop;

IF_stage IF_stage (
	.clk		(clk		),
	.resetn		(resetn		),

	.irom_inst_i       (inst_sram_rdata[31:0]),
	.irom_pc_i         (inst_sram_rdata[63:32]),
	.inst_valid        (inst_valid),
	.PC_ready          (PC_ready),
	.inst_ready        (inst_ready),
	.PC_valid          (PC_valid),
	.irom_pc_o         (if_irom_pc_o),
	
	.pc_o		(if_pc_o	),
	.pc_4_o		(if_pc_4_o	),
	.inst_o		(if_inst_o	),
	.wb_pc		(id_wb_pc_o	),
	.PCWriteCond	(id_wb_PCWC_o	),
	.stop          (stop|b_stop|mem_stop),
	.token         (token),
	.is_int		(is_int),
	.int_pc		(int_pc),
	.is_b(is_b)
);
assign  syscall = id_wb_ctrl_o[8];
assign  break = id_wb_ctrl_o[9];
assign is_int = is_eret | syscall | break |is_exception | ENTR;
assign eret_pc = cp0_epc;
assign int_pc = ({32{is_eret}} & cp0_epc) | ({32{((~sr_exl & ENTR) | (is_exception&~exception_sel) | syscall | break) & sr_bev}} & 32'hbfc00380) | ({32{(is_exception&exception_sel)}} & 32'hbfc00200);
assign  is_adel = (~(is_eret | syscall | break)) & (if_pc_o[1] | if_pc_o[0]);
assign  if_badvaddr =   {32{is_adel | if_tag[7]}} & if_pc_o;
assign  if_tag[6:0] = {is_adel,6'd0};
// 读数据需要一个周期，最好别用PC reg里的PC
/********************* end ***********************/

/******************* ID stage ********************/
wire [31:0] id_pc_o, id_inst_o;
wire [4:0] wb_wa1_o;
wire	[31:0]	reg_wdata;
wire [31:0] wb_wd1_o;
wire wb_RegWrite_o;
wire [31:0] id_rd1_o, id_rd2_o;
wire [18:0] id_mem_ctrl_o;
wire [12:0] id_exe_ctrl_o;
wire [31:0] id_extend_o;
wire	div_complete;
wire [31:0] ALU_final;
wire    [7:0] id_tag;
wire   [31:0] id_badvaddr_i;
wire [4:0]  db_dest;
wire Memread;
wire [4:0]  wb_dest_i;
ID_stage ID_stage (
    .clk            (clk),
    .resetn         (resetn),
    /*IF-ID Reg*/
    .pc_i   		({{32{ ~is_int  & ~weap}} & if_pc_o}	),
    .pc_4_i 		({{32{  ~is_int & ~weap}} &if_pc_4_o}	),
    .inst_i 		({{32{  ~is_int & ~weap}} &if_inst_o}	),
    .pc_valid_i       (inst_valid),
	.pc_o		    (id_pc_o	),
	.inst_o		      (id_inst_o	),
	.b_stop         (b_stop    ),
	.is_b           (is_b),
	.token          (token),
	.weap              (weap),
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
    .stop           (stop|mem_stop),
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
//内部包含寄存器堆，控制单元，分支选择单元
/******************* end *********************/

/****************** EX stage ********************/
wire [31:0]	ex_pc_o, ex_inst_o, ex_rd2_o,ex_rd1_o;
wire [12:0]	ex_wb_ctrl_o;
wire [16:0] 	ex_mem_ctrl_o;
wire [31:0]	ex_ALUOut_o;
wire [1:0]	mul_con_o;
wire [1:0]	div_con_o;
wire [1:0] ALUSrcB;
wire        ALUSrcA;
wire    [7:0] ex_tag;
wire    [31:0]  exe_badvaddr_i;
wire    [31:0]  exe_badvaddr_d;
wire    [1:0]   ForwardA;
wire    [1:0]   ForwardB;
wire            is_read;
wire            is_write;
EX_stage EX_stage (
	.clk		(clk		),
	.resetn		(resetn	& ~sweap	),

	.pc_i		(id_pc_o	),	//PC
	.inst_i		(id_inst_o	),
	.rd1_i	(id_rd1_o	),
	.rd2_i	(id_rd2_o	),
	.extend_i	(id_extend_o	),
	.wb_ctrl_i	(id_wb_ctrl_o	),
	.mem_ctrl_i	((id_mem_ctrl_o	& {19{~(id_tag[7] | id_tag[6])}})),
	.exe_ctrl_i	((id_exe_ctrl_o & {13{~(id_tag[7] | id_tag[6])}})	),

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
     .stop(stop|mem_stop),
     .ForwardA(ForwardA),
     .ForwardB(ForwardB),
     .exe_tag_o(ex_tag),
     .exe_tag_i(id_tag),
     .exe_badvaddr_i_i(id_badvaddr_i),
     .exe_badvaddr_d_o(exe_badvaddr_d),
     .exe_badvaddr_i_o(exe_badvaddr_i),
     .is_read           (is_read),
     .is_write          (is_write)
);
/******************* end **********************/

/******************* int ***********************/
wire    [31:0] Cp0_data;
wire [31:0] mem_pc_o, mem_inst_o,mem_badvaddr_d,mem_badvaddr_i,data_sram_wdata,mem_rd2;
interupt_unit  in_unit(
    .clk(clk),
    .resetn(resetn),
    
    .datain(mem_rd2),
    .pc(mem_pc_o),
    .cp0_waddr({mem_inst_o[15:11], mem_inst_o[2:0]}),
    .cp0_raddr({mem_inst_o[15:11], mem_inst_o[2:0]}),
    .is_eret(is_eret),
    .is_mtc0(mem_wb_ctrl_o[6]),
    .is_delayslot(mem_tag[4]),
    .is_syscall(mem_wb_ctrl_o[8]),
    .is_break(mem_wb_ctrl_o[9]),
    .is_AdEL_i(mem_tag[6]),
    .is_AdEL_d(mem_tag[1]),
    .is_AdES(mem_tag[0]),
    .is_RI(mem_tag[5]),
    .is_Ov(mem_tag[2]),
    .is_tlbr    (mem_wb_ctrl_o[10]),
    .is_tlbwi   (mem_wb_ctrl_o[11]),
    .is_tlbp    (mem_wb_ctrl_o[12]),
    .is_TLBL_i  (mem_tag[7]),   
    .is_TLBL_d  (TLBL_D),
    .is_TLBS    (TLBS),
    .is_MOD     (MOD),
    .int(int),
    .sr_exl(sr_exl),
    .sr_bev(sr_bev),
    .dataout(Cp0_data),
    .cp0_epc_o(cp0_epc),
    .badvaddr_i(mem_badvaddr_i),
    .badvaddr_d(mem_badvaddr_d | badvaddr_d),
    .sweap_o(sweap),
    .is_exception_o(is_exception),
    .ENTR(ENTR),
    
    .index      (CP0_index),
    .entry_hi   (CP0_entryhi),
    .entry_lo0  (CP0_entrylo0),
    .entry_lo1  (CP0_entrylo1),
    .mask       (CP0_mask),
    .tlbw       (tlbw),
    
    .tlb_entry  (tlb_entry),
    .entry_index(entry_index)
    );
/******************* end **********************/

/******************* Mem stage ***********************/
wire [12:0] mem_wb_ctrl_o;
wire is_mfhi,is_mflo;
wire [3:0] MemWrite;
wire    is_mfc0;
wire    [7:0]   mem_tag;
wire [31:0] mem_rdata_o, mem_ALUOut_o,data_sram_addr;
wire    [3:0]   data_sram_wen;
assign data_sram_wen = {MemWrite & {4{~mem_tag[0]}}};
assign  MEM_data_pack = {3'b010,data_sram_addr,data_sram_wen,data_sram_wdata};
MEM_stage MEM_stage (
	.clk		(clk		),
	.resetn		(resetn	& ~sweap	),
    
    .is_write   (is_write),
    .is_read    (is_read),
	.pc_i		(ex_pc_o	),
	.inst_i		(ex_inst_o	),
	.wdata_i	(ex_rd2_o	),
	.wb_ctrl_i	(ex_wb_ctrl_o	),
	.mem_ctrl_i	(ex_mem_ctrl_o	),
	.ALUOut_i	(ex_ALUOut_o	),
	.mem_tag_i     (ex_tag),
	.mem_tag_o     (mem_tag),
	.pc_o		(mem_pc_o	),
	.inst_o		(mem_inst_o	),
	.wdata_o	(data_sram_wdata	),
	.wb_ctrl_o	(mem_wb_ctrl_o	),
	.db_dest_i   (db_dest),
	.badvaddr_i_i  (exe_badvaddr_i),
    .badvaddr_d_i   (exe_badvaddr_d),
    .badvaddr_i_o   (mem_badvaddr_i),
    .badvaddr_d_o   (mem_badvaddr_d),

    	.is_mflo    (is_mflo),
    	.is_mfhi    (is_mfhi),
    	.is_mfc0(is_mfc0),
	.rdata_i	(data_sram_rdata),
	.MemRead_o	(Memread	),
	.MemWrite_o	(MemWrite	),
	.data_ready(MEM_data_ready),
	.mem_data_valid    (MEM_data_valid),
	.rdata_valid(MEM_rvalid),
	.rdata_o    (mem_rdata_o   ),
	.ALUOut_o	(mem_ALUOut_o	),
	.db_dest_o   (wb_dest_i),
	.mem_stop_o     (mem_stop),
	.rd2_o     (mem_rd2)
);
assign data_sram_addr = {{32{~(mem_tag[1]|mem_tag[0])}}&mem_ALUOut_o};	// for latency of sram
/**************** end *******************/

/*************** WB stage ***************/
wire [31:0] wb_pc_o, wb_inst_o;
reg	[31:0]	HI;
reg	[31:0]	LO;

assign  ALU_final = {{32{~is_mflo&~is_mfhi&~is_mfc0}}&mem_ALUOut_o}|{{32{is_mflo}}&LO}|{{32{is_mfhi}}&HI}|{{32{is_mfc0}}&Cp0_data};
WB_stage WB_stage (
	.clk		(clk		),
	.resetn		(resetn  & ~sweap 	),
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
	.resetn(resetn ),
	.sig(mul_con_o[1]& ~sweap),
	.en(mul_con_o[0]& ~sweap),
	.S1(ex_rd1_o),
	.S2(ex_rd2_o),
	//input			sig,	//0 --> unsigned; 1 --> signed
	.C(mul_result),
	.enable(enable)
);

divider divider (
	.div_clk	(clk		),
	.resetn		(resetn		),
	.div		(div_con_o[0] & ~sweap	),
	.div_signed	(div_con_o[1] & ~sweap	),
	.x		(ex_rd1_o	),
	.y		(ex_rd2_o	),
	.s		(div_s		),
	.r		(div_r		),
	.complete	(div_complete	)
);

always @(posedge clk)
begin
     if(ex_mem_ctrl_o[14] & ~sweap)
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
     if(ex_mem_ctrl_o[13] & ~sweap)
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
endmodule //mycpu_top