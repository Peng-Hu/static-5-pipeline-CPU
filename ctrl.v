`timescale 1ns/1ns
`include"define.h"

module ctrl (
    input wire         clk,
    input wire resetn,
	input wire [5:0]	opcode,
	input wire [5:0]	funcode,
	input wire [4:0]	rs,
	input wire [4:0]	rt,
	input wire [31:0] 	rdata1,
	input wire [31:0] 	rdata2,

	output wire [9:0]	wb_ctrl,
	output wire [18:0]	mem_ctrl,
	output wire [12:0]	exe_ctrl,
	output wire 		ExtOp,
	output wire    		is_b,
	output wire    		token,
	output wire		b_hit,
	output wire		j_hit,
	output wire		jr_hit,
	/*input to b forward unit*/
	input wire		complete,
    input   wire    [4:0]   	ex_db_dest,
    input   wire                ex_memread,
    input   wire    [4:0]   	mem_db_dest,
    input   wire    [31:0]      mem_wdata,
    input   wire            	memread,
    input   wire    [4:0]   	wb_dest,
    input   wire    [31:0]      wb_wdata,
    output  reg             	b_stop,
    output  reg     [1:0]   	forward_rs,
    output  reg     [1:0]   	forward_rt,
    output  wire		is_eret,
    output  wire    is_RI,
    output  wire    test_Ov,
    input   wire    sweap
);

/*decode start*/
wire is_rtype;
wire is_addi;
wire is_addiu;
wire is_slti;
wire is_sltiu;
wire is_andi;
wire is_lui;
wire is_ori;
wire is_xori;
wire is_beq;
wire is_bne;
wire is_bgez;///
wire is_bgtz;///
wire is_blez;///
wire is_bz;
wire is_j;
wire is_jal;
wire is_lb;
wire is_lbu;
wire is_lh;
wire is_lhu;
wire is_lw;
wire is_lwl;
wire is_lwr;
wire is_sb;
wire is_sh;
wire is_sw;
wire is_swl;
wire is_swr;
wire is_spec;

assign is_rtype	= (opcode == `RTYPE);
assign is_addi	= (opcode == `ADDI);
assign is_addiu	= (opcode == `ADDIU);
assign is_slti	= (opcode == `SLTI);
assign is_sltiu	= (opcode == `SLTIU);
assign is_andi	= (opcode == `ANDI);
assign is_lui	= (opcode == `LUI);
assign is_ori	= (opcode == `ORI);
assign is_xori	= (opcode == `XORI);
assign is_beq	= (opcode == `BEQ);
assign is_bne	= (opcode == `BNE);

assign is_bgtz	= (opcode == `BGTZ);
assign is_blez	= (opcode == `BLEZ);
assign is_bz    = (opcode == `BZ);
assign is_j	= (opcode == `J);
assign is_jal	= (opcode == `JAL);
assign is_lb	= (opcode == `LB);
assign is_lbu	= (opcode == `LBU);
assign is_lh	= (opcode == `LH);
assign is_lhu	= (opcode == `LHU);
assign is_lw	= (opcode == `LW);
assign is_lwl	= (opcode == `LWL);
assign is_lwr	= (opcode == `LWR);
assign is_sb	= (opcode == `SB);
assign is_sh	= (opcode == `SH);
assign is_sw	= (opcode == `SW);
assign is_swl	= (opcode == `SWL);
assign is_swr	= (opcode == `SWR);
assign is_spec	= (opcode == `SPEC);
/****** Function Code ******/
wire is_add;
wire is_addu;
wire is_sub;
wire is_subu;
wire is_slt;
wire is_sltu;
wire is_div;
wire is_divu;
wire is_mult;//
wire is_multu;//
wire is_and;
wire is_nor;
wire is_or;
wire is_xor;
wire is_sllv;
wire is_sll;
wire is_srav;
wire is_sra;
wire is_srlv;
wire is_srl;
wire is_jr;
wire is_jalr;///
wire is_mfhi;//
wire is_mflo;//	rd <-- lo
wire is_mthi;//
wire is_mtlo;// lo <-- rs
wire is_break;
wire is_syscall;
wire is_mfc0;
wire is_mtc0; 
wire is_bltz;///
wire is_bgezal;///
wire is_bltzal;//
wire is_itype;
wire is_link;
wire is_store;
wire is_load;
wire is_lhw;

assign is_add	= is_rtype & (funcode == `ADD);
assign is_addu	= is_rtype & (funcode == `ADDU);
assign is_sub	= is_rtype & (funcode == `SUB);
assign is_subu	= is_rtype & (funcode == `SUBU);
assign is_slt	= is_rtype & (funcode == `SLT);
assign is_sltu	= is_rtype & (funcode == `SLTU);
assign is_div	= is_rtype & (funcode == `DIV);
assign is_divu	= is_rtype & (funcode == `DIVU);
assign is_mult	= is_rtype & (funcode == `MULT);
assign is_multu	= is_rtype & (funcode == `MULTU);
assign is_and	= is_rtype & (funcode == `AND);
assign is_nor	= is_rtype & (funcode == `NOR);
assign is_or	= is_rtype & (funcode == `OR);
assign is_xor	= is_rtype & (funcode == `XOR);
assign is_sllv	= is_rtype & (funcode == `SLLV);
assign is_sll	= is_rtype & (funcode == `SLL);
assign is_srav	= is_rtype & (funcode == `SRAV);
assign is_sra	= is_rtype & (funcode == `SRA);
assign is_srlv	= is_rtype & (funcode == `SRLV);
assign is_srl	= is_rtype & (funcode == `SRL);
assign is_jr	= is_rtype & (funcode == `JR);
assign is_jalr	= is_rtype & (funcode == `JALR);
assign is_mfhi	= is_rtype & (funcode == `MFHI);
assign is_mflo	= is_rtype & (funcode == `MFLO);
assign is_mthi	= is_rtype & (funcode == `MTHI);
assign is_mtlo	= is_rtype & (funcode == `MTLO);
assign is_break	= is_rtype & (funcode == `BREAK);
assign is_syscall	= is_rtype & (funcode == `SYSCALL);

assign is_eret	= is_spec & (funcode == `ERET);
assign is_mfc0	= is_spec & (rs == `MFC0);
assign is_mtc0	= is_spec & (rs == `MTC0); 
assign is_bgez	= is_bz &(rt == `BGEZ);
assign is_bltz	= is_bz & (rt == `BLTZ);
assign is_bgezal	= is_bz & (rt == `BGEZAL);
assign is_bltzal	= is_bz & (rt == `BLTZAL);

assign is_itype	= is_addi | is_addiu | is_slti | is_sltiu | is_andi | is_lui | is_ori | is_xori | is_sllv | is_sll | is_srav | is_srlv | is_srl;
assign is_link	= is_bgezal | is_bltzal | is_jal;
assign is_store	= is_sb | is_sh | is_sw | is_swl | is_swr;
assign is_load	= is_lb | is_lbu | is_lh | is_lhu | is_lw | is_lwl | is_lwr;
assign  is_lhw = is_lh | is_lhu;
/*decode end*/

/*generating ctrl signals begin*/
/*
RegDst  00=rt	01=rd 	10=31 	11=0
Mem2Reg 0=Mrd 	1=ALUOut
ALUSrcA 0=rd1 	1=PC
ALUSrcB 00=rd2	01=EI	10=8	11=0
ExtOp	0=0	1=sign
*/
wire [1:0]	RegDst;
wire [1:0]	ALUSrcB;
wire [5:0]	ALUop;
wire		Mem2Reg, RegWrite, ALUSrcA,HI_write,LO_write,sig,mul_en, div_signed, div;
wire [4:0]	MemRead, MemWrite;
wire		MemUnsign;
reg	div_valid;

assign wb_ctrl = b_stop ? 'd0 : {is_break,is_syscall, is_mfc0, is_mtc0, div,is_multu|is_mult,RegDst, Mem2Reg, RegWrite};
assign mem_ctrl = b_stop ? 'd0 : {is_lw,is_lhw,is_sw,is_sh,HI_write,LO_write,is_mfhi,is_mflo,MemUnsign, MemRead, MemWrite};
assign exe_ctrl = b_stop ? 'd0 : {div_signed, div, sig,mul_en,ALUop, ALUSrcA, ALUSrcB};

assign ALUop = ({6{is_rtype}} & funcode) | ({6{is_itype}} & opcode) | ({6{is_link | is_store | is_load}} & `ADD);
assign ALUSrcA = is_link | is_jalr;
assign ALUSrcB[0] = (is_itype & ~is_sll & ~is_sllv & ~is_srav & ~is_sra & ~is_srl & ~is_srlv) | (|MemRead) | (|MemWrite);
assign ALUSrcB[1] = is_link | is_jalr;

assign MemRead = {is_lw, is_lwl, is_lwr, (is_lhu | is_lh), (is_lb | is_lbu)};
assign MemWrite = {is_sw, is_swl, is_swr, is_sh, is_sb};
assign MemUnsign = is_lbu | is_lhu;

assign RegDst[0] = is_rtype | is_jalr |is_mflo | is_mfhi;
assign RegDst[1] = is_link;
assign Mem2Reg = is_rtype | is_itype | is_link |is_mfhi|is_mflo|is_mfc0;
assign RegWrite = (is_rtype & ~is_jr) | is_itype | is_link | is_load | is_mfc0 | is_mflo | is_mfhi;

assign ExtOp = ~(is_andi | is_ori | is_xori);

assign	HI_write =  is_mthi;
assign	LO_write = is_mtlo;
assign	sig = is_mult;
assign	mul_en = is_mult | is_multu;
assign	div_signed = is_div;
assign	div = is_div | is_divu;

always @(posedge clk)
begin
	if(~resetn) div_valid <= 1'b1;
	else if(div & ~sweap) div_valid <= 1'b0;
	else if(complete) div_valid <= 1'b1;
	else div_valid <= div_valid;
end

assign  is_RI = ~(is_rtype|is_itype|is_link|is_store|is_load|is_b|j_hit|jr_hit | is_spec);
assign  test_Ov = is_add | is_sub| is_addi;
/*generating ctrl signals end*/

/*Branch Judge start*/
wire equal, rs_ls, rs_nz;
assign equal = (rdata1 == rdata2);
assign rs_ls = rdata1[31];	//rs<0
assign rs_nz = |rdata1;	//rs!=0


assign j_hit = is_j | is_jal;
assign jr_hit = is_jr | is_jalr;
assign  is_b = is_beq|is_bne|is_bgez|is_bgezal|is_bgtz|is_blez|is_bltz|is_bltzal;
/*Branch Judge end*/
/*b data forward unit*/
always @(*)
//always @(rs or rt or ex_db_dest or  mem_db_dest)
begin
        if(sweap)
            b_stop = 1'b0;
        else if(jr_hit )
        begin
                if((rs == ex_db_dest) && ex_db_dest != 'd0)
                        b_stop = 1'b1;
                else if((rs == mem_db_dest) && mem_db_dest != 'd0 && memread == 1'b1)
                        b_stop = 1'b1;
                else 
                        b_stop = 1'b0;
        end
        else if(is_b )
        begin
                if((rs == ex_db_dest) && ex_db_dest != 'd0 && ex_memread == 1'b1)
                         b_stop = 1'b1;
                else if((((is_beq == 1'b1||is_bne == 1'b1) && rt == ex_db_dest)) && ex_db_dest != 'd0 && ex_memread == 1'b1)
                        b_stop = 1'b1;
                else 
                        b_stop = 1'b0;
        end
	else if ((is_mfhi | is_mflo | is_mthi | is_mtlo) )
	begin
		b_stop = ~div_valid;
	end
        else
            b_stop = 1'b0;
end

always @(*)
begin
        if(is_b == 1'b1 || jr_hit == 1'b1)
        begin
                if(rs == mem_db_dest && mem_db_dest != 'd0 && memread == 1'b0)
                        forward_rs = 'd1;
                else if(rs == wb_dest && wb_dest != 'd0)
                        forward_rs = 'd2;
                else
                        forward_rs = 'd0;
        end
        else
            forward_rs = 'd0;
end

always @(*)
begin
        if(is_beq == 1'b1 || is_bne == 1'b1 || jr_hit == 1'b1)
        begin
                if(rt == mem_db_dest && mem_db_dest != 'd0 && memread == 1'b0)
                        forward_rt = 'd1;
                else if(rt == wb_dest && wb_dest != 'd0)
                        forward_rt = 'd2;
                else
                        forward_rt = 'd0;
        end
        else
            forward_rt = 'd0;
end

/*for inst buf*/
reg is_b_exe1_reg;
wire is_b_exe1;
reg is_b_m1_reg;
wire is_b_m1;
reg is_b_exe2_reg;
wire is_b_exe2;
reg is_b_m2_reg;
wire is_b_m2;
reg [31:0] rdata1_hold;
reg [31:0] rdata2_hold;
reg     is_beq_hold,is_bne_hold,is_bge_hold,is_bgtz_hold,is_blez_hold,is_blt_hold;
wire    equal_hold, rs_ls_hold, rs_nz_hold;

assign b_hit = (~is_b_exe1&~is_b_m1&~is_b_exe2&~is_b_m2) & (/*rs=rt*/(equal & is_beq) | /*rs!=rt*/(~equal & is_bne) | /*rs>=0*/(~rs_ls & (is_bgez | is_bgezal)) 
						| /*rs>0*/(~rs_ls & rs_nz & is_bgtz) | /*rs<=0*/((rs_ls | ~rs_nz) & is_blez) | /*rs<0*/(rs_ls & (is_bltz | is_bltzal)));
						
assign is_b_exe1 = (rs == ex_db_dest  ) && ex_db_dest != 'd0;
assign is_b_m1 = (rs == mem_db_dest ) && mem_db_dest != 'd0 && memread == 1'b1;
assign is_b_exe2 = (((is_beq == 1'b1||is_bne == 1'b1) && rt == ex_db_dest)) && ex_db_dest != 'd0;
assign is_b_m2 = (((is_beq == 1'b1||is_bne == 1'b1) && rt == mem_db_dest)) && mem_db_dest != 'd0 && memread == 1'b1;
always@(posedge clk)
begin
    if(~resetn || sweap)
        begin
            rdata1_hold <= 'd0;
            rdata2_hold <= 'd0;
        end
    else
        begin
            rdata1_hold <= rdata1;
            rdata2_hold <= rdata2;
        end
end

always @(posedge clk)
//always @(rs or rt or ex_db_dest or  mem_db_dest)
begin
        if(is_b)
        begin
                if(is_b_exe1)
                        is_b_exe1_reg <= 1'b1;
                else
                        is_b_exe1_reg <= 1'b0;
        end
        else
            is_b_exe1_reg <= 1'b0;
end
always @(posedge clk)
begin
        if(is_b)
        begin
                if(is_b_exe2)
                        is_b_exe2_reg <= 1'b1;
                else
                        is_b_exe2_reg <= 1'b0;
        end
        else
                    is_b_exe2_reg <= 1'b0;
end
always @(posedge clk)
//always @(rs or rt or ex_db_dest or  mem_db_dest)
begin
        if(is_b)
        begin

                if(is_b_m1)
                        is_b_m1_reg <= 1'b1;
                else 
                        is_b_m1_reg <= 1'b0;
        end
        else
                    is_b_m1_reg <= 1'b0;
end
always @(posedge clk)
begin
        if(is_b)
        begin

                if(is_b_m2)
                        is_b_m2_reg <= 1'b1;
                else 
                        is_b_m2_reg <= 1'b0;
        end
        else
                    is_b_m2_reg <= 1'b0;
end
always @(posedge clk)
begin
    if(~resetn || b_stop || sweap)
    begin
            is_beq_hold <= 1'b0;
            is_bne_hold <= 1'b0;
            is_bge_hold <= 1'b0;
            is_bgtz_hold <= 1'b0;
            is_blez_hold <= 1'b0;
            is_blt_hold <= 1'b0;
    end
    else if(is_b)
    begin
            is_beq_hold <= (is_b_exe1|is_b_exe2|is_b_m2|is_b_m1)&is_beq;
            is_bne_hold <= (is_b_exe1|is_b_exe2|is_b_m2|is_b_m1)&is_bne;
            is_bge_hold <= (is_b_m1|is_b_exe1)&(is_bgez | is_bgezal);
            is_bgtz_hold <= (is_b_m1|is_b_exe1)&is_bgtz;
            is_blez_hold <= (is_b_m1|is_b_exe1)&is_blez;
            is_blt_hold <= (is_b_m1|is_b_exe1)&(is_bltz | is_bltzal);
    end
    else
    begin
            is_beq_hold <= 1'b0;
            is_bne_hold <= 1'b0;
            is_bge_hold <= 1'b0;
            is_bgtz_hold <= 1'b0;
            is_blez_hold <= 1'b0;
            is_blt_hold <= 1'b0;
    end
end

assign equal_hold = is_b_exe1_reg ? (is_b_m2_reg ? (mem_wdata == wb_wdata):(mem_wdata == rdata2_hold)) : (is_b_exe2_reg ? (is_b_m1_reg ? (wb_wdata == mem_wdata):(rdata1_hold == mem_wdata)) : (is_b_m1_reg ? (wb_wdata == rdata2_hold) : (is_b_m2_reg ? (rdata1_hold == wb_wdata) : 1'b0)));
assign  rs_ls_hold = is_b_exe1_reg ? mem_wdata[31] : (is_b_m1_reg ? wb_wdata[31] : 1'b0);
assign  rs_nz_hold = is_b_exe1_reg ? |mem_wdata : (is_b_m1_reg ? |wb_wdata : 1'b0);
assign token =  (/*rs=rt*/(equal_hold & is_beq_hold) | /*rs!=rt*/(~equal_hold & is_bne_hold) | /*rs>=0*/(~rs_ls_hold & is_bge_hold) 
						| /*rs>0*/(~rs_ls_hold & rs_nz_hold & is_bgtz_hold) | /*rs<=0*/((rs_ls_hold | ~rs_nz_hold) & is_blez_hold) | /*rs<0*/(rs_ls_hold & is_blt_hold));
endmodule
