`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/11/17 09:59:53
// Design Name: 
// Module Name: interupt_unit
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module interupt_unit(
    input clk,
    input   resetn,
    
    input [31:0] datain,
    input   [31:0]  pc,
    input   [7:0]   cp0_waddr,
    input   [7:0]   cp0_raddr,
    input   is_eret,
    input   is_mtc0,
    input   is_delayslot,
    input   is_syscall,
    input   is_break,
    input   is_AdEL_i,
    input   is_AdEL_d,
    input   is_AdES,
    input   is_RI,
    input   is_Ov,
    input   is_TLBL_i,
    input   is_TLBL_d,
    input   is_TLBS,
    input   is_MOD,
    input   [7:0]   int,
    input   [31:0]  badvaddr_i,
    input   [31:0]  badvaddr_d,
    input   is_tlbr,
    input   is_tlbwi,
    input   is_tlbp,
    
    output  sr_exl,
    output  sr_bev,
    output [31:0] dataout,
    output  [31:0]  cp0_epc_o,
    output   sweap_o,
    output  is_exception_o,
    output  wire    ENTR,
/**************** MMU *******************/
    output  reg [31:0]  index,
    output  reg [31:0]  entry_hi,
    output  reg [31:0]  entry_lo0,
    output  reg [31:0]  entry_lo1,
    output  reg [31:0]  mask,
    output              tlbw,
    
    input   [89:0]  tlb_entry,
    input   [31:0]   entry_index
    );

    /**************** CP0 *******************/
    reg [31:0] cp0_epc;
    reg [31:0] cp0_status, cp0_cause;
    reg [31:0]  cp0_count,cp0_compare;
    reg         cp0_count_step;
    reg [31:0]  cp0_BadVaddr;
    reg [31:0]        cp0_status_k;
   // wire [31:0] cp0_wdata;
    wire    is_AdEL;
    wire [31:0] new_status, new_cause, new_epc,new_hi,new_lo0,new_lo1,new_mask,new_index;
    wire status_wen, cause_wen, epc_wen,hi_wen,lo1_wen,lo0_wen,mask_wen,index_wen;
    wire t_status, t_cause, t_epc,t_count,t_compare,t_hi,t_lo0,t_lo1,t_mask,t_index;
    wire f_status, f_cause, f_epc,f_count,f_compare,f_badvaddr,f_hi,f_lo0,f_lo1,f_mask,f_index;
    wire    [31:0]  int_cause;
    wire    [31:0]  exception_cause;
    wire    [31:0]  w_epc;
    wire    [31:0]  badvaddr;
    wire    is_clock;
    wire    INTR,HW0,HW1,HW2,HW3,HW4,HW5,SW0,SW1;
    wire    [4:0]   Exccode;
    wire    is_exception;
    wire    pc_valid;
    //wire    sweap;
    assign  pc_valid = |pc;
    assign  is_exception_o = is_AdEL|is_AdES|is_RI|is_Ov|is_TLBL_i | is_TLBS | is_MOD | is_TLBL_d;
    assign  is_AdEL = is_AdEL_i | is_AdEL_d;
    assign  Exccode = ENTR ? 5'd0 :(is_AdEL ? 5'd4: (is_TLBL_i ? 5'd2 : (is_AdES ? 5'd5:(is_RI ? 5'ha: (is_Ov ? 5'hc: (is_syscall ? 5'd8:(is_break ? 5'd9:(is_TLBL_d ? 5'd2 : (is_TLBS ? 5'd3 : (is_MOD ? 5'd1 : 5'd0))))))))));
    assign  sweap_o = is_AdEL|is_AdES|is_RI|is_Ov|ENTR|is_TLBL_i | is_TLBS | is_MOD | is_TLBL_d;
    //interrupt
    assign  is_clock = (|(cp0_compare)) & (~(|(cp0_compare ^ cp0_count)));
    assign  HW0 =   cp0_status[0]&~cp0_status[1]&cp0_status[10]&cp0_cause[10]&pc_valid;
    assign  HW1 =   cp0_status[0]&~cp0_status[1]&cp0_status[11]&cp0_cause[11]&pc_valid;
    assign  HW2 =   cp0_status[0]&~cp0_status[1]&cp0_status[12]&cp0_cause[12]&pc_valid;
    assign  HW3 =   cp0_status[0]&~cp0_status[1]&cp0_status[13]&cp0_cause[13]&pc_valid;
    assign  HW4 =   cp0_status[0]&~cp0_status[1]&cp0_status[14]&cp0_cause[14]&pc_valid;
    assign  HW5 =   cp0_status[0]&~cp0_status[1]&cp0_status[15]&cp0_cause[15]&pc_valid;
    assign  SW0 =   cp0_status[0]&~cp0_status[1]&cp0_status[8] & cp0_cause[8]&pc_valid;
    assign  SW1 =   cp0_status[0]&~cp0_status[1]&cp0_status[9] & cp0_cause[9]&pc_valid;
    assign  INTR = (|int)|is_clock;
    assign  ENTR = HW0|HW1|HW2|HW3|HW4|HW5|SW0|SW1;
    assign  int_cause = ENTR ? {is_delayslot,is_clock,15'd0,HW5,HW4,HW3,HW2,HW1,HW0,SW1,SW0,1'b0,Exccode,2'b0} 
                                : {1'b0,is_clock,15'd0,(int[7]|is_clock),int[6],int[5],int[4],int[3],int[2],int[1],int[0],8'd0};
    //system call                                                                 
    assign exception_cause = {is_delayslot,24'd0,Exccode,2'd0};
    assign  w_epc = is_delayslot ? pc-4 : pc;
    //exception
    assign  is_exception = (is_break|is_syscall|is_AdEL|is_AdES|is_RI|is_Ov|is_TLBL_i | is_TLBS | is_MOD | is_TLBL_d);
    //special instruction
    assign t_status = ~|(cp0_waddr ^ 8'b01100000);
    assign t_cause = ~|(cp0_waddr ^ 8'b01101000);
    assign t_epc = ~|(cp0_waddr ^ 8'b01110000);
    assign  t_count = ~|(cp0_waddr ^ 8'b01001000);
    assign  t_compare =  ~|(cp0_waddr ^ 8'b01011000);
    assign  t_hi = ~|(cp0_waddr ^ 8'b01010000);
    assign  t_lo0 = ~|(cp0_waddr ^ 8'b00010000);
    assign  t_lo1 = ~|(cp0_waddr ^ 8'b00011000);
    assign  t_mask = ~|(cp0_waddr ^ 8'b00101000);
    assign  t_index = (~|(cp0_waddr ^ 8'b00000000)) & pc_valid;
    assign  f_hi = ~|(cp0_raddr ^ 8'b01010000);
    assign  f_lo0 = ~|(cp0_raddr ^8'b00010000);
    assign  f_lo1 = ~|(cp0_raddr ^ 8'b00011000);
    assign  f_mask = ~|(cp0_raddr ^ 8'b00101000);
    assign  f_index = (~|(cp0_raddr ^ 8'b00000000)) & pc_valid;
    assign  f_count =  ~|(cp0_raddr ^ 8'b01001000);
    assign  f_compare =  ~|(cp0_raddr ^ 8'b01011000);
    assign f_status = ~|(cp0_raddr ^ 8'b01100000);
    assign f_cause = ~|(cp0_raddr ^ 8'b01101000);
    assign f_epc = ~|(cp0_raddr ^ 8'b01110000);
    assign  f_badvaddr = ~|(cp0_raddr ^ 8'b01000000);
    
    assign  tlbw = is_tlbwi;
    assign sr_exl = cp0_status[1];
    assign sr_bev = cp0_status[22];
    assign status_wen = (is_mtc0 & t_status) | is_exception | is_eret | ENTR;
    //assign status_wen = 1'b1;    //每个周期都要更新status？
    assign cause_wen = (is_mtc0 & t_cause) | is_eret | is_exception | INTR | ENTR| (t_compare & is_mtc0);
    assign epc_wen = (is_mtc0 & t_epc) | is_exception | ENTR;    //这里注意，是此周期写入的exl
    assign  hi_wen = (is_mtc0 & t_hi) | is_tlbr;
    assign  lo1_wen = (is_mtc0 & t_lo1) | is_tlbr;
    assign  lo0_wen = (is_mtc0 & t_lo0) | is_tlbr;
    assign  mask_wen = (is_mtc0 & t_mask) | is_tlbr;
    assign  index_wen = (is_mtc0 & t_index)  | is_tlbp;
    assign  badvaddr = (is_AdEL_i|is_TLBL_i) ? badvaddr_i : ((is_AdEL_d|is_AdES|is_TLBS | is_MOD | is_TLBL_d) ? badvaddr_d : 'd0);
    
    assign  new_hi = ({32{is_mtc0 & t_hi}} & {datain[31:13],5'd0,datain[7:0]}) | ({32{is_tlbr}} & {{tlb_entry[89:83],tlb_entry[82:71]&(~tlb_entry[62:51])},5'd0,tlb_entry[70:63]}) 
                        | ({32{is_TLBS | is_MOD | is_TLBL_d}} & {badvaddr_d[31:12],entry_hi[11:0]}) | ({32{is_TLBL_i}} & {badvaddr_i[31:12],entry_hi[11:0]});
    assign  new_lo0 = ({32{is_mtc0 & t_lo0}} & {6'd0,datain[25:0]}) | ({32{is_tlbr}} & {6'd0,{tlb_entry[49:42],tlb_entry[41:30]&(~tlb_entry[62:51])},tlb_entry[29:25],tlb_entry[50]});
    assign  new_lo1 = ({32{is_mtc0 & t_lo1}} & {6'd0,datain[25:0]}) | ({32{is_tlbr}} & {6'd0,{tlb_entry[24:17],tlb_entry[16:5]&(~tlb_entry[62:51])},tlb_entry[4:0],tlb_entry[50]});
    assign  new_mask = ({32{is_mtc0 & t_mask}} & {7'd0,datain[24:13],13'd0}) | ({32{is_tlbr}} & {7'd0,tlb_entry[62:51],13'd0});
    assign  new_index = ({32{is_mtc0 & t_index}} & {index[31],26'd0,datain[4:0]}) | ({32{is_tlbp}} & {entry_index[31],26'd0,entry_index[4:0]});
    assign new_status = ({32{is_mtc0 & t_status}} & {cp0_status[31:16],datain[15:8],cp0_status[7:2],datain[1:0]}) | ({32{is_exception|ENTR}}&{cp0_status[31:2],1'b1,cp0_status[0]})|({32{is_eret}} & cp0_status_k);
    assign new_cause = (INTR|ENTR) ? int_cause : (is_exception ? exception_cause : 'd0);
    assign new_epc = ({32{is_mtc0 & t_epc}} & datain) | ({32{is_exception | ENTR}} & w_epc);
    
    assign  cp0_epc_o = cp0_epc;
    assign dataout = f_status ? cp0_status : 
                                (f_cause ? cp0_cause : 
                                            (f_epc ? cp0_epc : 
                                                    (f_count ? cp0_count:
                                                                (f_compare ? cp0_compare: 
                                                                                (f_badvaddr ? cp0_BadVaddr:
                                                                                                (f_hi ? entry_hi :
                                                                                                        (f_lo0 ? entry_lo0 :
                                                                                                                    (f_lo1 ? entry_lo1:
                                                                                                                                (f_mask ? mask : 
                                                                                                                                            (f_index ? index : 'd0)) )))))))); 
    
    always @(posedge clk)
    begin
        if(~resetn)
            cp0_status_k <= 'd0;
        else if(is_exception  | ENTR)
            cp0_status_k <= cp0_status;
        else
            cp0_status_k <= cp0_status_k; 
    end
    
    always @(posedge clk)
    begin
        if(~resetn)
            cp0_status <= 32'h00400000;
        else if(status_wen)
            cp0_status <= new_status;
        else
            cp0_status <= cp0_status;
    end
    
    always @(posedge clk)
        begin
            if(~resetn)
                cp0_BadVaddr <= 'd0;
            else if(is_AdEL|is_AdES|is_TLBL_i | is_TLBS | is_MOD | is_TLBL_d)
                cp0_BadVaddr <= badvaddr;
            else
                cp0_BadVaddr <= cp0_BadVaddr;
        end
        
    always @(posedge clk)
    begin
        if(~resetn)
            cp0_cause <= 'd0;
        else if(cause_wen)
            cp0_cause <= new_cause;
        else
            cp0_cause <= cp0_cause;
    end
    
    always @(posedge clk)
    begin
        if(~resetn)
            cp0_epc <= 'd0;
        else if(epc_wen)
            cp0_epc <= new_epc;
        else
            cp0_epc <= cp0_epc;
    end
    
    always @(posedge clk)
    begin
        if(~resetn)
            cp0_count_step <= 1'b0;
        else if(~cp0_count_step)
            cp0_count_step <= 1'b1;
        else if(cp0_count_step)
            cp0_count_step <= 1'b0;
        else
            cp0_count_step <= cp0_count_step;
    end
    
    always @(posedge clk)
        begin
            if(~resetn)
                cp0_count <= 'd0;
            else if(t_count & is_mtc0)
                cp0_count <= datain;
            else if(cp0_count_step)
                cp0_count <= cp0_count + 'd1;
            else
                cp0_count <= cp0_count;
        end
    
    always@(posedge clk)
    begin
        if(~resetn)
            cp0_compare <= 'd0;
        else if(t_compare & is_mtc0)
            cp0_compare <= datain;
        else
            cp0_compare <= cp0_compare;
    end
    always @(posedge clk)
    begin
        if(~resetn)
            entry_hi <= 'd0;
        else if(hi_wen|is_TLBL_i | is_TLBS | is_MOD | is_TLBL_d)
            entry_hi <= new_hi;
        else
            entry_hi <= entry_hi;
    end
    always @(posedge clk)
    begin
        if(~resetn)
            entry_lo0 <= 'd0;
        else if(lo0_wen)
            entry_lo0 <= new_lo0;
        else
            entry_lo0 <= entry_lo0;
    end
    always @(posedge clk)
    begin
        if(~resetn)
            entry_lo1 <= 'd0;
        else if(lo1_wen)
            entry_lo1 <= new_lo1;
        else
            entry_lo1 <= entry_lo1;
    end
    always @(posedge clk)
    begin
        if(~resetn)
            mask <= 'd0;
        else if(mask_wen)
            mask <= new_mask;
        else
            mask <= mask;
    end
    always @(posedge clk)
    begin
        if(~resetn)
            index <= 'd0;
        else if(index_wen)
            index <= new_index;
        else
            index <= index;
    end
endmodule
