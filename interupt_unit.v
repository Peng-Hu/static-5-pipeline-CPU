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
    input   [7:0]   int,
    input   [31:0]  badvaddr_i,
    input   [31:0]  badvaddr_d,
    
    output  sr_exl,
    output  sr_bev,
    output [31:0] dataout,
    output  [31:0]  cp0_epc_o,
    output   sweap_o,
    output  is_exception_o,
    output  wire    ENTR
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
    wire [31:0] new_status, new_cause, new_epc;
    wire status_wen, cause_wen, epc_wen;
    wire t_status, t_cause, t_epc,t_count,t_compare;
    wire f_status, f_cause, f_epc,f_count,f_compare,f_badvaddr;
    wire [31:0] syscall_cause;
    wire    [31:0]  int_cause;
    wire    [31:0]  break_cause;
    wire    [31:0]  AdEL_cause;
    wire    [31:0]  AdES_cause;
    wire    [31:0]  RI_cause;
    wire    [31:0]  Ov_cause;
    wire    [31:0]  w_epc;
    wire    [31:0]  badvaddr;
    wire    is_clock;
    wire    INTR,HW0,HW1,HW2,HW3,HW4,HW5,SW0,SW1;
    wire    [4:0]   Exccode;
    wire    is_exception;
    //wire    sweap;
    
    assign  is_exception_o = is_AdEL|is_AdES|is_RI|is_Ov;
    assign  is_AdEL = is_AdEL_i | is_AdEL_d;
    assign  Exccode = ENTR ? 5'd0 :(is_AdEL ? 5'd4: (is_AdES ? 5'd5:(is_RI ? 5'ha: (is_Ov ? 5'hc: (is_syscall ? 5'd8:(is_break ? 5'd9:5'd0))))));
    assign  sweap_o = is_AdEL|is_AdES|is_RI|is_Ov|ENTR;
    //interrupt
    assign  is_clock = (|(cp0_compare)) & (~(|(cp0_compare ^ cp0_count)));
    assign  HW0 =   cp0_status[0]&~cp0_status[1]&cp0_status[10]&cp0_cause[10];
    assign  HW1 =   cp0_status[0]&~cp0_status[1]&cp0_status[11]&cp0_cause[11];
    assign  HW2 =   cp0_status[0]&~cp0_status[1]&cp0_status[12]&cp0_cause[12];
    assign  HW3 =   cp0_status[0]&~cp0_status[1]&cp0_status[13]&cp0_cause[13];
    assign  HW4 =   cp0_status[0]&~cp0_status[1]&cp0_status[14]&cp0_cause[14];
    assign  HW5 =   cp0_status[0]&~cp0_status[1]&cp0_status[15]&cp0_cause[15];
    assign  SW0 =   cp0_status[0]&~cp0_status[1]&cp0_status[8] & cp0_cause[8];
    assign  SW1 =   cp0_status[0]&~cp0_status[1]&cp0_status[9] & cp0_cause[9];
    assign  INTR = (|int)|is_clock;
    assign  ENTR = HW0|HW1|HW2|HW3|HW4|HW5|SW0|SW1;
    assign  int_cause = ENTR ? {is_delayslot,is_clock,15'd0,HW5,HW4,HW3,HW2,HW1,HW0,SW1,SW0,1'b0,Exccode,2'b0} 
                                : {1'b0,is_clock,15'd0,(int[7]|is_clock),int[6],int[5],int[4],int[3],int[2],int[1],int[0],8'd0};
    //system call                                                                 
    assign syscall_cause = {is_delayslot,24'd0,Exccode,2'd0};
    assign  w_epc = is_delayslot ? pc-4 : pc;
    assign  break_cause = {is_delayslot,24'd0,Exccode,2'd0};
    //exception
    assign  is_exception = (is_break|is_syscall|is_AdEL|is_AdES|is_RI|is_Ov);
    assign  AdEL_cause = {is_delayslot,24'd0,Exccode,2'd0};
    assign  AdES_cause = {is_delayslot,24'd0,Exccode,2'd0};
    assign  RI_cause = {is_delayslot,24'd0,Exccode,2'd0};
    assign  Ov_cause = {is_delayslot,24'd0,Exccode,2'd0};
    //special instruction
    assign t_status = ~|(cp0_waddr ^ 8'b01100000);
    assign t_cause = ~|(cp0_waddr ^ 8'b01101000);
    assign t_epc = ~|(cp0_waddr ^ 8'b01110000);
    assign  t_count = ~|(cp0_waddr ^ 8'b01001000);
    assign  t_compare =  ~|(cp0_waddr ^ 8'b01011000);
    assign  f_count =  ~|(cp0_waddr ^ 8'b01001000);
    assign  f_compare =  ~|(cp0_waddr ^ 8'b01011000);
    assign f_status = ~|(cp0_raddr ^ 8'b01100000);
    assign f_cause = ~|(cp0_raddr ^ 8'b01101000);
    assign f_epc = ~|(cp0_raddr ^ 8'b01110000);
    assign  f_badvaddr = ~|(cp0_raddr ^ 8'b01000000);
    
    assign sr_exl = cp0_status[1];
    assign sr_bev = cp0_status[22];
    assign status_wen = (is_mtc0 & t_status) | is_exception | is_eret | ENTR;
    //assign status_wen = 1'b1;    //每个周期都要更新status？
    assign cause_wen = (is_mtc0 & t_cause) | is_eret | is_exception | INTR | ENTR| (t_compare & is_mtc0);
    assign epc_wen = (is_mtc0 & t_epc) | is_exception | ENTR;    //这里注意，是此周期写入的exl
    assign  badvaddr = is_AdEL_i ? badvaddr_i : ((is_AdEL_d|is_AdES) ? badvaddr_d : 'd0);
    assign new_status = ({32{is_mtc0 & t_status}} & {cp0_status[31:16],datain[15:8],cp0_status[7:2],datain[1:0]}) | ({32{is_exception|ENTR}}&{cp0_status[31:2],1'b1,cp0_status[0]})|({32{is_eret}} & cp0_status_k);
    assign new_cause = (INTR|ENTR) ? int_cause : 
                            (is_exception ? (is_AdEL_i ? AdEL_cause:(is_RI ? RI_cause:(is_syscall ? syscall_cause:(is_break ? break_cause: (is_Ov ? Ov_cause:  (is_AdEL_d ? AdEL_cause : AdES_cause)))))):
                                            ({32{is_mtc0 & t_cause}} & datain) | ({32{is_eret}} & 32'd0) | ({32{is_mtc0 & t_compare}} & {cp0_cause[31],1'b0,cp0_cause[29:16],1'b0,cp0_cause[14:0]}));
    assign new_epc = ({32{is_mtc0 & t_epc}} & datain) | ({32{is_exception | ENTR}} & w_epc);
    
    assign  cp0_epc_o = cp0_epc;
    assign dataout = f_status ? cp0_status : (f_cause ? cp0_cause : (f_epc ? cp0_epc : (f_count ? cp0_count:(f_compare ? cp0_compare: (f_badvaddr ? cp0_BadVaddr:'d0))))); 
    
   /* always @(posedge clk)
    begin
        if(~resetn)
            sweap_o <= 1'b0;
        else
            sweap_o <= sweap;
    end*/
    
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
            else if(is_AdEL|is_AdES)
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
endmodule
