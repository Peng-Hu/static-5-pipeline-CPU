`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/12/17 09:35:23
// Design Name: 
// Module Name: MMU
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


module MMU  
#(
    parameter   integer TLB_size = 32
)
(
      input             clk,
      input             resetn,
//interface with IF
      input  wire [31:0] PC,
      input  wire        PC_valid_i,
      output wire        PC_ready_o,
//interface with axi     
      output wire        PC_valid_o,
      output wire [31:0]    irom_pc_o,
      input  wire        PC_ready_i,
//interface with MEM
      input  wire [70:0] MEM_data_pack_i,
      input  wire        MEM_data_valid_i,
      output wire        MEM_data_ready_o,
//interface with axi
      output  reg [70:0] MEM_data_pack_o,
      output  reg        MEM_data_valid_o,
      input wire        data_ready,
//interface with CP0
    input wire  [31:0]  index,
    input wire  [31:0]  entry_hi,
    input wire  [31:0]  entry_lo0,
    input wire  [31:0]  entry_lo1,
    input wire  [31:0]  mask,
    input wire             tlbw,

    output wire  [89:0]  tlb_entry,
    output wire  [31:0]   entry_index,
//exception
    output  reg    TLBS,
    output  reg    TLBL_D,
    output  wire    TLBL_I_o,
    output  reg    MOD,
    output  reg     [31:0]  badvaddr_d,
    output  reg    exception_sel,       //0 --> bfc00380 1 --> bfc00200
    input   wire    sweap
    );
    reg [89:0]  TLB [TLB_size-1 : 0];
    wire    [31:0]  hit;
    wire    [19:0]  PFN;
    wire    [19:0]  PFN_d;
    wire            valid0;
    wire            dirty0;
    wire            valid1;
    wire            dirty1;
    wire    [31:0]  v0,d0,v1,d1;
    wire    [19:0]  tmp_PFN     [31:0];
    wire    [19:0]  tmp_PFN_d   [31:0];
    wire    tlbl_d, tlbl_i, tlbs, mod;
    wire    data_wrong;
    reg    TLBL_I;
    assign  TLBL_I_o = TLBL_I | tlbl_i;
    assign  PC_ready_o = PC_ready_i && ~TLBL_I;
    assign  PC_valid_o = PC_valid_i && ~TLBL_I;
    assign  irom_pc_o = (PC[31]| (~PC[31] && TLBL_I_o)) ? PC : {PFN,PC[11:0]};
    assign  MEM_data_ready_o = data_ready && ~data_wrong;
    assign  tlb_entry = TLB[index[4:0]];
//data exception
    assign  tlbl_d = (~(|PFN_d) || ~valid1) && ~|MEM_data_pack_i[35:32] && MEM_data_valid_i && ~MEM_data_pack_i[67];
    assign  tlbs = (~(|PFN_d) || ~valid1) && |MEM_data_pack_i[35:32] && MEM_data_valid_i&& ~MEM_data_pack_i[67];
    assign  mod = ((|PFN_d) && valid1 && ~dirty1) && |MEM_data_pack_i[35:32] && MEM_data_valid_i&& ~MEM_data_pack_i[67];
    assign  data_wrong = tlbl_d | tlbs | mod;
    always @(posedge clk)
    begin
            if(~resetn | sweap)
                TLBL_D <= 1'b0;
            else if(tlbl_d)
                TLBL_D <= 1'b1;
            else
                TLBL_D <= TLBL_D;           
    end
    always @(posedge clk)
    begin
            if(~resetn | sweap)
                TLBS <= 1'b0;
            else if(tlbs)
                TLBS <= 1'b1;
            else
                TLBS <= TLBS;           
    end
    always @(posedge clk)
    begin
            if(~resetn | sweap)
                MOD <= 1'b0;
            else if(mod)
                MOD <= 1'b1;
            else
                MOD <= MOD;           
    end
    always @(posedge clk)
    begin
            if(~resetn | sweap)
                badvaddr_d <= 'd0;
            else if(data_wrong)
                badvaddr_d <= MEM_data_pack_i[67:36];
            else
                badvaddr_d <= badvaddr_d;           
    end
    always @(posedge clk)
    begin
            if(~resetn | sweap)
                exception_sel <= 1'b0;
            else if(~(|PFN_d) && MEM_data_valid_i&& ~MEM_data_pack_i[67])
                exception_sel <= 1'b1;
            else    if(~(|PFN)   && PC_valid_i&& ~PC[31])
                exception_sel <= 1'b1;
            else
                exception_sel <= exception_sel;           
    end
//PC exception
    assign  tlbl_i = (~(|PFN) || ~valid0)  && PC_valid_i && PC_ready_i && ~PC[31];
    always @(posedge clk)
    begin
            if(~resetn | sweap)
                TLBL_I <= 1'b0;
            else if(tlbl_i)
                TLBL_I <= 1'b1;
            else
                TLBL_I <= TLBL_I;           
    end
//signal for data
    always @(posedge clk)
    begin
        if(~resetn)
                    MEM_data_pack_o <= 'd0;
                else       
                    MEM_data_pack_o <= MEM_data_pack_i[67] ? MEM_data_pack_i : {MEM_data_pack_i[70:68],PFN_d,MEM_data_pack_i[47:0]};
    end
    
    always @(posedge clk)
    begin
            if(~resetn)
                MEM_data_valid_o <= 'd0;
            else if(data_ready && MEM_data_valid_o)
                    MEM_data_valid_o <= 1'b0;
            else if(MEM_data_valid_i && ~data_wrong)
                MEM_data_valid_o <= 1'b1;
            else
                MEM_data_valid_o <= MEM_data_valid_o;      
    end
//TLB
    always @(posedge clk)
    begin
            if(~resetn)                     TLB[0] <= 'd0;
            else if(tlbw && index[4:0] == 'd0)      TLB[0] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
            else                            TLB[0] <= TLB[0];    
    end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[1] <= 'd0;
                else if(tlbw && index[4:0] == 'd1)      TLB[1] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[1] <= TLB[1];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[2] <= 'd0;
                else if(tlbw && index[4:0] == 'd2)      TLB[2] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[2] <= TLB[2];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[3] <= 'd0;
                else if(tlbw && index[4:0] == 'd3)      TLB[3] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[3] <= TLB[3];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[4] <= 'd0;
                else if(tlbw && index[4:0] == 'd4)      TLB[4] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[4] <= TLB[4];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[5] <= 'd0;
                else if(tlbw && index[4:0] == 'd5)      TLB[5] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[5] <= TLB[5];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[6] <= 'd0;
                else if(tlbw && index[4:0] == 'd6)      TLB[6] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[6] <= TLB[6];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[7] <= 'd0;
                else if(tlbw && index[4:0] == 'd7)      TLB[7] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[7] <= TLB[7];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[8] <= 'd0;
                else if(tlbw && index[4:0] == 'd8)      TLB[8] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[8] <= TLB[8];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[9] <= 'd0;
                else if(tlbw && index[4:0] == 'd9)      TLB[9] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[9] <= TLB[9];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[10] <= 'd0;
                else if(tlbw && index[4:0] == 'd10)      TLB[10] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[10] <= TLB[10];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[11] <= 'd0;
                else if(tlbw && index[4:0] == 'd11)      TLB[11] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[11] <= TLB[11];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[12] <= 'd0;
                else if(tlbw && index[4:0] == 'd12)      TLB[12] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[12] <= TLB[12];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[13] <= 'd0;
                else if(tlbw && index[4:0] == 'd13)      TLB[13] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[13] <= TLB[13];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[14] <= 'd0;
                else if(tlbw && index[4:0] == 'd14)      TLB[14] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[14] <= TLB[14];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[15] <= 'd0;
                else if(tlbw && index[4:0] == 'd15)      TLB[15] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[15] <= TLB[15];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[16] <= 'd0;
                else if(tlbw && index[4:0] == 'd16)      TLB[16] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[16] <= TLB[16];    
        end
    always @(posedge clk)
        begin
                if(~resetn)                     TLB[17] <= 'd0;
                else if(tlbw && index[4:0] == 'd17)      TLB[17] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                else                            TLB[17] <= TLB[17];    
        end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[18] <= 'd0;
                    else if(tlbw && index[4:0] == 'd18)      TLB[18] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[18] <= TLB[18];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[19] <= 'd0;
                    else if(tlbw && index[4:0] == 'd19)      TLB[19] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[19] <= TLB[19];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[20] <= 'd0;
                    else if(tlbw && index[4:0] == 'd20)      TLB[20] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[20] <= TLB[20];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[21] <= 'd0;
                    else if(tlbw && index[4:0] == 'd21)      TLB[21] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[21] <= TLB[21];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[22] <= 'd0;
                    else if(tlbw && index[4:0] == 'd22)      TLB[22] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[22] <= TLB[22];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[23] <= 'd0;
                    else if(tlbw && index[4:0] == 'd23)      TLB[23] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[23] <= TLB[23];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[24] <= 'd0;
                    else if(tlbw && index[4:0] == 'd24)      TLB[24] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[24] <= TLB[24];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[25] <= 'd0;
                    else if(tlbw && index[4:0] == 'd25)      TLB[25] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[25] <= TLB[25];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[26] <= 'd0;
                    else if(tlbw && index[4:0] == 'd26)      TLB[26] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[26] <= TLB[26];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[27] <= 'd0;
                    else if(tlbw && index[4:0] == 'd27)      TLB[27] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[27] <= TLB[27];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[28] <= 'd0;
                    else if(tlbw && index[4:0] == 'd28)      TLB[28] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[28] <= TLB[28];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[29] <= 'd0;
                    else if(tlbw && index[4:0] == 'd29)      TLB[29] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[29] <= TLB[29];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[30] <= 'd0;
                    else if(tlbw && index[4:0] == 'd30)      TLB[30] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[30] <= TLB[30];    
            end
        always @(posedge clk)
            begin
                    if(~resetn)                     TLB[31] <= 'd0;
                    else if(tlbw && index[4:0] == 'd31)      TLB[31] <= {entry_hi[31:13],entry_hi[7:0],mask[24:13],entry_lo0[0]&entry_lo1[0],entry_lo0[25:1],entry_lo1[25:1]};
                    else                            TLB[31] <= TLB[31];    
            end
    genvar gen;
    generate
    for(gen = 0; gen < TLB_size; gen = gen + 1)
    begin
            assign hit[gen] = (~|({TLB[gen][89:83],TLB[gen][82:71]&(~TLB[gen][62:51])}^{entry_hi[31:25],entry_hi[24:13]&(~TLB[gen][62:51])})) & (TLB[gen][50] | (~|(TLB[gen][70:63]^entry_hi[7:0])));
    end
    endgenerate
    assign  entry_index[31] = ~|hit;
    assign  entry_index[30:5] = 'd0;
    
    decoder32_5 decoder32_5(
        .S      (hit),
        .R      (entry_index[4:0])
    );
    
     generate
       for(gen = 0; gen < TLB_size; gen = gen + 1)
       begin
               assign tmp_PFN[gen] = ((~|({TLB[gen][89:83],TLB[gen][82:71]&(~TLB[gen][62:51])}^{PC[31:25],PC[24:13]&(~TLB[gen][62:51])})) & (TLB[gen][50] | (~|(TLB[gen][70:63]^entry_hi[7:0])))) ? 
                                        (PC[12] ? TLB[gen][24:5] : TLB[gen][49:30]) : 'd0;
               assign   v0[gen] = ((~|({TLB[gen][89:83],TLB[gen][82:71]&(~TLB[gen][62:51])}^{PC[31:25],PC[24:13]&(~TLB[gen][62:51])})) & (TLB[gen][50] | (~|(TLB[gen][70:63]^entry_hi[7:0])))) ? 
                                        (PC[12] ? TLB[gen][0] : TLB[gen][25]) : 1'b0;
               assign   d0[gen] = ((~|({TLB[gen][89:83],TLB[gen][82:71]&(~TLB[gen][62:51])}^{PC[31:25],PC[24:13]&(~TLB[gen][62:51])})) & (TLB[gen][50] | (~|(TLB[gen][70:63]^entry_hi[7:0])))) ? 
                                        (PC[12] ? TLB[gen][1] : TLB[gen][26]) : 1'b0;
       end
       endgenerate
    assign  PFN = tmp_PFN[31] | tmp_PFN[29] | tmp_PFN[28] | tmp_PFN[27] | tmp_PFN[26] | tmp_PFN[25] | tmp_PFN[24] | tmp_PFN[23] | tmp_PFN[22] | tmp_PFN[21] | tmp_PFN[20] | tmp_PFN[19]
                    | tmp_PFN[18] | tmp_PFN[17] | tmp_PFN[16] | tmp_PFN[15] | tmp_PFN[14] | tmp_PFN[13] | tmp_PFN[12] | tmp_PFN[11] | tmp_PFN[10] | tmp_PFN[9] | tmp_PFN[8] | tmp_PFN[7]
                    | tmp_PFN[6] | tmp_PFN[5] | tmp_PFN[4] | tmp_PFN[3] | tmp_PFN[2] | tmp_PFN[1] | tmp_PFN[0];
    assign  valid0 = |v0;
    assign  dirty0 = |d0;
    
     generate
       for(gen = 0; gen < TLB_size; gen = gen + 1)
       begin
               assign tmp_PFN_d[gen] = ((~|({TLB[gen][89:83],TLB[gen][82:71]&(~TLB[gen][62:51])}^{MEM_data_pack_i[67:61],MEM_data_pack_i[60:49]&(~TLB[gen][62:51])})) & (TLB[gen][50] | (~|(TLB[gen][70:63]^entry_hi[7:0])))) ? 
                                          (MEM_data_pack_i[48] ? TLB[gen][24:5] : TLB[gen][49:30]) : 'd0;
               assign   v1[gen] = ((~|({TLB[gen][89:83],TLB[gen][82:71]&(~TLB[gen][62:51])}^{MEM_data_pack_i[67:61],MEM_data_pack_i[60:49]&(~TLB[gen][62:51])})) & (TLB[gen][50] | (~|(TLB[gen][70:63]^entry_hi[7:0])))) ? 
                                          (MEM_data_pack_i[48] ? TLB[gen][0] : TLB[gen][25]) : 1'b0;
               assign   d1[gen] = ((~|({TLB[gen][89:83],TLB[gen][82:71]&(~TLB[gen][62:51])}^{MEM_data_pack_i[67:61],MEM_data_pack_i[60:49]&(~TLB[gen][62:51])})) & (TLB[gen][50] | (~|(TLB[gen][70:63]^entry_hi[7:0])))) ? 
                                          (MEM_data_pack_i[48] ? TLB[gen][1] : TLB[gen][26]) : 1'b0;
       end
       endgenerate
    assign  PFN_d = tmp_PFN_d[31] | tmp_PFN_d[29] | tmp_PFN_d[28] | tmp_PFN_d[27] | tmp_PFN_d[26] | tmp_PFN_d[25] | tmp_PFN_d[24] | tmp_PFN_d[23] | tmp_PFN_d[22] | tmp_PFN_d[21] | tmp_PFN_d[20] | tmp_PFN_d[19]
                    | tmp_PFN_d[18] | tmp_PFN_d[17] | tmp_PFN_d[16] | tmp_PFN_d[15] | tmp_PFN_d[14] | tmp_PFN_d[13] | tmp_PFN_d[12] | tmp_PFN_d[11] | tmp_PFN_d[10] | tmp_PFN_d[9] | tmp_PFN_d[8] | tmp_PFN_d[7]
                    | tmp_PFN_d[6] | tmp_PFN_d[5] | tmp_PFN_d[4] | tmp_PFN_d[3] | tmp_PFN_d[2] | tmp_PFN_d[1] | tmp_PFN_d[0];
    assign  valid1 = |v1;
    assign  dirty1 = |d1;
endmodule
