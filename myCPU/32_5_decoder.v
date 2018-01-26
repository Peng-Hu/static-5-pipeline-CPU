`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/12/18 00:10:31
// Design Name: 
// Module Name: 32_3_decoder
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


module decoder32_5(
    input   [31:0]  S,
    output  [4:0]   R
    );
    assign  R[4] = S[31] | S[30] | S[29] | S[28] | S[27] | S[26] | S[25] | S[24] | S[23]
                    | S[22] | S[21] | S[20]| S[19] | S[18] | S[17] | S[16];
    assign  R[3] = S[31] | S[30] | S[29] | S[28] | S[27] | S[26] | S[25] | S[24] | S[15] | S[14]
                    | S[13] | S[12] | S[11] | S[10] | S[9] | S[8];
    assign  R[2] = S[31] | S[30] | S[29] | S[28] | S[23] | S[22] | S[21] | S[20] | S[15] | S[14]
                    |S[13] | S[12] | S[7] | S[6] | S[5] | S[4];
    assign  R[1] = S[31] | S[30] | S[27] | S[26] | S[23] | S[22] | S[19] | S[18] | S[15] | S[14]
                    | S[11] | S[10] | S[7] | S[6] | S[3] | S[2];
    assign  R[0] = S[31] | S[29] | S[27] | S[25] | S[23]| S[21] | S[19] | S[17] | S[15] | S[13]
                    | S[11] | S[9] | S[7] | S[5] | S[3] | S[1];
endmodule
