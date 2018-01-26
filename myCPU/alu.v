`timescale 1ns/1ns
`include"define.h"

module alu(
	input wire [5 :0]	ALUop,	// function code
	input wire [31:0]	A,
	input wire [31:0]	B,
	input wire [4 :0]	shamt,
	output reg [31:0]	result,
	output wire        is_Ov
);
    wire [31:0] C;
    wire [32:0] F;
    wire [32:0] H;
    wire [31:0] sr1;
    wire [31:0] sr2;
    wire Overflow;
    wire CarryOut;
    reg cin;
    assign sr1 = {32{B[31]}} << (32 - shamt);
    assign sr2 = {32{B[31]}} << (32 - A[4:0]);
    assign  C = (~B);
    assign H = (cin) ? ({1'b0,A}+{1'b0,C}+cin) : ({1'b0,A}+{1'b0,B});           //??????carryout??
    assign F = (cin) ? ({A[31],A}+{C[31],C}+cin) : ({A[31],A}+{B[31],B});
    assign Overflow = F[32]^F[31];
    assign CarryOut = cin^H[32];
    assign  is_Ov = ~((|(ALUop ^ `ADD))&(|(ALUop ^ `ADDI))&(|(ALUop ^ `SUB))) & Overflow;
	always @(*)
	begin
		casex(ALUop)
			`ADD,`ADDI,`ADDU,`ADDIU,`SUB,`SUBU:
				result = F[31:0];
			`AND,`ANDI:
				result = A & B;
			`OR,`ORI:
				result = A | B;
			`XOR,`XORI:
				result = A ^ B;
			`NOR:
				result = ~(A|B);
			`SLT,`SLTI:
				result = {{31{1'b0}},F[32]};
			`SLL:
				result = B << shamt;
			`SRL:
				result = B >> shamt;
			`SLLV:
				result = B << A[4:0];
			`SRLV:
				result = B >> A[4:0];
			`SLTU,`SLTIU:
				result = {{31{1'b0}},CarryOut};   
			`LUI:
				result = B << 16;
			`SRA:
			    result = sr1 | (B >> shamt);
			`SRAV:
			     result = sr2 | (B >> A[4:0]);
			default:
				result = 32'b0;
		endcase
		
		casex(ALUop)
		`SUB,`SUBU,`SLT,`SLTI,`SLTU,`SLTIU:
		      cin = 1'b1;
		 default:
		      cin = 1'b0;
		 endcase
	end
endmodule
