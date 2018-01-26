
/****** Op Code ******/
`define RTYPE	6'b000000
`define ADDI	6'b001000
`define ADDIU	6'b001001
`define SLTI	6'b001010
`define SLTIU	6'b001011
`define ANDI	6'b001100
`define LUI	6'b001111
`define ORI	6'b001101
`define XORI	6'b001110
`define BEQ	6'b000100
`define BNE	6'b000101

`define BGTZ	6'b000111
`define BLEZ	6'b000110
`define BZ	6'b000001	// BGEZAL, BLTZAL
`define J	6'b000010
`define JAL	6'b000011
`define LB	6'b100000
`define LBU	6'b100100
`define LH	6'b100001
`define LHU	6'b100101
`define LW	6'b100011
`define LWL	6'b100010
`define LWR	6'b100110
`define SB	6'b101000
`define SH	6'b101001
`define SW	6'b101011
`define SWL	6'b101010
`define SWR	6'b101110
`define SPEC	6'b010000	// MFC0, MFT0

/****** Function Code ******/
`define	ADD	6'b100000 
`define ADDU	6'b100001
`define SUB	6'b100010
`define SUBU	6'b100011
`define SLT	6'b101010
`define SLTU	6'b101011
`define DIV	6'b011010
`define DIVU	6'b011011
`define MULT	6'b011000
`define MULTU	6'b011001
`define AND	6'b100100
`define NOR	6'b100111
`define OR	6'b100101
`define XOR	6'b100110
`define SLLV	6'b000100
`define SLL	6'b000000
`define SRAV	6'b000111
`define SRA	6'b000011
`define SRLV	6'b000110
`define SRL	6'b000010
`define JR	6'b001000
`define JALR	6'b001001
`define MFHI	6'b010000
`define MFLO	6'b010010
`define MTHI	6'b010001
`define MTLO	6'b010011
`define BREAK	6'b001101
`define SYSCALL	6'b001100
`define ERET	6'b011000

//BLTZ code
`define BLTZ	5'b00000
`define BGEZAL	5'b10001
`define BLTZAL	5'b10000
`define BGEZ	6'b00001
//SPEC rs code
`define MFC0	5'b00000
`define MTC0	5'b00100

