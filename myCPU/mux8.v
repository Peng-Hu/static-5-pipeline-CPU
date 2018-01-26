`timescale 1ns/1ns

module mux8(mux8_out,m0_in,m1_in,m2_in,m3_in,m4_in,m5_in,m6_in,m7_in,sel_in);
   parameter width = 32;
     
   output[width-1:0] mux8_out;//数据输出端口
   input [width-1:0] m0_in;   //数据输入端口0，sel_in='b000
   input [width-1:0] m1_in;   //数据输入端口1，sel_in='b001
   input [width-1:0] m2_in;   //数据输入端口2，sel_in='b010
   input [width-1:0] m3_in;   //数据输入端口3，sel_in='b011
   input [width-1:0] m4_in;   //数据输入端口0，sel_in='b100
   input [width-1:0] m5_in;   //数据输入端口1，sel_in='b101
   input [width-1:0] m6_in;   //数据输入端口2，sel_in='b110
   input [width-1:0] m7_in;   //数据输入端口3，sel_in='b111
   
	input [2:0]       sel_in;  //选择控制输入
   
   reg   [width-1:0] mux8_out;

   always @(*)
      begin
	case(sel_in)
	'b000:mux8_out = m0_in;
	'b001:mux8_out = m1_in;
	'b010:mux8_out = m2_in;
	'b011:mux8_out = m3_in;
	'b100:mux8_out = m4_in;
	'b101:mux8_out = m5_in;
	'b110:mux8_out = m6_in;
	'b111:mux8_out = m7_in;
	default: mux8_out = 'd0;
	endcase
      end
endmodule
