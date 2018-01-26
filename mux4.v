`timescale 1ns/1ns

module mux4(mux4_out,m0_in,m1_in,m2_in,m3_in,sel_in);
   parameter width = 32;
     
   output[width-1:0] mux4_out;//数据输出端口
   input [width-1:0] m0_in;   //数据输入端口0，sel_in='b00
   input [width-1:0] m1_in;   //数据输入端口1，sel_in='b01
   input [width-1:0] m2_in;   //数据输入端口2，sel_in='b10
   input [width-1:0] m3_in;   //数据输入端口3，sel_in='b11
   
	input [1:0]       sel_in;  //选择控制输入
   
   reg   [width-1:0] mux4_out;

   always @(*)
      begin
	case(sel_in)
	'b00:mux4_out = m0_in;
	'b01:mux4_out = m1_in;
	'b10:mux4_out = m2_in;
	'b11:mux4_out = m3_in;
	default: mux4_out = 'd0;
	endcase
      end
endmodule
