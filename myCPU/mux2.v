`timescale 1ns/1ns

module mux2(mux_out,m0_in,m1_in,sel_in);
   parameter width=32;
   
   output [width-1:0] mux_out; //数据输出端口
   input  [width-1:0] m0_in;    //数据输入端口0
   input  [width-1:0] m1_in;    //数据输入端口1
   input  sel_in;               //选择控制

   reg    [width-1:0]  mux_out;
     
   always @(*)
      begin
	case(sel_in)
	'b0:mux_out = m0_in;
	'b1:mux_out = m1_in;
	default: 
	       mux_out = 'd0;
	endcase
      end
endmodule
