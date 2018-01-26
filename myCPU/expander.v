`timescale 1ns/1ns

module expander(
	input wire [width-1:0]	data_i,
	input wire 		ExtOp,
	output reg [31:0]	data_o
);
parameter width = 16;

always @(*)
begin
	data_o = {{(32-width){(ExtOp & data_i[width-1])}}, data_i[width-1:0]};
end
	
endmodule
			
