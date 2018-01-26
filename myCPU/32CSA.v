module	_32CSA(
	input	[63:0]	A,
	input	[63:0]	B,
	input	[63:0]	C,
	output	[63:0]	S1,
	output	[63:0]	S2
);
	generate
	genvar	i;
	for(i = 0;i < 63;i = i+1)
	begin
		assign {S2[i+1],S1[i]} = A[i] + B[i] + C[i];
	end
	endgenerate
	assign	S2[0] = 1'b0;
	assign	S1[63] = A[63] + B[63] + C[63];
endmodule