module _42compressor(
	input	[63:0] 	A,
	input	[63:0]	B,
	input	[63:0]	C,
	input	[63:0]	D,
	input			cin,
	output	[63:0]	S1,
	output	[63:0]	S2,
	output			cout
);
	wire	[63:0] C0;
	wire	[63:0]	CI;
	wire	[63:0]	s;
	assign	cout = C0[63];
	assign	CI[0] = cin;
	generate
	genvar	i;
	for(i = 0;i < 63;i = i+1)
	begin
		assign {C0[i],s[i]} = A[i] + B[i] + C[i];
		assign	{S2[i+1],S1[i]} = s[i] + D[i] + CI[i];
		assign	CI[i+1] = C0[i];
	end
	assign {C0[63],s[63]} = A[63] + B[63] + C[63];
	assign	S1[63] = s[63] + D[63] + CI[63];
	assign	S2[0] = 1'b0;
	endgenerate
endmodule