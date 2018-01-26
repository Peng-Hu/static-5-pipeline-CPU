module multi(
	input			clk,
	input			resetn,
	input			sig,
	input			en,
	input	[31:0]	S1,
	input	[31:0]	S2,
	//input			sig,	//0 --> unsigned; 1 --> signed
	output	[63:0]	C,
	output  reg    enable
);
    reg    [63:0]  record1,record2,record3,record4,record5;
	/*reg		[33:0]	A;
	reg	[33:0]	B;
	
	
	always@(posedge clk)
	begin
		if(~resetn)
		begin
			//A	<= 'd0;
			//B	<= 'd0;
			enable   <=  1'b0;
		end
		else if(en)
		begin
			//A   <= {{2{sig&S1[31]}},S1};
			//B	<=	{{2{sig&S2[31]}},S2};
			enable   <=  en;
		end
		else
		begin
			//A	<=	'd0;
			//B	<=	'd0;
			enable   <=  1'b0;
		end
	end*/
	//test code segement//
	wire   [33:0]  A;
	wire   [33:0]  B;
	assign A   =   {{2{sig&S1[31]}},S1};
	assign B   =   {{2{sig&S2[31]}},S2};
	always @(posedge clk)
	if(~resetn)
	   enable <= 1'b0;
	else if(en)
	   enable  <=  1'b1;
	else
	   enable  <=  1'b0;
	//test code end
	wire	[63:0]	I	[16:0];
	wire	[63:0]	II	[7:0];
	wire	[63:0]	III	[3:0];
	wire	[63:0]	IV	[1:0];
	wire	[63:0]	V	[1:0];
	wire	[34:0]	X1,X2,X3,X4;
	assign	X1 = {A[33],A};
	assign	X2 = ~X1+1;
	assign	X3 = X1<<1;
	assign	X4 = ~(X1<<1)+1;
	assign	I[0] = B[1] ? (B[0] ? ({{29{X2[34]}},X2}) 
									: ({{29{X4[34]}},X4})) 
						  :(B[0] ? ({{29{X1[34]}},X1}) 
									:  64'b0);
	always@(posedge clk)
    begin
            if(~resetn)
            begin
                       record1    <= 'd0;
                       record2    <= 'd0;
                       record3   <=  'd0;
                       record4  <=  'd0;
                       record5  <=  'd0;
            end
            else if(en)
            begin
                       record1   <= III[0];
                       record2    <= III[1];
                       record3   <=  III[2];
                       record4  <=  III[3];
                       record5  <=  II[7];
            end
            else
            begin
                       record1    <= 'd0;
                       record2    <= 'd0;
                       record3   <=  'd0;
                       record4  <=  'd0;
                       record5  <=  'd0;
            end
    end
	generate
	genvar	i;
	for(i = 1;i < 17;i = i+1)
	begin
			assign	I[i] = B[2*i+1] ? (B[2*i] ? (B[2*i-1] ? 64'b0 
															: ({{(29-2*i){X2[34]}},X2,{2*i{1'b0}}})) 
												: (B[2*i-1] ? ({{(29-2*i){X2[34]}},X2,{2*i{1'b0}}}) 
																: ({{(29-2*i){X4[34]}},X4,{2*i{1'b0}}}))) 
									  :(B[2*i] ? (B[2*i-1] ? ({{(29-2*i){X3[34]}},X3,{2*i{1'b0}}}) 
															: ({{(29-2*i){X1[34]}},X1,{2*i{1'b0}}})) 
												: (B[2*i-1] ? ({{(29-2*i){X1[34]}},X1,{2*i{1'b0}}}) 
																: 64'b0));//(B[2*i-1] + B[2*i] - 2*B[2*i+1])
	end
	endgenerate
	_42compressor layer1(.A(I[1]),.B(I[2]),.C(I[3]),.D(I[4]),.cin(1'b0),.S1(II[0]),.S2(II[1]));
	_42compressor layer1_1(.A(I[5]),.B(I[6]),.C(I[7]),.D(I[8]),.cin(1'b0),.S1(II[2]),.S2(II[3]));
	_42compressor layer1_2(.A(I[9]),.B(I[10]),.C(I[11]),.D(I[12]),.cin(1'b0),.S1(II[4]),.S2(II[5]));
	_42compressor layer1_3(.A(I[13]),.B(I[14]),.C(I[15]),.D(I[16]),.cin(1'b0),.S1(II[6]),.S2(II[7]));
	_42compressor layer2(.A(I[0]),.B(II[0]),.C(II[1]),.D(II[2]),.cin(1'b0),.S1(III[0]),.S2(III[1]));
	_42compressor layer2_1(.A(II[3]),.B(II[4]),.C(II[5]),.D(II[6]),.cin(1'b0),.S1(III[2]),.S2(III[3]));
	_42compressor layer3(.A(record2),.B(record3),.C(record4),.D(record5),.cin(1'b0),.S1(IV[0]),.S2(IV[1]));
	_32CSA		 layer4(.A(record1),.B(IV[0]),.C(IV[1]),.S1(V[0]),.S2(V[1]));
	wire cout;
	add32		 layer5(.A(V[0][31:0]),.B(V[1][31:0]),.cin(1'b0),.S(C[31:0]),.cout(cout));
	add32		 layer5_1(.A(V[0][63:32]),.B(V[1][63:32]),.cin(cout),.S(C[63:32]));
endmodule