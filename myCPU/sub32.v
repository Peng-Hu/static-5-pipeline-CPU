module sub32(
	input	[31:0]	A,
	input	[31:0]	B,
	input		cin,
	input		exA,
	input		exB,
	output  [31:0]	S,
	output		exS
);
	wire	[31:0]	d;
	wire	[31:0]	t;
	wire	[7:0]	D;
	wire	[7:0]	T;
	wire	[31:0]	C;
	assign	d = A & B;
	assign	t = A ^ B;
	assign	S[0] = ~A[0]&~B[0]&cin | ~A[0]&B[0]&~cin | A[0]&~B[0]&~cin | A[0]&B[0]&cin;
	assign  exS =  ~exA&~exB&C[31] | ~exA&exB&~C[31] | exA&~exB&~C[31] | exA&exB&C[31];
	generate
	genvar i;
	for(i = 1;i < 32;i = i+1)
	begin
		assign S[i] =  ~A[i]&~B[i]&C[i-1] | ~A[i]&B[i]&~C[i-1] | A[i]&~B[i]&~C[i-1] | A[i]&B[i]&C[i-1];
	end
	endgenerate
	carry3 ZERgroup(.d(d[3:0]),.t(t[3:0]),.cin(cin),.cout(C[2:0]),.D(D[0]),.T(T[0]));
	carry3 FIRgroup(.d(d[7:4]),.t(t[7:4]),.cin(C[3]),.cout(C[6:4]),.D(D[1]),.T(T[1]));
	carry3 SECgroup(.d(d[11:8]),.t(t[11:8]),.cin(C[7]),.cout(C[10:8]),.D(D[2]),.T(T[2]));
	carry3 THIgroup(.d(d[15:12]),.t(t[15:12]),.cin(C[11]),.cout(C[14:12]),.D(D[3]),.T(T[3]));
	carry3 FORgroup(.d(d[19:16]),.t(t[19:16]),.cin(C[15]),.cout(C[18:16]),.D(D[4]),.T(T[4]));
	carry3 FIVgroup(.d(d[23:20]),.t(t[23:20]),.cin(C[19]),.cout(C[22:20]),.D(D[5]),.T(T[5]));
	carry3 SIXZgroup(.d(d[27:24]),.t(t[27:24]),.cin(C[23]),.cout(C[26:24]),.D(D[6]),.T(T[6]));
	carry3 SEVgroup(.d(d[31:28]),.t(t[31:28]),.cin(C[27]),.cout(C[30:28]),.D(D[7]),.T(T[7]));
	carry4	big_group1(.D(D[3:0]),.T(T[3:0]),.cin(cin),.C({C[15],C[11],C[7],C[3]}));
	carry4	big_group2(.D(D[7:4]),.T(T[7:4]),.cin(C[15]),.C({C[31],C[27],C[23],C[19]}));
endmodule

