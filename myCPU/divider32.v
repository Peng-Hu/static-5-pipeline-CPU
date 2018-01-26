module divider (
	input		div_clk,
	input		resetn,
	input		div,
	input		div_signed,
	input	[31:0]	x,	//devidend
	input	[31:0]	y,	//devisor
	output	[31:0]	s,	//quotient
	output	[31:0]	r,	//reminder
	output		complete
);

reg [65:0] a;	//最终被除数或余数[65:33]|商[32:0]
reg [65:0] shift_a;
reg [32:0] b, negb;
reg [5:0] cnt;
reg current;
reg next;
wire first;	//1st clk, fetch data

wire asign;
wire bsign;
wire [32:0] abs_a;
wire [32:0] abs_b;	//除数
wire [32:0] abs_negb;
reg  ssign, rsign;
wire [33:0] neg_r;
//wire [31:0] temp_s, temp_r;
wire [31:0] debug_r, debug_s;
//wire [32:0] neg_r, neg_s;
wire [32:0] addA, addB, addS;
wire cin;

/*预处理，正确地取除数和被除数的绝对值，补一位0*/
/*初始化余数为0，商为0*/
assign asign = x[31] & div_signed;
assign abs_a = ({32{asign}}^x) + asign;
assign bsign = y[31] & div_signed;
assign abs_b = ({32{bsign}}^y) + bsign;
assign abs_negb = ~abs_b;
assign first = div & ~(|cnt);

/*结束处理，判断商和余数的值，有符号的话要补正*/
/*因为做的绝对值除法，所以要保证temp_r,s都是正的，然后再补符号*/
assign complete = cnt[5] & cnt[1] & ~cnt[0] & ~(|cnt[4:2]);	//cnt = 34
assign s = {32{complete}} & (({32{ssign}}^a[31:0]) + ssign);
assign r = {32{complete}} & (({32{rsign}}^neg_r[31:0]) + rsign);



assign neg_r = a[65] ? (a[65:33] + ~negb) : a[65:33];
/*
assign temp_r = a[65] ? neg_r[31:0] : a[64:33];
assign s = {32{complete}} & {(ssign | temp_s[31]), temp_s[30:0]};
assign r = {32{complete}} & {(rsign | temp_r[31]), temp_r[30:0]};
*/
assign debug_s = a[31:0];
assign debug_r = a[65:33];

//每次迭代的加减法

assign addA = shift_a[65:33];
assign addB = b;
assign cin = (cnt == 5'd1) | a[0];

sub32 sub32 (
	.A	(addA[31:0]	),
	.B	(addB[31:0]	),
	.cin	(cin		),
	.exA	(addA[32]	),
	.exB	(addB[32]	),
	.S	(addS[31:0]	),
	.exS	(addS[32]	)
);

//FSM state
always @(cnt or div or current or resetn or complete)
begin // 0=stop, 1=running
next = ~div & resetn;
casex(current)
1'b0: next = div;	
1'b1: next = ~complete;
endcase
end

//FSM input
always @(posedge div_clk)
begin
    if(~resetn)
    begin
        current <= 1'b0;
        a <= 66'b0;
	b <= 33'b0;
	negb <= 33'b0;
        shift_a <= 66'b0;
        cnt <= 6'b0;
	ssign <= 1'b0;
	rsign <= 1'b0;
    end
    else
    begin
        casex(current)
        0:  //stop
        begin
            current <= next;
	    a <= first ? {33'b0, abs_a} : a;
	    shift_a <= first ? {32'b0, abs_a, 1'b0} : shift_a;
	    negb <= first ? abs_negb : 33'b0;
	    b <= first ? abs_negb: b;
	    cnt <= first ? 6'd1 : 6'd0;
	    ssign <= first ? (div_signed & (x[31] ^ y[31])) : 1'b0;
	    rsign <= first ? (div_signed & x[31]) : 1'b0;
        end
        1:  //running
        begin
            current <= next;
	    a <= {addS[32:0], a[31:0], ~addS[32]}; 
            shift_a <= {addS[31:0], a[31:0], ~addS[32], 1'b0};//余数33，原a32，商1 << 1
            negb <= negb;
	    b <= addS[32] ? ~negb : negb;
	    cnt <= cnt+1;
	    ssign <= ssign;
	    rsign <= rsign;
        end
        endcase
    end
end

endmodule
