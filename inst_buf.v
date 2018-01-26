module inst_buf(
	input	clk,
	input 	resetn,
	
	input           stop,
	output          j_token,
	output	[31:0]	pc_to_ins,
	output	[31:0]	pc_to_if,
	output     [31:0]  pc_to_id,
	output	[31:0]	token_inst,
	input	[31:0]	j_pc,
	input			is_j,
	input			token,
	input	[31:0]	j_inst
);
	reg		[31:0]	PC;
	reg            token_hold;
	assign	pc_to_ins = token ? PC : j_pc;
	assign  j_token = token | token_hold;
	always@(posedge clk)
	begin
		if(~resetn)
			PC <= 'd0;
		else	if(is_j)
			PC <= j_pc;
		else  if(stop)
			PC <= PC;
		else
		    PC <= PC;
	end
	always@(posedge clk)
        begin
            if(~resetn)
                token_hold <= 'd0;
            else  if(stop)
                token_hold <= token;
            else if(token_hold)
                token_hold <= 1'b0;
        end
        
	assign  pc_to_id = j_token ? PC : 'd0;
	assign	pc_to_if = j_token? (PC + 'd4) : 'd0;
	assign	token_inst = j_token?  j_inst : 'd0;
endmodule