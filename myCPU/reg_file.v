`ifdef PRJ1_FPGA_IMPL
	// the board does not have enough GPIO, so we implement 4 4-bit registers
    `define DATA_WIDTH 4
	`define ADDR_WIDTH 2
`else
    `define DATA_WIDTH 32
	`define ADDR_WIDTH 5
`endif

module reg_file(
	input clk,
	input rst,
	input [`ADDR_WIDTH - 1:0] waddr, //写口
	input [`ADDR_WIDTH - 1:0] raddr1, //读口
	input [`ADDR_WIDTH - 1:0] raddr2,
	input wen, //写使能
	input [`DATA_WIDTH - 1:0] wdata, //写数据
	output [`DATA_WIDTH - 1:0] rdata1, //读数据
	output [`DATA_WIDTH - 1:0] rdata2
);
    reg [31:0] Mem[31:0]; //32 x 32 registers
    integer i; //counter

/*同步写*/
    always @(posedge clk)
    begin
        if(rst)
    begin
        for(i=0; i<`DATA_WIDTH; i=i+1)
            begin Mem[i] <= 'd0; end
    end

    else
    begin
        if(wen && waddr == 'd0)
            begin Mem[waddr] <= 'd0; end
        else if(wen)    Mem[waddr] <= wdata;
        else    Mem[waddr] <= Mem[waddr];
    end
    end

/*异步读*/
    assign rdata1 =  Mem[raddr1];
    assign rdata2 =  Mem[raddr2];

	// TODO: insert your code

endmodule
