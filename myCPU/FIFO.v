`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/12/02 15:55:21
// Design Name: 
// Module Name: FIFO
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module FIFO
#(
    parameter   integer QUEUE_LENGTH = 10,      //less than 16
    parameter   integer DATA_WEDTH = 71
)
(
    input                       resetn,
    input                       clk,
    
    input                       complete,
    input   [DATA_WEDTH - 1:0]  wdata_pack,
    input                       valid,
    
    output                      ready,
    output  [DATA_WEDTH - 1:0]  rdata_pack,
    output                      is_empty,
    output                      is_full,
    output  [DATA_WEDTH * QUEUE_LENGTH -1 : 0]    queue_data_pack,
    output   reg     [3:0]           write_ptr
    );
    reg     [DATA_WEDTH-1:0]    queue   [QUEUE_LENGTH - 1:0];
    reg     [3:0]               read_ptr;
    reg     [QUEUE_LENGTH-1:0]  tag;
    assign  rdata_pack = is_empty ? 'd0 : queue[read_ptr];
    assign  is_empty = ~|tag;
    assign  is_full = &tag;
    genvar gen;
    generate
    for(gen = 0; gen < QUEUE_LENGTH; gen = gen + 1)
    begin
            assign queue_data_pack[(gen+1) * DATA_WEDTH - 1 : gen * DATA_WEDTH] = queue[gen];
    end
    endgenerate
    always @(posedge clk)
    begin
            if(~resetn)
                    read_ptr <= 'd0;
            else if(~is_empty && complete)
                    read_ptr <= (read_ptr == 'd9) ? 4'b0000 : (read_ptr + 1'b1);
            else
                    read_ptr <= read_ptr;
    end
    
    always @(posedge clk)
    begin
            if(~resetn)
                    write_ptr <= 'd0;
            else if(~is_full && valid)
                    write_ptr <= (write_ptr == 'd9) ? 4'b0000 : write_ptr + 1'b1;
            else
                    write_ptr <= write_ptr;
    end
    integer i;
    always @(posedge clk)
    begin
            if(~is_full && valid)
                    queue[write_ptr] <= wdata_pack;
            else
            begin
                    for(i = 0; i < QUEUE_LENGTH; i = i+1)
                    begin      queue[i] <= queue[i];    end
            end
     end
     assign ready = ~is_full;
     integer j;
     always @(posedge clk)
     begin
            if(~resetn)
                    tag <= 'd0;
            else if(~is_full && valid && ~is_empty && complete)
                    begin
                            tag[write_ptr] <=  1'b1;
                            tag[read_ptr] <= 1'b0;
                    end
            else if(~is_full && valid)
                    tag[write_ptr] <=  1'b1;
            else if(~is_empty && complete)
                    tag[read_ptr] <= 1'b0;
            else
                    for(j = 0; j < QUEUE_LENGTH; j = j+1)
                    begin      tag[i] <= tag[i];    end
     end
endmodule
