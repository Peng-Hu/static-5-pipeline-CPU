`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/10/14 22:00:09
// Design Name: 
// Module Name: forward_unit
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


module forward_unit(
    input wire            rst,
    input wire            clk,
    input wire  [4:0]     Asource,
    input wire  [4:0]     Bsource,
    input wire  [4:0]     mem_dest,
    input wire            mem_load,
    input wire  [4:0]     wb_dest,
        
    output reg    [1:0]   ForwardA,
    output reg    [1:0]   ForwardB,
    output reg            stop
    );
    always @(*)
    begin   
            if(~rst)
                ForwardA = 'd0;
            else if(Asource != mem_dest && Asource != wb_dest)
                ForwardA = 'd0;
            else if(Asource == mem_dest && mem_dest != 'd0 && mem_load == 1'b0 )
                ForwardA = 'd1;
            else if(Asource == wb_dest && wb_dest != 'd0)
                ForwardA = 'd2;
            else
                ForwardA = 'd0;
            
            if(~rst)
                ForwardB = 'd0;
            else if(Bsource != mem_dest && Bsource != wb_dest)
                ForwardB = 'd0;
            else if(Bsource == mem_dest && mem_dest != 'd0 && mem_load == 1'b0 )
                ForwardB = 'd1;
            else if(Bsource == wb_dest && wb_dest != 'd0)
                ForwardB = 'd2;
            else
                ForwardB = 'd0;
                
            
    end
    always@(*)
    //always @(Asource or mem_dest or Bsource )
    begin
            if(~rst)
                    stop = 1'b0;
                else if((Asource == mem_dest || Bsource == mem_dest) && mem_load == 1'b1 && mem_dest != 'd0)
                    stop = 1'b1;
                else
                    stop = 1'b0;
    end
endmodule
