`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/12/02 15:44:52
// Design Name: 
// Module Name: axi_master
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

`define C_M_AXI_ADDR_WIDTH 32
module axi_master
#(
    parameter integer  C_M_AXI_DATA_WIDTH       = 32
)
(
  // System Signals
  input wire M_AXI_ACLK,
  input wire M_AXI_ARESETN,
  ////////////////////////////////////////////////////////////////////////////
  // Master Interface Write Address
  output wire [`C_M_AXI_ADDR_WIDTH-1:0]      M_AXI_AWADDR,
  output wire [2:0] M_AXI_AWSIZE,
  output wire M_AXI_AWVALID,
  input  wire M_AXI_AWREADY,
  ////////////////////////////////////////////////////////////////////////////
  // Master Interface Write Data
  output wire [C_M_AXI_DATA_WIDTH-1:0]      M_AXI_WDATA,
  output wire [C_M_AXI_DATA_WIDTH/8-1:0]    M_AXI_WSTRB,
  output wire M_AXI_WVALID,
  input  wire M_AXI_WREADY,
  ////////////////////////////////////////////////////////////////////////////
  // Master Interface Write Response
  input  wire M_AXI_BVALID,
  output wire M_AXI_BREADY,
  ////////////////////////////////////////////////////////////////////////////
  // Master Interface Read Address
  output wire [3:0] M_AXI_ARID,
  output wire [`C_M_AXI_ADDR_WIDTH-1:0]      M_AXI_ARADDR,
  output wire [2:0] M_AXI_ARSIZE,
  output wire M_AXI_ARVALID,
  input  wire M_AXI_ARREADY,
  ////////////////////////////////////////////////////////////////////////////
  // Master Interface Read Data
  input wire [3:0] M_AXI_RID,
  input  wire [C_M_AXI_DATA_WIDTH-1:0]      M_AXI_RDATA,
  input  wire M_AXI_RVALID,
  output wire M_AXI_RREADY,
  ////////////////////////////////////////////////////////////////////////////
  // Master Interface with IF stage
  input  wire [31:0] PC,
  input  wire        PC_valid,
  input  wire        ins_ready,
  output wire        PC_ready,
  output wire [63:0] ins_back_pack,
  output wire        ins_valid,
  input wire        TLBL,
  output    wire    TLBL_i,
  ////////////////////////////////////////////////////////////////////////////
  // Master Interface with ID stage
  input  wire        weap,
  input  wire        token_i,
  input wire        j_b,
  input wire        is_int,
  ////////////////////////////////////////////////////////////////////////////
  // Master Interface with MEM stage
  input  wire [70:0] MEM_data_pack,
  input  wire        MEM_data_valid,
  output wire        MEM_data_ready,
  output wire [31:0] MEM_rdata,
  output wire        MEM_rvalid
    );
// AXI4 internal temp signals
    wire [`C_M_AXI_ADDR_WIDTH-1:0] axi_awaddr;
    reg axi_awvalid;
    wire [C_M_AXI_DATA_WIDTH-1:0] axi_wdata;
    reg axi_wvalid;
    wire    [3:0]   axi_arid;                       //0 for data ID ,1 for instruction ID
    wire [`C_M_AXI_ADDR_WIDTH-1:0] axi_araddr_d;
    wire [`C_M_AXI_ADDR_WIDTH-1:0] axi_araddr_i;
    reg axi_arvalid;
    wire axi_arvalid_d;
    wire axi_arvalid_i;
    wire [2:0]  axi_awsize;
    wire    [3:0]   axi_wstrb;
    reg         mem_is_write;
    reg   [3:0] PC_count;
    reg         weap_keep;
    reg         token_keep;
    reg         is_deleslot_sent;
    reg         token;
    reg         re_token;
    reg         data_read;
    reg         [3:0]   w_count;
    
    assign M_AXI_AWADDR =  axi_awaddr;
    assign M_AXI_AWSIZE = axi_awsize;
    assign M_AXI_AWVALID = axi_awvalid;
    assign M_AXI_WDATA = axi_wdata;
    assign M_AXI_WVALID = axi_wvalid;
    assign M_AXI_BREADY = 1'b1;
    assign M_AXI_ARADDR =  data_read ? axi_araddr_d : axi_araddr_i;        //读数据的优先级高于读指令
    assign M_AXI_ARSIZE = 'd2;
    assign M_AXI_ARVALID = axi_arvalid;
    assign M_AXI_RREADY = 1'b1;
    assign M_AXI_WSTRB = axi_wstrb;
    assign M_AXI_ARID = axi_arid;
////////////////////////////////////////////////////////////////////////////
    //data queue
    wire    [359 : 0]   data_pack;
    wire    data_queue_empty;
    wire    data_queue_full;
    wire    mem_write;
    wire    data_ready;
    wire    miss;
    wire    [3:0]   write_ptr;
    assign  mem_write = |axi_wstrb & ~data_queue_empty;
    FIFO    
    #(
         .QUEUE_LENGTH  ('d10),
         .DATA_WEDTH    ('d36)
    )data_queue
    (
        .resetn         (M_AXI_ARESETN),
        .clk            (M_AXI_ACLK),
        
        .complete       (M_AXI_WREADY& axi_wvalid | ( (M_AXI_ARREADY && (~|axi_arid)) && ~mem_write)),
        .wdata_pack     (MEM_data_pack[35:0]),
        .valid          (MEM_data_ready & MEM_data_valid & (miss | (|(MEM_data_pack[35:32])))),
        
        .ready          (data_ready),
        .rdata_pack     ({axi_wstrb,axi_wdata}),
        .is_empty       (data_queue_empty),
        .is_full        (data_queue_full),
        .queue_data_pack(data_pack),
        .write_ptr(write_ptr)
    );
////////////////////////////////////////////////////////////////////////////
        //addr queue
    wire    [349:0]     addr_pack;
    wire    addr_queue_full;
    wire    addr_queue_empty;
    wire    addr_ready;
    wire    [9:0]   tag_compare;
    reg     [9:0]   w_valid;
    wire    [31:0]  addr;
    assign  axi_arvalid_d = axi_arvalid ? 1'b0 : ~addr_queue_empty & ~mem_write & miss ;
    assign  MEM_data_ready = data_ready&addr_ready&((mem_is_write&(|MEM_data_pack[35:32])&~miss)|(~mem_is_write));
    assign  axi_awaddr = addr;
    assign  axi_araddr_d = addr;
    FIFO    
    #(
         .QUEUE_LENGTH  ('d10),
         .DATA_WEDTH    ('d35)
    )addr_queue
    (
         .resetn         (M_AXI_ARESETN),
         .clk            (M_AXI_ACLK),
               
         .complete       (M_AXI_AWREADY&axi_awvalid|( (M_AXI_ARREADY&(~|axi_arid))&~mem_is_write)),
         .wdata_pack     (MEM_data_pack[70:36]),
         .valid          (MEM_data_ready & MEM_data_valid & (miss | (|(MEM_data_pack[35:32])))),
                
         .ready          (addr_ready),
         .rdata_pack     ({axi_awsize,addr}),
         .is_empty       (addr_queue_empty),
         .is_full        (addr_queue_full),
         .queue_data_pack(addr_pack)
    );
    
    genvar k;
    generate
            for(k = 0; k < 10;k = k+1)
            begin
                    assign  tag_compare[k] = (addr_pack[k*35+31:k*35+2] == MEM_data_pack[67:38]);
            end
    endgenerate
    integer i;
    //integer j;
    always @(posedge M_AXI_ACLK)
    begin
           if(~M_AXI_ARESETN)
                   w_valid <= 'd0;
           else if(MEM_data_valid && MEM_data_ready && (|(MEM_data_pack[35:32])))
           begin
                    /*for(i = 0;i < write_ptr; i =i+1)
                    begin   w_valid[i] <= w_valid[i] & (~tag_compare[i]);    end
                    w_valid[write_ptr] <= &MEM_data_pack[35:32];
                    for(j = write_ptr+1;j < 10; j =j+1)
                    begin   w_valid[j] <= w_valid[j] & (~tag_compare[j]);    end*/
                    w_valid[0] <= (write_ptr=='d0) ? &MEM_data_pack[35:32] : (w_valid[0] & (~tag_compare[0]));
                    w_valid[1] <= (write_ptr=='d1) ? &MEM_data_pack[35:32] : (w_valid[1] & (~tag_compare[1]));
                    w_valid[2] <= (write_ptr=='d2) ? &MEM_data_pack[35:32] : (w_valid[2] & (~tag_compare[2]));
                    w_valid[3] <= (write_ptr=='d3) ? &MEM_data_pack[35:32] : (w_valid[3] & (~tag_compare[3])); 
                    w_valid[4] <= (write_ptr=='d4) ? &MEM_data_pack[35:32] : (w_valid[4] & (~tag_compare[4]));
                    w_valid[5] <= (write_ptr=='d5) ? &MEM_data_pack[35:32] : (w_valid[5] & (~tag_compare[5]));
                    w_valid[6] <= (write_ptr=='d6) ? &MEM_data_pack[35:32] : (w_valid[6] & (~tag_compare[6]));
                    w_valid[7] <= (write_ptr=='d7) ? &MEM_data_pack[35:32] : (w_valid[7] & (~tag_compare[7]));
                    w_valid[8] <= (write_ptr=='d8) ? &MEM_data_pack[35:32] : (w_valid[8] & (~tag_compare[8]));
                    w_valid[9] <= (write_ptr=='d9) ? &MEM_data_pack[35:32] : (w_valid[9] & (~tag_compare[9]));
           end
           else if(MEM_data_valid && MEM_data_ready && miss)
           begin
                    w_valid[write_ptr] <= 1'b0;
           end
           else
           begin
                    for(i = 0;i < 10; i =i+1)
                    begin   w_valid[i] <= w_valid[i];    end
           end
    end
    assign  miss = ~mem_is_write & (~|(tag_compare & w_valid)) & (~|(MEM_data_pack[35:32]));
    assign  MEM_rvalid = miss ? (M_AXI_RVALID & (~|M_AXI_RID)) : MEM_data_valid & (~|MEM_data_pack[35:32]);
    assign  MEM_rdata = miss ? M_AXI_RDATA : (({32{tag_compare[9]&w_valid[9]}}&data_pack[355:324]) | ({32{tag_compare[8]&w_valid[8]}}&data_pack[319:288]) | ({32{tag_compare[7]&w_valid[7]}}&data_pack[283:252]) | ({32{tag_compare[6]&w_valid[6]}}&data_pack[247:216]) | ({32{tag_compare[5]&w_valid[5]}}&data_pack[211:180])
                                              | ({32{tag_compare[4]&w_valid[4]}}&data_pack[175:144]) | ({32{tag_compare[3]&w_valid[3]}}&data_pack[139:108]) | ({32{tag_compare[2]&w_valid[2]}}&data_pack[103:72]) | ({32{tag_compare[1]&w_valid[1]}}&data_pack[67:36])
                                              | ({32{tag_compare[0]&w_valid[0]}}&data_pack[31:0]));
                                              
    always @(posedge M_AXI_ACLK)
    begin
            if(~M_AXI_ARESETN)
                data_read <= 1'b0;
            else if(axi_arvalid_d)
                data_read <= 1'b1;
            else if(axi_arvalid && M_AXI_ARREADY)
                 data_read <= 1'b0;
            else
                data_read <= data_read;
    end
////////////////////////////////////////////////////////////////////////////
    //instruction requst queue
    wire    ins_queue_empty;
    wire    ins_queue_full;
    wire   PC_queue_empty;
    wire    tlbl;
    wire    req_ready;
    assign  axi_arvalid_i = ~ins_queue_empty  & (~token_keep | is_deleslot_sent) & ~is_int;
    assign  PC_ready = req_ready & (~(token & is_deleslot_sent));
    FIFO    
    #(
         .QUEUE_LENGTH  ('d10),
         .DATA_WEDTH    ('d33)
    )ins_req_queue
    (
         .resetn         (M_AXI_ARESETN & (~(token|re_token) | is_deleslot_sent) & ~is_int),
         .clk            (M_AXI_ACLK),
            
         .complete       (M_AXI_ARREADY&(|axi_arid)&axi_arvalid),
         .wdata_pack     ({TLBL,PC}),
         .valid          (PC_valid),
           
         .ready          (req_ready),
         .rdata_pack     ({tlbl,axi_araddr_i}),
         .is_empty       (ins_queue_empty),
         .is_full        (ins_queue_full),
         .queue_data_pack()
    );
   always@(posedge M_AXI_ACLK)
       begin
             if(~M_AXI_ARESETN | is_int)
                     is_deleslot_sent <= 1'b0;
             else if(j_b & PC_queue_empty & ~(M_AXI_ARREADY & axi_arvalid &(|axi_arid)))            //延迟槽指令在req queue
                     is_deleslot_sent <=  1'b1;
             else if(is_deleslot_sent & (M_AXI_ARREADY & axi_arvalid &(|axi_arid)))
                     is_deleslot_sent <= 1'b0;
             else
                     is_deleslot_sent <= is_deleslot_sent;
       end
////////////////////////////////////////////////////////////////////////////
     //PC wait queue
     wire   PC_queue_full;

    FIFO    
    #(
         .QUEUE_LENGTH  ('d10),
         .DATA_WEDTH    ('d33)
    )PC_queue
    (
         .resetn         (M_AXI_ARESETN & ~weap & ~weap_keep & ~is_int),
         .clk            (M_AXI_ACLK),
               
         .complete       (ins_ready & ins_valid),
         .wdata_pack     ({tlbl,axi_araddr_i}),
         .valid          (M_AXI_ARREADY&(|axi_arid)&axi_arvalid),
              
         .ready          (),
         .rdata_pack     ({TLBL_i,ins_back_pack[63:32]}),
         .is_empty       (PC_queue_empty),
         .is_full        (PC_queue_full),
         .queue_data_pack()
    );
  always@(posedge M_AXI_ACLK)
    begin
          if(~M_AXI_ARESETN)
                  PC_count <= 'd0;
          else if(M_AXI_ARREADY && (|axi_arid) && ~(M_AXI_RVALID && (|M_AXI_RID)))
                  PC_count <= PC_count + 1'b1;
          else if(M_AXI_RVALID && (|M_AXI_RID) && ~(M_AXI_ARREADY && (|axi_arid)))
                  PC_count <= PC_count - 1'b1;
          else
                  PC_count <= PC_count;
    end
   always@(posedge M_AXI_ACLK)
      begin
            if(~M_AXI_ARESETN)
                    weap_keep <= 1'b0;
            else if(weap | is_int )
                    weap_keep <=  1'b1;
            else if(weap_keep && PC_count == 'd0 && (is_deleslot_sent || ~((M_AXI_ARREADY)  &(|axi_arid))))
                    weap_keep <= 1'b0;
            else
                    weap_keep <= weap_keep;
      end
   always@(posedge M_AXI_ACLK)
         begin
               if(~M_AXI_ARESETN)
                       token_keep <= 1'b0;
               else if(token_i | is_int)
                       token_keep <=  1'b1;
               else if(PC_count == 'd0 && ~is_deleslot_sent && ~((M_AXI_ARREADY|axi_arvalid)  &(|axi_arid)))
                       token_keep <= 1'b0;
               else
                       token_keep <= token_keep;
         end
   always@(posedge M_AXI_ACLK)
             begin
                   if(~M_AXI_ARESETN | is_int)
                           token <= 1'b0;
                   else if(token_i)            //延迟槽指令再req queue
                           token <=  1'b1;
                   else if(~is_deleslot_sent)
                           token <= 1'b0;
                   else
                           token <= token;
             end
      always@(posedge M_AXI_ACLK)
                       begin
                             if(~M_AXI_ARESETN | is_int)
                                     re_token <= 1'b0;
                             else if(token_i & ~(PC_ready & PC_valid))            //延迟槽指令再req queue
                                     re_token <=  1'b1;
                             else if(PC_ready & PC_valid)
                                     re_token <= 1'b0;
                             else
                                     re_token <= re_token;
                       end
////////////////////////////////////////////////////////////////////////////
        //instruction wait queue
        wire    insw_queue_empty;
        wire    insw_queue_full;
        assign ins_valid = ~PC_queue_empty & ~insw_queue_empty & ~weap;
        FIFO    
        #(
             .QUEUE_LENGTH  ('d10),
             .DATA_WEDTH    ('d32)
        )ins_wait_queue
        (
             .resetn         (M_AXI_ARESETN & ~weap & ~weap_keep & ~is_int),
             .clk            (M_AXI_ACLK),
                
             .complete       (ins_ready & ins_valid),
             .wdata_pack     (M_AXI_RDATA),
             .valid          (M_AXI_RVALID & (|M_AXI_RID)),
               
             .ready          (),
             .rdata_pack     (ins_back_pack[31:0]),
             .is_empty       (insw_queue_empty),
             .is_full        (insw_queue_full),
             .queue_data_pack()
        );
////////////////////////////////////////////////////////////////////////////
    //Write Address Channel
    always @(posedge M_AXI_ACLK)
    begin
        if (M_AXI_ARESETN == 0 )
            axi_awvalid <= 1'b0;
        else if(~axi_awvalid && mem_is_write && ~addr_queue_empty && (w_count == 'd0))
            axi_awvalid <= 1'b1;    
        else if (M_AXI_AWREADY)
            axi_awvalid <= 1'b0;
        else
          axi_awvalid <= axi_awvalid;
    end
////////////////////////////////////////////////////////////////////////////
    // WVALID logic, similar to the axi_awvalid always block above
    always @(posedge M_AXI_ACLK)
    begin
        if (M_AXI_ARESETN == 0 )
            axi_wvalid <= 1'b0;
       else if (~axi_wvalid && mem_write && (w_count == 'd0))
            axi_wvalid <= 1'b1;
       else if (axi_wvalid && M_AXI_WREADY)
            axi_wvalid <= 1'b0;
       else
          axi_wvalid <= axi_wvalid;
   end
////////////////////////////////////////////////////////////////////////////
  //Read Address Channel
  assign    axi_arid = data_read ? 'd0 : 'd1;
  always @(posedge M_AXI_ACLK)
  begin
    if (M_AXI_ARESETN == 0 )
        axi_arvalid <= 1'b0;
    else if ((~axi_arvalid) && ((axi_arvalid_i&&~PC_queue_full) || axi_arvalid_d))          //若PC队列不能接受新数据则不再传PC
        axi_arvalid <= 1'b1;
    else if (M_AXI_ARREADY)
        axi_arvalid <= 1'b0;
    else if(axi_arvalid && ~((axi_arvalid_i&&~PC_queue_full ) || data_read))
        axi_arvalid <= 1'b0;
    else
      axi_arvalid <= axi_arvalid;
  end
////////////////////////////////////////////////////////////////////////////
    // record data transfer direction
  always@(posedge M_AXI_ACLK)
  begin
        if(~M_AXI_ARESETN)
                mem_is_write <= 1'b0;
        else if(mem_write)
                mem_is_write <= 1'b1;
        else if(addr_queue_empty && data_queue_empty && w_count == 'd0)
                mem_is_write <= 1'b0;
        else
                mem_is_write <= mem_is_write;
  end
  
    always@(posedge M_AXI_ACLK)
  begin
        if(~M_AXI_ARESETN)
                w_count <= 'd0;
        else if(axi_wvalid && M_AXI_WREADY && M_AXI_BVALID)
                w_count <= w_count;
        else if(axi_awvalid && M_AXI_AWREADY)
                w_count <= w_count + 'd1;
        else if(M_AXI_BVALID)
                w_count <= w_count - 'd1;
        else
                w_count <= w_count;
  end
endmodule
