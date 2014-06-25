module pcie_tx
  (
   input 	     clock,
   input 	     reset,
   input [15:0]      pcie_id,
   // read completion (rc)
   input 	     rc_valid,
   input [23:0]      rc_rid_tag,
   input [3:0] 	     rc_lower_addr,
   input [63:0]      rc_data,
   output 	     rc_ready,
   // read request (rr)
   input 	     rr_valid,
   input [63:0]      rr_addr,
   input [7:0] 	     rr_tag,
   output 	     rr_ready,
   // write request (wr)
   input 	     wr_valid,
   input [63:0]      wr_addr,
   output 	     wr_ready,
   input [63:0]      wr_data,
   // AXI stream to PCI Express core
   input 	     tx_tready,
   output reg [63:0] tx_tdata = 0,
   output reg 	     tx_1dw = 0,
   output reg 	     tx_tlast = 0,
   output reg 	     tx_tvalid = 0
   );

   function [31:0] es; // endian swap
      input [31:0]   x;
      es = {x[7:0], x[15:8], x[23:16], x[31:24]};
   endfunction

   reg [63:0] 	     wr_data_q;
   reg [4:0] 	     state = 0;
   wire 	     rr_32 = rr_addr[63:32] == 0;
   wire 	     wr_32 = wr_addr[63:32] == 0;
   wire 	     last = (state == 3) || (state == 5) || (state == 23);
   wire [4:0] 	     state_next = (rc_valid & (state != 3)) ? 3'd1 : (rr_valid ? 3'd4 : (wr_valid ? 3'd6 : 3'd0));
   assign rc_ready = (tx_tready | ~tx_tvalid) && (state == 3);
   assign rr_ready = (tx_tready | ~tx_tvalid) && (state == 5);
   assign wr_ready = (tx_tready | ~tx_tvalid) && (state > 6) && (state < 23);
   
   always @(posedge clock)
     begin
	tx_tvalid <= reset ? 1'b0 : (state != 0);
	if(reset)
	  state <= 5'd0;
	else if((tx_tready | ~tx_tvalid) && (last || (state == 0)))
	  state <= state_next;
	else if((tx_tready | ~tx_tvalid) && (state != 0))
	  state <= state + 1'b1;
	if(tx_tready | ~tx_tvalid)
	  begin
	     tx_1dw <= ((state == 3) || ((state == 5) && (rr_addr[63:32] == 0)) || ((state == 23) && (wr_addr[63:32] == 0)));
	     tx_tlast <= (state == 3) || (state == 5) || (state == 23);
	     wr_data_q <= wr_data;
	     case(state)
	       // idle
	       0: tx_tdata <= 64'h0;
	       // read completion (rc)
	       1: tx_tdata <= {pcie_id, 16'd8, 32'h4A000002}; // always 2 DW
	       2: tx_tdata <= {es(rc_data[31:0]), rc_rid_tag, 1'b0, rc_lower_addr, 3'd0}; // rc DW3, DW2
	       3: tx_tdata <= {32'h0, es(rc_data[63:32])}; // rc DW4
	       // read request (rr)
	       4: tx_tdata <= {pcie_id, rr_tag[7:0], 8'hFF, 2'd0, ~rr_32, 29'd128}; // always 128 DW
	       5: tx_tdata <= {rr_addr[31:0], (rr_32 ? rr_addr[31:0]: rr_addr[63:32])};
	       // write request (wr)
	       6: tx_tdata <= {pcie_id, 16'h00FF, 2'b01, ~wr_32, 29'd32};
	       7: tx_tdata <= wr_32 ? {es(wr_data[31:0]), wr_addr[31:0]} : {wr_addr[31:0], wr_addr[63:32]};
	       default: tx_tdata <= wr_32 ? {es(wr_data[31:0]), es(wr_data_q[63:32])} : {es(wr_data_q[63:32]),es(wr_data_q[31:0])};
	     endcase
	  end
     end   
endmodule
