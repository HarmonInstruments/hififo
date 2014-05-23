`timescale 1ns / 1ps

module pcie_tx
  (
   input 	     clock,
   input 	     reset,
   input [15:0]      pcie_id,
   // read completion
   input 	     read_completion_valid,
   input [23:0]      read_completion_rid_tag,
   input [3:0] 	     read_completion_lower_addr,
   input [63:0]      read_completion_data,
   output reg 	     read_completion_ready = 0,
   // write
   input 	     write_request_valid,
   input [63:0]      write_request_data,
   input [63:0]      write_request_address,
   output reg 	     write_request_accepted = 0,
   output reg 	     write_request_ready = 0,
   // read request
   input 	     read_request_valid,
   input [63:0]      read_request_address,
   input [7:0] 	     read_request_tag,
   output reg 	     read_request_ready = 0,
   // AXI stream to PCI Express core
   input 	     axis_tx_tready,
   output reg [63:0] axis_tx_tdata = 0,
   output reg 	     axis_tx_1dw = 0,
   output reg 	     axis_tx_tlast = 0,
   output reg 	     axis_tx_tvalid = 0
   );

   reg [3:0] 	     tx_state = 0;
      
   wire [31:0] 	     read_completion_dw0 = 32'h4A000002; // 2 DW
   wire [31:0] 	     read_completion_dw1 = {pcie_id, 16'd8}; // 8 bytes
   wire [31:0] 	     read_completion_dw2 = {read_completion_rid_tag, 1'b0, read_completion_lower_addr, 3'd0 };
   wire [31:0] 	     read_completion_dw3;
   wire [31:0] 	     read_completion_dw4;
   endian_swap endian_swap_rc3(.din(read_completion_data[31: 0]), .dout(read_completion_dw3));
   endian_swap endian_swap_rc4(.din(read_completion_data[63:32]), .dout(read_completion_dw4));
      
   wire [31:0] 	     read_request_dw0 = {1'b0, 7'b0100000, 24'd128}; // 512 byte read request
   wire [31:0] 	     read_request_dw1 = {pcie_id, read_request_tag, 8'hFF};
   wire [31:0] 	     read_request_dw2 = read_request_address[63:32];
   wire [31:0] 	     read_request_dw3 = read_request_address[31: 0];

   reg [3:0] 	     write_request_count = 0;
   wire [31:0] 	     write_request_dw0 = {1'b0, 7'b1100000, 24'd32}; // 128 byte write request
   wire [31:0] 	     write_request_dw1 = {pcie_id, 16'h00FF};
   wire [31:0] 	     write_request_dw4;
   wire [31:0] 	     write_request_dw5;
   endian_swap endian_swap_wr4(.din(write_request_data[31: 0]), .dout(write_request_dw4));
   endian_swap endian_swap_wr5(.din(write_request_data[63:32]), .dout(write_request_dw5));

   always @(posedge clock)
     begin
	read_completion_ready <= tx_state == 3;
	read_request_ready <= tx_state == 5;
	write_request_ready <= write_request_count == 15;
	write_request_accepted <= (tx_state == 8) && axis_tx_tready;
	write_request_count <= tx_state == 8 ? write_request_count + axis_tx_tready : 1'b0;
	axis_tx_tvalid <= tx_state != 0;
	axis_tx_1dw <= tx_state == 3;
	axis_tx_tlast <= (tx_state == 3) || (tx_state == 5) || (write_request_count == 15);
	case(tx_state)
	  1: axis_tx_tdata <= {read_completion_dw1, read_completion_dw0};
	  2: axis_tx_tdata <= {read_completion_dw3, read_completion_dw2};
	  3: axis_tx_tdata <= {32'h0,               read_completion_dw4};
	  4: axis_tx_tdata <= {read_request_dw1, read_request_dw0};
	  5: axis_tx_tdata <= {read_request_dw3, read_request_dw2};
	  6: axis_tx_tdata <= {write_request_dw1, write_request_dw0};
	  7: axis_tx_tdata <= {write_request_address[31:0], write_request_address[63:32]};
	  8: axis_tx_tdata <= {write_request_dw5, write_request_dw4};
	  default: axis_tx_tdata <= 64'h0;
	endcase
	if(reset)
	  tx_state <= 4'd0;
	else if(tx_state == 0)
	  begin
	     if(read_completion_valid)
	       tx_state <= 4'd1;
	     else if(read_request_valid & ~read_request_ready)
	       tx_state <= 4'd4;
	     else if(write_request_valid & ~write_request_ready)
	       tx_state <= 4'd6;
	     else
	       tx_state <= 4'd0;
	  end
	else if(axis_tx_tready)
	  begin
	     if(tx_state == 3)
	       tx_state <= 4'd0;
	     else if(tx_state == 5)
	       tx_state <= 4'd0;
	     else if((tx_state == 8) && (write_request_count == 15))
	       tx_state <= 4'd0;
	     else
	       tx_state <= tx_state + 1'b1;
	  end
     end
   
endmodule // pcie_tx

module endian_swap
  (
   input [31:0] din,
   output [31:0] dout
   );
   assign dout[7:0] = din[31:24];
   assign dout[15:8] = din[23:16];
   assign dout[23:16] = din[15:8];
   assign dout[31:24] = din[7:0];
endmodule