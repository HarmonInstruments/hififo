`timescale 1ns / 1ps

module pcie_rx
  (
   input 	     clock,
   input 	     reset,
   // outputs
   output reg 	     write_valid = 0,
   output reg 	     read_valid = 0,
   output reg 	     completion_valid = 0,
   output reg [63:0] data = 0,
   output reg [12:0] address = 0,
   output reg [23:0] rid_tag = 0,
   // AXI stream from PCIE core
   input 	     tvalid,
   input 	     tlast,
   input [63:0]      tdata
   );
   
   reg 		     tvalid_q = 0;
   reg [63:0] 	     tdata_q = 0;
   reg 		     tlast_q = 0;
   reg 		     wait_dw01 = 1;
   reg 		     wait_dw23 = 0;
   reg 		     wait_dw45 = 0;
   reg [31:0] 	     previous_dw = 0;
   reg 		     is_write_32 = 0;
   reg 		     is_cpld = 0;
   reg 		     is_read_32_2dw = 0;
            
   // receive
   always @ (posedge clock)
     begin
	tvalid_q <= tvalid;
	tlast_q <= tlast;
	tdata_q <= tdata;
	if(tvalid_q)
	  begin
	     data[15: 0] <= {previous_dw[23:16], previous_dw[31:24]}; // endian swap
	     data[31:16] <= {previous_dw[ 7: 0], previous_dw[15: 8]};
	     data[47:32] <= {tdata_q[23:16], tdata_q[31:24]};
	     data[63:48] <= {tdata_q[ 7: 0], tdata_q[15: 8]};
	     previous_dw <= tdata_q[63:32];
	     if(wait_dw01)
	       begin
		  is_write_32 <= tdata_q[30:24] == 7'b1000000;
		  is_cpld <= tdata_q[30:24] == 7'b1001010;
		  is_read_32_2dw <= (tdata_q[30:24] == 7'b0000000) && (tdata_q[9:0] == 10'd2);
		  rid_tag <= tdata_q[63:40];
	       end
	     if(wait_dw23)
	       address <= tdata_q[15:3];
	  end
	if(reset || (tvalid_q && tlast_q))
	  {wait_dw45, wait_dw23, wait_dw01} <= 3'b001;
	else if(tvalid_q & wait_dw01)
	  {wait_dw45, wait_dw23, wait_dw01} <= 3'b010;
	else if(tvalid_q && wait_dw23)
	  {wait_dw45, wait_dw23, wait_dw01} <= 3'b100;
	write_valid <= is_write_32 && wait_dw45 && tvalid_q;
	read_valid <= is_read_32_2dw && wait_dw23 && tvalid_q;
	completion_valid <= is_cpld && wait_dw45 && tvalid_q;
     end
endmodule