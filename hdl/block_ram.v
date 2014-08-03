module block_ram
  (
   input 		  clock,
   input [DBITS-1:0] 	  w_data,
   input 		  w_valid,
   input [ABITS-1:0] 	  w_addr,
   output reg [DBITS-1:0] r_data,
   input [ABITS-1:0] 	  r_addr
   );
   parameter ABITS = 9;
   parameter DBITS = 64;
   reg [DBITS-1:0] 	  bram[0:2**ABITS-1];
   reg [DBITS-1:0] 	  bram_oreg;
   always @ (posedge clock)
     begin
	if(w_valid)
	  bram[w_addr] <= w_data;
	bram_oreg <= bram[r_addr];
	r_data <= bram_oreg;
     end
endmodule
