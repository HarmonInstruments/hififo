module fpc_rr_mux
  (
   input 	 clock,
   input 	 reset,
   // PIO for page table writes
   input 	 pio_wvalid,
   input [63:0]  pio_wdata,
   input [12:0]  pio_addr,
   // read request in
   input [3:0] 	 rr_valid,
   output [3:0]  rr_ready,
   input [63:0]  rr0_addr,
   input [63:0]  rr1_addr,
   input [63:0]  rr2_addr,
   input [63:0]  rr3_addr,
   // rr request multiplexed
   output 	 rrm_valid,
   output [63:0] rrm_addr,
   output [7:0]  rrm_tag,
   input 	 rrm_ready
   );

   reg [6:0] 	 pt_addr;
   wire [42:0] 	 pt_data;
   reg [3:0] 	 state;
   reg [11:0] 	 rrm_la;
   
   assign rrm_addr[20:0] = {rrm_la, 9'd0};
   assign rrm_valid = state[1:0] == 3;
   assign rrm_tag[2:0] = rrm_la[2:0];
   assign rrm_tag[7:3] = state[3:2];
   assign rr_ready[0] = rrm_ready && (state == 3);
   assign rr_ready[1] = rrm_ready && (state == 7);
   assign rr_ready[2] = rrm_ready && (state == 11);
   assign rr_ready[3] = rrm_ready && (state == 15);

   always @ (posedge clock)
     begin
	if(reset)
	  state <= 1'b0;
	else
	  case(state[1:0])
	    0: state <= rr_valid[state[3:2]] ? state + 1'b1 : state + 4'd4;
	    1: state <= state + 1'b1;
	    2: state <= state + 1'b1;
	    3: state <= state + rrm_ready;
	  endcase
	case(state)
	  0:  {pt_addr, rrm_la} <= {2'd0, rr0_addr[25:9]};
	  4:  {pt_addr, rrm_la} <= {2'd1, rr1_addr[25:9]};
	  8:  {pt_addr, rrm_la} <= {2'd2, rr2_addr[25:9]};
	  12: {pt_addr, rrm_la} <= {2'd3, rr3_addr[25:9]};
	endcase
	case(state)
	  0:  {pt_addr, rrm_la} <= {2'd0, rr0_addr[25:9]};
	  4:  {pt_addr, rrm_la} <= {2'd1, rr1_addr[25:9]};
	  8:  {pt_addr, rrm_la} <= {2'd2, rr2_addr[25:9]};
	  12: {pt_addr, rrm_la} <= {2'd3, rr3_addr[25:9]};
	endcase
   end
   
   block_ram #(.DBITS(43), .ABITS(7)) bram_pt_fpc
     (.clock(clock),
      .w_data(pio_wdata[63:21]),
      .w_valid(pio_wvalid && (pio_addr[12:7] == 1)),
      .w_addr(pio_addr[6:0]),
      .r_data(rrm_addr[63:21]),
      .r_addr(pt_addr)
      );
   
endmodule