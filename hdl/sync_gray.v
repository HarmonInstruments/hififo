module gray_sync_3
  (
   input 	    clock_in,
   input [2:0] 	    in,
   input 	    clock_out,
   output reg [2:0] out
   );
   reg [2:0] 	gray_inclock = 0;
   always @ (posedge clock_in)
     begin
	case(in)
	  0: gray_inclock <= 3'b000;
	  1: gray_inclock <= 3'b001;
	  2: gray_inclock <= 3'b011;
	  3: gray_inclock <= 3'b010;
	  4: gray_inclock <= 3'b110;
    	  5: gray_inclock <= 3'b111;
	  6: gray_inclock <= 3'b101;
	  7: gray_inclock <= 3'b100;
	endcase
     end
   
   (* ASYNC_REG="TRUE", TIG="TRUE" *) reg [2:0] gray_outclock0 = 0;
   (* ASYNC_REG="TRUE", TIG="TRUE" *) reg [2:0] gray_outclock1 = 0;
   (* ASYNC_REG="TRUE", TIG="TRUE" *) reg [2:0] gray_outclock2 = 0;
   
   always @ (posedge clock_out)
     begin
	gray_outclock0 <= gray_inclock;
	gray_outclock1 <= gray_outclock0;
	gray_outclock2 <= gray_outclock1;
	case(gray_outclock2)
	  3'b000: out <= 3'd0;
	  3'b001: out <= 3'd1;
	  3'b011: out <= 3'd2;
	  3'b010: out <= 3'd3;
	  3'b110: out <= 3'd4;
	  3'b111: out <= 3'd5;
	  3'b101: out <= 3'd6;
	  3'b100: out <= 3'd7;
	endcase
     end
endmodule