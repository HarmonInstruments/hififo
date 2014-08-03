module sync 
  (
   input  clock,
   input  in,
   output out
   );
   
   (* ASYNC_REG="TRUE", TIG="TRUE" *) reg [2:0] sreg = 0;
   always @(posedge clock)
     sreg <= {sreg[1:0], in};
   assign out = sreg[2];
endmodule