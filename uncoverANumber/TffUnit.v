module TffUnit(Clk,T,Clr_,ClrS_,Tout,Q);
  input Clk,T;   // Clock and T inputs
  input Clr_;    // Asynchronous clear
  input ClrS_;   // Synchonous clear
  output Tout;   // T output for next stage
  output reg Q;
  
  always @(posedge Clk, negedge Clr_)
    if (!Clr_) Q <= 1'b0;
    else if (!ClrS_) Q <= 1'b0;
	 else Q <= Q&~T|~Q&T;
  assign Tout = T&Q;
    
endmodule  // TffUnit