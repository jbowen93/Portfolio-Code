/**********************************
/* Positive edge-triggered N-bit  *
* T flip-flop counter with        *
* enable (En) input, asynchronous *
* (Clr_) and synchronous (ClrS_)  *
* clear, and terminal count       *
* output (TCO).                   *
* 11-22,12, P. Mathys             *
**********************************/

module TffCounter(Clk,En,Clr_,ClrS_,QQ,TCO);
  parameter N = 4; // Number of bits
  input Clk;       // Clock
  input En;        // Enable
  input Clr_;      // Asynchronous clear
  input ClrS_;     // Synchronous clear
  output [N-1:0] QQ;  // Flip-flop state values
  output TCO;      // Terminal (all 1's) count output
  wire [N:0] TT;   // Flip-flop T values
  
  genvar i;
  assign TT[0] = En;
  generate
    for (i=0; i<N; i=i+1) begin: unit
      TffUnit FF(Clk,TT[i],Clr_,ClrS_,TT[i+1],QQ[i]);
    end
  endgenerate
  assign TCO = &QQ;
  
endmodule  // TffCounter