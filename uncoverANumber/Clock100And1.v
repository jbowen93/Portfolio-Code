/***************************
* Generate 100 Hz and 1 Hz *
* clocks from CLOCK_50 of  *
* DE0 board                *
* 11-25-12, P. Mathys      *
***************************/

module Clock100And1(ClkIn,En,En1,Clr_,Clk100,Clk1);
  input ClkIn;    // 50 MHz clock input
  input En;       // Enable clock generation
  input En1;      // Enable 1 Hz generation
  input Clr_;     // Clear all registers
  output Clk100;  // 100 Hz clock
  output Clk1;    // 1 Hz clock
  parameter N1 = 5;    // Prescaler, divide by 32
  wire [N1-1:0] QQ1;
  wire TCO1;           // Terminal count 1
  reg TCO1S;           // TCO1 synchronized
  parameter N2 = 14;   // Divide by 15625 counter
  parameter M2 = 15625;
  wire [N2-1:0] QQ2;
  wire CM2;            // M2-1 count detect
  reg CM2S;            // Synchronized CM2
  wire TCO2;
  parameter N3 = 7;    // Divide by 100 counter
  parameter M3 = 100;
  wire [N3-1:0] QQ3;
  wire CM3;            // M3-1 count detect
  reg CM3S;            // Synchronized CM3
  wire TCO3;
  
  TffCounter #(.N(N1)) div32(ClkIn,En,Clr_,1'b1,QQ1,TCO1);
  always @(negedge ClkIn)
    if (TCO1) TCO1S <= 1'b1;
	 else TCO1S <= 1'b0;
  TffCounter #(.N(N2)) div15625(TCO1S,En,Clr_,~CM2,QQ2,TCO2);
  assign CM2 = (QQ2==M2-1) ? 1'b1 : 1'b0;
  always @(negedge TCO1S)
	 if (CM2) CM2S <= 1'b1;
	 else CM2S <= 1'b0;
  assign Clk100 = CM2S;
  TffCounter #(.N(N3)) div100(Clk100,En&En1,Clr_,~CM3,QQ3,TCO3);
  assign CM3 = (QQ3==M3-1) ? 1'b1 : 1'b0;
  always @(negedge Clk100)
    if (CM3) CM3S <= 1'b1;
	 else CM3S <= 1'b0;
  assign Clk1 = CM3S;	 
  
  
      
endmodule  // Clock100And1
