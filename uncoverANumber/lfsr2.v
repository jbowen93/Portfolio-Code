module LFSR2(Clk,k,QQ,Set_,SS);
parameter N = 10;
input Clk; // Clock
input [N-1:0] k; // Second tap for feedback
output reg [N:1] QQ; // State of LFSR
input Set_; // Set with contents of SS
input [N-1:0] SS; // Data for Set_
reg D;

initial
 begin
    QQ[N:1]= 10'b1010101010;
    D = 1'b1;
  end
  
  
always @ (posedge Clk)
begin 
  QQ[N:1] = {QQ[N-1:1], D};
  D = QQ[N]^QQ[k];
end

endmodule