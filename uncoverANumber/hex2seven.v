module hex2seven(SevenS,x,BI_,LT_);
  input [3:0] x;
  input BI_, LT_;
  output [6:0]SevenS;
  
  reg [6:0] SevenS;

  
  always @ *
  begin
    
  if(~BI_ == 1'b1) // BI_ = 0
      SevenS = 7'b1111111;
  else // BI_ = 1
    if(~LT_ == 1'b1) // LT_ = 0
      SevenS = 7'b0000000;
    else //LT_ = 1
      
    if(x == 4'b0000)
      SevenS = 7'b1000000;
    if(x == 4'b0001)
      SevenS = 7'b1111001;
    if(x == 4'b0010)
      SevenS = 7'b0100100;
    if(x == 4'b0011)
      SevenS = 7'b0110000;
    if(x == 4'b0100)
      SevenS = 7'b0011001;
    if(x == 4'b0101)
      SevenS = 7'b0010010;
    if(x == 4'b0110)
      SevenS = 7'b0000010;
    if(x == 4'b0111)
      SevenS = 7'b1111000;
    if(x == 4'b1000)
      SevenS = 7'b0000000;
    if(x == 4'b1001)
      SevenS = 7'b0011000;
    if(x == 4'b1010)
      SevenS = 7'b0001000;
    if(x == 4'b1011)
      SevenS = 7'b0000011;
    if(x == 4'b1100)
      SevenS = 7'b1000110;
    if(x == 4'b1101)
      SevenS = 7'b0100001;
    if(x == 4'b1110)
      SevenS = 7'b0000110;
    if(x == 4'b1111)
      SevenS = 7'b0001110;
  
    end
 endmodule


