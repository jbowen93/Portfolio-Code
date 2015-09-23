module uncoverANumber(ClkIn, LEDS, BUT0, BUT1, BUT2, SevenS0, SevenS1, dec1, SevenS2, SevenS3);

	output reg dec1;
	input ClkIn;    // 50 MHz clock input
	input BUT0;     // Clear all registers
	input BUT1;
	input BUT2;
	
	reg gamefinished;
	reg [2:0]endGameDisplay;
	
	wire Clk100;  // 100 Hz clock
	wire Clk1;    // 1 Hz clock
	
	wire [9:0] random; 	// random number gereated
	wire [11:0] Binary2Decimal;
	
	reg errorflag;
	reg [3:0]errorcount;
	reg [3:0]errortens;
	reg [3:0]errorhundreds;
	wire[6:0] errorhex0, errorhex1, errorhex2;
	
	reg [3:0] time100s, time10s, time1s, time10ths, time100ths;
	wire[6:0] timehex0, timehex1, timehex2, timehex3;
	
	integer BUT0set;
	integer BUT1set; 
	integer BUT2set;
	reg pause;
	
	output reg[9:0] LEDS;
	output reg [6:0] SevenS0;
	output reg [6:0] SevenS1;
	output reg [6:0] SevenS2;
	output reg [6:0] SevenS3;
	
	reg [6:0] FoundS0;
	reg [6:0] FoundS1;
	reg [6:0] FoundS2;
	
	integer SS;
	reg [3:0] countS0;
	reg [3:0] countS1;
	reg [3:0] countS2;
	reg [3:0] countTemp;
	
   wire [6:0] bcdONES;
	wire [6:0] bcdTENS;
	wire [6:0] bcdHUNDREDS;
	
	
	initial begin
		
		dec1 = 1;
		gamefinished = 1;
		endGameDisplay = 0;
		errorcount = 0;
		// number = 0
		countS0 = 0;
		countS1 = 0;
		countS2 = 0;
		SS = 0;
		
		pause = 1;		
		BUT0set = 1;
		BUT1set = 1;
		BUT2set = 1;
		
		LEDS = 10'b1111111111;
		
		SevenS0 = 7'b1111111;
		SevenS1 = 7'b1111111;
		SevenS2 = 7'b1111111;
		SevenS3 = 7'b1111111;
		
		FoundS0 = 7'b1111111;
		FoundS1 = 7'b1111111;
		FoundS2 = 7'b1111111;
		
	end
	
	Clock100And1 m1(ClkIn, 1,1, 1, Clk100, Clk1);
	LFSR2 rand(BUT2,4'b0111, random, 1'b1,4'b1000);
   binary_to_BCD converter(random,Binary2Decimal);
	hex2seven ones(bcdONES,Binary2Decimal[3:0],0,0); 
	hex2seven tens(bcdTENS,Binary2Decimal[7:4],0,0); 
	hex2seven hundreds(bcdHUNDREDS,Binary2Decimal[11:8],0,0);
	hex2seven error1s(errorhex0,errorcount,0,0); 
	hex2seven error10s(errorhex1,errortens,0,0); 
	hex2seven error100s(errorhex2,errorhundreds,0,0);
	hex2seven time10thss(timehex0,time10ths,0,0); 
	hex2seven time1ss(timehex1,time1s,0,0); 
	hex2seven time10ss(timehex2,time10s,0,0);
	hex2seven time100ss(timehex3,time100s,0,0);
	
	
	// function nextCount
	// input: vector of size 7. 1 if the segment hasn't been found, 0 if it has
	//        current count
	// output: new count that finds the next 1 in the list from the current count.  
	function [3:0] nextCount;
		input [6:0] foundList;
		input [3:0] count;
		
		reg [3:0] tempCount;
		integer i;
		integer found = 1; 
		
		begin
		
			tempCount = count;
			for (i = 0; i < 7; i = i + 1) begin
				if (found == 1) begin
				
					tempCount = tempCount + 1;
					if (tempCount == 7) begin
						tempCount = 0;
					end	
					
					if (foundList[tempCount] == 1) begin
						nextCount = tempCount;
						found = 0;
					end
					
				end
			end
		end
		
	endfunction
	
	function [3:0] lastCount;
		input [6:0] foundList;
		input [3:0] count;
		
		reg [3:0] tempCount;
		integer i;
		integer found = 1; 
		
		begin
		
			tempCount = count + 1;
			for (i = 7; i > 0; i = i -1) begin
				if (found == 1) begin
				
					tempCount = tempCount - 1;
					if (tempCount == 0) begin
						tempCount = 7;
					end	
					
					if (foundList[tempCount - 1] == 1) begin
						lastCount = tempCount - 1;
						found = 0;
					end
					
				end
			end
		end
		
	endfunction
	
	always @ (posedge Clk100) begin
	if(gamefinished == 1)begin
	time100ths = time100ths + 1'b1;
	
	if(time100s == 4'b1001)begin 
time100s = 0;
end 
if(time10s == 4'b1001) begin 
    time10s = 0; 
    time100s = time100s + 1'b1; 
  end 
if(time1s == 4'b1001) begin 
  time1s = 0; 
  time10s = time10s + 1'b1; 
end 
if(time10ths == 4'b1001) begin 
    time10ths = 0; 
    time1s = time1s +1'b1; 
  end 
  if(time100ths == 4'b1001)begin
  time100ths = 0;
  time10ths = time10ths + 1'b1;
  end

end
 

	
		// Pressing button 0
		if (!BUT0 & BUT0set == 1) begin
			//LEDS[9:0] = QQ;
			
			BUT0set = 0;
			
		
			case (SS)
				0: begin
						endGameDisplay = 0;
						errorflag = 1;
						gamefinished = 1;
						time100s = 0;
						time10s = 0;
						time1s = 0;
						time10ths = 0;
						time100ths = 0;
						SS = 1;
						 errorcount = 0;
						
						FoundS0 = 7'b1111111;
						FoundS1 = 7'b1111111;
						FoundS2 = 7'b1111111;
						LEDS[9:0] = random;  // generate a new number
						//bcdONES = hexones;
						//bcdTENS = hextens;
						//bcdHUNDREDS = hexhundreds;
						
					end
				1: SS = 2;
				2: SS = 3;
				3: SS = 1; // added case 4 for comparison, normally 3: SS = 1
				4: begin
					case(endGameDisplay)
					0: endGameDisplay = 1;
					1: endGameDisplay = 2;
					2: endGameDisplay = 0;
					endcase
					end
			endcase
		end else if (BUT0 & BUT0set == 0) begin
			BUT0set = 1;
			
		// Pressing button 1
		end else if (!BUT1 & BUT1set == 1) begin
			errorflag = 1;
			pause = 0;			
			BUT1set = 0;
			//LEDS[0] = ~LEDS[0];
			
			case (SS)
				1: begin
						countTemp = lastCount(FoundS0, countS0);
						
						if(bcdONES[countTemp] == 0) begin
							FoundS0[countTemp] = 0;
						end else begin
						 errorflag = 0;
						end
					end
				2: begin
						countTemp = lastCount(FoundS1, countS1);
						
						if(bcdTENS[countTemp] == 0) begin
							FoundS1[countTemp] = 0;
						end else begin
							errorflag = 0;
						end
					end
				3: begin
						countTemp = lastCount(FoundS2, countS2);
						
						if(bcdHUNDREDS[countTemp] == 0) begin
							FoundS2[countTemp] = 0;
						end else begin
							errorflag = 0;
						end
					end
				
			
						  
					
					
			endcase
			 if(errorflag == 0)begin
				errorcount = errorcount + 1'b1;
			  if(errorcount == 4'b1001)begin
			  errorcount = 0;
			  errortens = errortens + 1'b1;
			  end
			  if(errortens == 4'b1001)begin
			  errortens = 0;
			  errorhundreds = errorhundreds + 1'b1;
				end
			  if(errorhundreds == 4'b1001)begin
			  errorhundreds = 0; //give up
  end
  errorflag = 1;
  end
  
			
		end else if (BUT1 & BUT1set == 0) begin
			pause = 1;
			BUT1set = 1;
			if (FoundS0 == bcdONES & FoundS1 == bcdTENS & FoundS2 == bcdHUNDREDS) begin
				SS = 4;
				gamefinished = 0;
			end
			//LEDS[0] = ~LEDS[0];
			
		// Pressing button 2
		end else if(!BUT2 & BUT2set == 1) begin
			BUT2set = 0;
			LEDS = 0;
			SS = 0;
			//LEDS[1] = ~LEDS[1];
		end else if (BUT2 & BUT2set == 0) begin
			BUT2set = 1;
		end
		
	end
	
	
	always @ (posedge Clk1) begin
		if (pause == 1) begin
			case (SS)
				0: 
					begin
						SevenS0 = 7'b1111111;
						SevenS1 = 7'b1111111;
						SevenS2 = 7'b1111111;
						
						countS0 = 0;
						countS1 = 0;
						countS2 = 0;
						dec1 = 1;
					end // running == 1
				1: 
					begin
						SevenS1 = FoundS1;
						SevenS2 = FoundS2;
						case (countS0)
							0: SevenS0 = 7'b1111110&FoundS0;
							1: SevenS0 = 7'b1111101&FoundS0;
							2: SevenS0 = 7'b1111011&FoundS0;
							3: SevenS0 = 7'b1110111&FoundS0;
							4: SevenS0 = 7'b1101111&FoundS0;
							5: SevenS0 = 7'b1011111&FoundS0;
							6: SevenS0 = 7'b0111111&FoundS0;
						endcase // count
						countS0 = nextCount(FoundS0, countS0);
					end
				2:
					begin
						SevenS0 = FoundS0;
						SevenS2 = FoundS2;
						case (countS1)
							0: SevenS1 = 7'b1111110&FoundS1;
							1: SevenS1 = 7'b1111101&FoundS1;
							2: SevenS1 = 7'b1111011&FoundS1;
							3: SevenS1 = 7'b1110111&FoundS1;
							4: SevenS1 = 7'b1101111&FoundS1;
							5: SevenS1 = 7'b1011111&FoundS1;
							6: SevenS1 = 7'b0111111&FoundS1;
						endcase // count
						countS1 = nextCount(FoundS1, countS1);
					end
				3:
					begin
						SevenS0 = FoundS0;
						SevenS1 = FoundS1;
						case (countS2)
							0: SevenS2 = 7'b1111110&FoundS2;
							1: SevenS2 = 7'b1111101&FoundS2;
							2: SevenS2 = 7'b1111011&FoundS2;
							3: SevenS2 = 7'b1110111&FoundS2;
							4: SevenS2 = 7'b1101111&FoundS2;
							5: SevenS2 = 7'b1011111&FoundS2;
							6: SevenS2 = 7'b0111111&FoundS2;
						endcase // case(count)
						countS2 = nextCount(FoundS2, countS2);

					end
				4: begin
						
						case (endGameDisplay)
							0: begin
									// Error Count
									SevenS0 = errorhex0;
									SevenS1 = errorhex1;
									SevenS2 = errorhex2;
									dec1 = 1;
								end
							1: begin
										// Random Number
									SevenS0 = bcdONES;
									SevenS1 = bcdTENS;
									SevenS2 = bcdHUNDREDS;
									dec1 = 1;
								end
							2: begin
									// Time elapsed
									SevenS0 = timehex0;
									SevenS1 = timehex1;
									SevenS2 = timehex2;
									dec1 = 0;
									//SevenS3 = timehex3;
								end
						endcase
					end
				
				
			endcase // case(SS)
		end
	
						
	//end // always
	
		
		
	
  end // end always

		
endmodule
