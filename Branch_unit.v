`timescale 1ns / 1ps

module Branch_unit(
   input [2:0] Funct3,
   input [63:0] ReadData1,
   input [63:0] ReadData2,
   output reg addermuxselect
//   output reg stall
  );
  
  initial
    begin
      addermuxselect = 1'b0;
    end
  
  always @(*)
	begin
      case (Funct3)
        3'b000:
          begin
            if (ReadData1 == ReadData2)
              addermuxselect <= 1'b1;
            else
              addermuxselect <= 1'b0;
            end
         3'b100:
    		begin
              if (ReadData1 < ReadData2)
              addermuxselect <= 1'b1;
            else
              addermuxselect <= 1'b0;
            end
        3'b101:
          begin
            if (ReadData1 > ReadData2)
          	addermuxselect <= 1'b1;
           else
              addermuxselect <= 1'b0;
          end    
      endcase
     end
//  always @(*)
//  begin
//    if (addermuxselect)
//        stall <= 1'b1;
//    else
//        stall <= 1'b0;
//  end
endmodule
