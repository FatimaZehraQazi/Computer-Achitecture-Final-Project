`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////


module Hazard_Detection(
    input [4:0] RS1, RS2, Rd_ID_EX,
    input MemRead_ID_EX,
    output reg stall 
    );

always @(*) 
    begin
        if (MemRead_ID_EX && (Rd_ID_EX == RS1 || Rd_ID_EX == RS2)) 
        begin
            stall <= 1;  
        end
        else 
        begin
            stall <= 0; 
        end
    end
endmodule
