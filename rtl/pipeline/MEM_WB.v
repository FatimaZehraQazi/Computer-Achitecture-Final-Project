`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////



module MEM_WB(
    input clk, reset,
    input MemtoReg,  RegWrite,
    input [63:0] Read_Data, Result_EX_MEM,
    input [4:0] Rd_EX_MEM,
    output reg MemtoReg_MEM_WB, RegWrite_MEM_WB, 
    output reg [63:0] Read_Data_MEM_WB, Result_MEM_WB,
    output reg [4:0] Rd_MEM_WB
    );
    
    always @(posedge clk or posedge reset) 
    begin
        if (reset == 1)             //reset values to 0 if reset initiated
        begin
            MemtoReg_MEM_WB <= 0; 
            RegWrite_MEM_WB <= 0; 
            Rd_MEM_WB <= 0;
            Read_Data_MEM_WB <= 0;
            Result_MEM_WB <= 0; 
            Rd_MEM_WB <= 0;
        end
        else                //forward values to register otherwise
        begin
            MemtoReg_MEM_WB <= MemtoReg; 
            RegWrite_MEM_WB <= RegWrite; 
            Rd_MEM_WB <= Rd_EX_MEM;
            Read_Data_MEM_WB <= Read_Data;
            Result_MEM_WB <= Result_EX_MEM; 
            Rd_MEM_WB <= Rd_EX_MEM;
        end 
    end
endmodule
