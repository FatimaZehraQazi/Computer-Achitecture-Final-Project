`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////



module EX_MEM(input clk, 
    input reset,
    input Branch, Zero, MemRead, MemWrite, MemtoReg, RegWrite,
    input [63:0] Adder_Out_2, Result, Write_Data,
    input [4:0] RD,
    output reg EX_MEM_Branch, EX_MEM_Zero, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_MemtoReg, EX_MEM_RegWrite,
    output reg [63:0] EX_MEM_Adder_Out_2, EX_MEM_Result, EX_MEM_Write_Data,
    output reg [4:0] EX_MEM_RD
);

always @(posedge clk or posedge reset)
begin   
    if (reset == 1)         //reset values to 0 if reset initiated
    begin   
        EX_MEM_Branch <= 0;
        EX_MEM_Zero <= 0;
        EX_MEM_MemRead <= 0;
        EX_MEM_MemWrite <= 0;
        EX_MEM_MemtoReg <= 0;
        EX_MEM_RegWrite <= 0;
        EX_MEM_Adder_Out_2 <= 0;
        EX_MEM_Result <= 0;
        EX_MEM_Write_Data <= 0;
        EX_MEM_RD <= 0;
    end
    else                //forward values to register otherwise
    begin
        EX_MEM_Branch <= Branch;
        EX_MEM_Zero <= Zero;
        EX_MEM_MemRead <= MemRead;
        EX_MEM_MemWrite <= MemWrite;
        EX_MEM_MemtoReg <= MemtoReg;
        EX_MEM_RegWrite <= RegWrite;
        EX_MEM_Adder_Out_2 <= Adder_Out_2;
        EX_MEM_Result <= Result;
        EX_MEM_Write_Data <= Write_Data;
        EX_MEM_RD <= RD;
    end
end
endmodule