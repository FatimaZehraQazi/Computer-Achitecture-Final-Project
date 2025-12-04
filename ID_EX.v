`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////



module ID_EX(input clk, 
    input reset,
    input Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite,
    input [1:0] ALUOp,
    input [63:0] PC_Out, ReadData1, ReadData2, Imm_Data,
    input [4:0] RS1, RS2, RD,
    input [3:0] Funct,
    input [2:0] Funct3,
    output reg ID_EX_Branch, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_MemtoReg, ID_EX_ALUSrc, ID_EX_RegWrite,
    output reg [1:0] ID_EX_ALUOp,
    output reg [63:0] ID_EX_PC_Out, ID_EX_ReadData1, ID_EX_ReadData2, ID_EX_Imm_Data,
    output reg [4:0] ID_EX_RS1, ID_EX_RS2, ID_EX_RD,
    output reg [3:0] ID_EX_Funct,
    output reg [2:0] ID_EX_Funct3
);

always @(posedge clk or posedge reset)
begin
    if (reset == 1)         //reset values to 0 if reset initiated
    begin
        ID_EX_Branch <= 0;
        ID_EX_MemRead <= 0;
        ID_EX_MemWrite <= 0;
        ID_EX_MemtoReg <= 0;
        ID_EX_ALUSrc <= 0;
        ID_EX_RegWrite <= 0;
        ID_EX_ALUOp <= 0;
        ID_EX_PC_Out <= 0;
        ID_EX_ReadData1 <= 0;
        ID_EX_ReadData2 <= 0;
        ID_EX_Imm_Data <= 0;
        ID_EX_RS1 <= 0;
        ID_EX_RS2 <= 0;
        ID_EX_RD <= 0;
        ID_EX_Funct <= 0;
        ID_EX_Funct3 <= 0;
        
    end
    else                //forward values to register otherwise
    begin
        ID_EX_Branch <= Branch;
        ID_EX_MemRead <= MemRead;
        ID_EX_MemWrite <= MemWrite;
        ID_EX_MemtoReg <= MemtoReg;
        ID_EX_ALUSrc <= ALUSrc;
        ID_EX_RegWrite <= RegWrite;
        ID_EX_ALUOp <= ALUOp;
        ID_EX_PC_Out <= PC_Out;
        ID_EX_ReadData1 <= ReadData1;
        ID_EX_ReadData2 <= ReadData2;
        ID_EX_Imm_Data <= Imm_Data;
        ID_EX_RS1 <= RS1;
        ID_EX_RS2 <= RS2;
        ID_EX_RD <= RD;
        ID_EX_Funct <= Funct;
        ID_EX_Funct3 <= Funct3;
    end
end
endmodule