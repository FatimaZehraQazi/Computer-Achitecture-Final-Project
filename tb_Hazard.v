`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/22/2024 02:49:45 PM
// Design Name: 
// Module Name: tb_Hazard
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module tbHazard();

reg clk, reset; 
wire [63:0] PC_In, PC_Out, adder_out1, adder_out2, imm_data, WriteData, ReadData1, ReadData2, Result, Read_Data;
wire [63:0] muxmid_out;
wire [31:0] Instruction;
wire [6:0] opcode, funct7;
wire [4:0] rd, rs1, rs2;
wire [3:0] Funct, Operation;
wire [2:0] funct3;
wire [1:0] ALUOp;
wire Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite, Zero, addermuxselect, branch_sel, stall;
wire [63:0] index0, index1, index2, index3, index4;


wire [31:0] IF_ID_Instruction;
wire [63:0] IF_ID_PC_Out;

wire ID_EX_Branch, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_MemtoReg, ID_EX_ALUSrc, ID_EX_RegWrite;
wire [1:0] ID_EX_ALUOp;
wire [63:0] ID_EX_PC_Out, ID_EX_ReadData1, ID_EX_ReadData2, ID_EX_Imm_Data;
wire [4:0] ID_EX_RS1, ID_EX_RS2, ID_EX_RD;
wire [3:0] ID_EX_Funct;
wire EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_MemtoReg, EX_MEM_RegWrite;
wire [63:0] EX_MEM_Adder_Out_2, EX_MEM_Result, EX_MEM_Write_Data;
wire [4:0] EX_MEM_RD;
wire MEM_WB_MemtoReg, MEM_WB_RegWrite;
wire [63:0] MEM_WB_Read_Data, MEM_WB_Result;
wire [4:0] MEM_WB_RD;

/*

module pipelinedProcessor(input clk,
    input reset,
    output reg [63:0] PC_In, PC_Out, ReadData1, ReadData2, WriteData, Result, Read_Data, imm_data,
    output reg [31:0] Instruction,
    output reg [6:0] opcode,
    output reg [4:0] rs1, rs2, rd,
    output reg [1:0] ALUOp,
    output reg [63:0] adder_out1, adder_out2,
    output reg Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite, Zero,
    output reg [63:0] index0, index1, index2, index3, index4
    );
*/

hazardProcessor HP(
     clk, reset,
     PC_In, PC_Out, ReadData1, ReadData2, WriteData, Result, Read_Data, imm_data,
     Instruction,
     opcode,
     rs1, rs2, rd,
     ALUOp,
     adder_out1, adder_out2,
     Branch, MemRead, MemWrite, MemtoReg, ALUSrc, RegWrite, Zero, stall,
     index0, index1, index2, index3, index4
    );

initial begin
clk = 0; reset = 1'b1;
#5 reset = 0;
end

always
    #5 clk = ~clk;

endmodule
