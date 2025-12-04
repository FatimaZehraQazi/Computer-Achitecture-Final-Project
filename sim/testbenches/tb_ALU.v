`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/21/2024 03:49:42 PM
// Design Name: 
// Module Name: tb_ALU
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


module tb_ALU();

reg [63:0] A;
reg [63:0] B;
reg [3:0] ALUOp;
wire Zero;
wire [63:0] Result;

ALU64bit ALU(A,
    B,
    ALUOp,
    Zero,
    Result
);

initial 
begin
    A = 0;
    B = 0;
    ALUOp = 0;
    
#100 A = 1;
#100 B = 1;

end
endmodule
