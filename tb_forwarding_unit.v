`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 22.04.2024 12:16:41
// Design Name: 
// Module Name: tb_forwarding_unit
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


module tb_forwarding_unit();

reg [4:0] ID_EX_RS1, ID_EX_RS2, EX_MEM_RD, MEM_WB_RD;
reg EX_MEM_RegWrite, MEM_WB_RegWrite;
wire [1:0] ForwardA, ForwardB;

Forwarding_Unit fu(ID_EX_RS1, ID_EX_RS2, EX_MEM_RD, MEM_WB_RD,
 EX_MEM_RegWrite, MEM_WB_RegWrite,
 ForwardA, ForwardB);

initial begin

ID_EX_RS1 = 4'b0010;
ID_EX_RS2 = 4'b0010;
EX_MEM_RD = 4'b1000;
MEM_WB_RD = 4'b1000;
EX_MEM_RegWrite = 1;
MEM_WB_RegWrite = 1;

#10
EX_MEM_RD = 4'b0010;

#10
EX_MEM_RD = 4'b1000;
MEM_WB_RD = 4'b0010;


end

endmodule



