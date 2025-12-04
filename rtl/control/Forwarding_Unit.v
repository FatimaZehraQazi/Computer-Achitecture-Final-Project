`timescale 1ns / 1ps

module Forwarding_Unit(
    input [4:0] ID_EX_RS1, ID_EX_RS2, EX_MEM_RD, MEM_WB_RD,
    input EX_MEM_RegWrite, MEM_WB_RegWrite,
    output reg [1:0] ForwardA, ForwardB
    );

always @(*)
begin
    ForwardA[1] <= ((EX_MEM_RegWrite == 1) && (EX_MEM_RD != 0) && (EX_MEM_RD == ID_EX_RS1)) ? 1 : 0;
    ForwardB[1] <= ((EX_MEM_RegWrite == 1) && (EX_MEM_RD != 0) && (EX_MEM_RD == ID_EX_RS2)) ? 1 : 0;
    
    ForwardA[0] <= ((MEM_WB_RegWrite == 1) && (MEM_WB_RD != 0) && (MEM_WB_RD == ID_EX_RS1)) ? 1 : 0;
    ForwardB[0] <= ((MEM_WB_RegWrite == 1) && (MEM_WB_RD != 0) && (MEM_WB_RD == ID_EX_RS2)) ? 1 : 0;
end
endmodule
