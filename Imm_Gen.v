`timescale 1ns / 1ps

module Imm_Gen(
    input [31:0] Instruction,
    output reg [63:0] Imm
    );
    
reg [11:0] ld_imm;
reg [11:0] sd_imm;
reg [11:0] br_imm;
wire [1:0]	sel;

assign sel = Instruction[6:5];

localparam [1:0]
ld_ins 	= 2'b00,
sd_ins 	= 2'b01,
br_ins 	= 2'b11;

always @ (Instruction)
begin
	case (sel)
		ld_ins:
			begin
			ld_imm = Instruction[31:20];
			Imm = {{52{Instruction[31]}}, ld_imm};
			end
		
		sd_ins:
			begin
			sd_imm = {Instruction[31:25], Instruction[11:7]};
			Imm = {{52{Instruction[31]}}, sd_imm};
			end
			
		br_ins:
			begin
			br_imm = {Instruction[31], Instruction[7], Instruction[30:25], Instruction[11:8]};
			Imm = {{52{Instruction[31]}}, br_imm};
			end
			
		default: Imm = 64'h0;
	endcase
end
endmodule

