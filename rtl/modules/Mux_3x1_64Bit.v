`timescale 1ns / 1ps

module Mux_3x1(
    input [63:0] A, B, C,
    input [1:0] sel,
    output [63:0] O
    );
    assign O = (sel[1] == 1) ? C : ((sel[0] == 1) ? B : A);
endmodule
