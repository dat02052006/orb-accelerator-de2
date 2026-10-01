`timescale 1ns/1ps
// Signed comparator trees; explicit hardware, no procedural loops.
module orb_min9_signed (
    input wire signed [8:0] a0, a1, a2, a3, a4, a5, a6, a7, a8,
    output wire signed [8:0] result
);
    wire signed [8:0] m01 = (a0 < a1) ? a0 : a1;
    wire signed [8:0] m23 = (a2 < a3) ? a2 : a3;
    wire signed [8:0] m45 = (a4 < a5) ? a4 : a5;
    wire signed [8:0] m67 = (a6 < a7) ? a6 : a7;
    wire signed [8:0] m03 = (m01 < m23) ? m01 : m23;
    wire signed [8:0] m47 = (m45 < m67) ? m45 : m67;
    wire signed [8:0] m07 = (m03 < m47) ? m03 : m47;
    assign result = (m07 < a8) ? m07 : a8;
endmodule

module orb_max4_signed (
    input wire signed [8:0] a0, a1, a2, a3,
    output wire signed [8:0] result
);
    wire signed [8:0] m01 = (a0 > a1) ? a0 : a1;
    wire signed [8:0] m23 = (a2 > a3) ? a2 : a3;
    assign result = (m01 > m23) ? m01 : m23;
endmodule

module orb_max8_signed (
    input wire signed [8:0] a0, a1, a2, a3, a4, a5, a6, a7,
    output wire signed [8:0] result
);
    wire signed [8:0] m01 = (a0 > a1) ? a0 : a1;
    wire signed [8:0] m23 = (a2 > a3) ? a2 : a3;
    wire signed [8:0] m45 = (a4 > a5) ? a4 : a5;
    wire signed [8:0] m67 = (a6 > a7) ? a6 : a7;
    wire signed [8:0] m03 = (m01 > m23) ? m01 : m23;
    wire signed [8:0] m47 = (m45 > m67) ? m45 : m67;
    assign result = (m03 > m47) ? m03 : m47;
endmodule
