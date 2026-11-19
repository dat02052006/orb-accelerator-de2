`timescale 1ns / 1ps

// ============================================================================
// 1. Mo hinh hanh vi chip SRAM bat dong bo (256K x 16-bit)
// ============================================================================
module sram_model (
    input  wire [17:0] addr,
    inout  wire [15:0] dq,
    input  wire        ce_n,
    input  wire        oe_n,
    input  wire        we_n,
    input  wire        ub_n,
    input  wire        lb_n
);
    // Mang nho 256K word 16-bit
    reg [15:0] mem [0:262143];

    // Lai bus DQ khi chip duoc chon, cho phep doc (oe_n=0) va khong ghi (we_n=1)
    assign dq = (!ce_n && !oe_n && we_n) ? mem[addr] : 16'hzzzz;

    // Ghi du lieu tai suon len cua we_n hoac ce_n (chuan timing SRAM bat dong bo)
    always @(posedge we_n or posedge ce_n) begin
        if (!ub_n && !lb_n) begin
            mem[addr] <= dq;
        end
    end
endmodule