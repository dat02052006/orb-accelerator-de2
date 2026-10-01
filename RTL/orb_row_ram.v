`timescale 1ns/1ps
// Verilog-2001. One synchronous read/write port, OLD DATA on same-address write.
// No RAM reset: suppress window_valid until six fresh rows have arrived.
module orb_row_ram #(
    parameter MAX_WIDTH = 640,
    parameter X_BITS = 10
) (
    input wire clk,
    input wire enable,
    input wire write_enable,
    input wire [X_BITS-1:0] address,
    input wire [7:0] write_data,
    output reg [7:0] read_data
);
    reg [7:0] memory [0:MAX_WIDTH-1];
    always @(posedge clk) begin
        if (enable) begin
            read_data <= memory[address];
            if (write_enable)
                memory[address] <= write_data;
        end
    end
endmodule
