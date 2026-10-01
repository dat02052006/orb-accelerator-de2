`timescale 1ns/1ps
// {is_corner, score}: nine bits per position, synchronous OLD DATA read.
module orb_nms_row_ram #(
    parameter MAX_WIDTH = 634,
    parameter X_BITS = 10
) (
    input wire clk,
    input wire enable,
    input wire write_enable,
    input wire [X_BITS-1:0] address,
    input wire [8:0] write_data,
    output reg [8:0] read_data
);
    reg [8:0] memory [0:MAX_WIDTH-1];
    always @(posedge clk) begin
        if (enable) begin
            read_data <= memory[address];
            if (write_enable)
                memory[address] <= write_data;
        end
    end
endmodule
