`timescale 1ns/1ps
// Six rotating row RAMs. Column byte 0 is y-6; byte 6 is current y.
// advance freezes both RAM reads and the associated metadata on output stall.
module orb_line_buffer #(
    parameter MAX_WIDTH = 640,
    parameter X_BITS = 10,
    parameter Y_BITS = 9
) (
    input wire clk,
    input wire reset,
    input wire clear,
    input wire advance,
    input wire pixel_fire,
    input wire [7:0] pixel_data,
    input wire [X_BITS-1:0] pixel_x,
    input wire [Y_BITS-1:0] pixel_y,
    input wire [2:0] write_bank,
    output reg column_valid,
    output reg [X_BITS-1:0] column_x,
    output reg [Y_BITS-1:0] column_y,
    output reg [55:0] column_data
);
    wire [47:0] row_reads;
    reg [2:0] column_bank;
    reg [7:0] current_pixel;
    orb_row_ram #(.MAX_WIDTH(MAX_WIDTH), .X_BITS(X_BITS)) row_ram_0 (
        .clk(clk), .enable(pixel_fire),
        .write_enable(write_bank == 3'd0),
        .address(pixel_x), .write_data(pixel_data),
        .read_data(row_reads[7:0])
    );

    orb_row_ram #(.MAX_WIDTH(MAX_WIDTH), .X_BITS(X_BITS)) row_ram_1 (
        .clk(clk), .enable(pixel_fire),
        .write_enable(write_bank == 3'd1),
        .address(pixel_x), .write_data(pixel_data),
        .read_data(row_reads[15:8])
    );

    orb_row_ram #(.MAX_WIDTH(MAX_WIDTH), .X_BITS(X_BITS)) row_ram_2 (
        .clk(clk), .enable(pixel_fire),
        .write_enable(write_bank == 3'd2),
        .address(pixel_x), .write_data(pixel_data),
        .read_data(row_reads[23:16])
    );

    orb_row_ram #(.MAX_WIDTH(MAX_WIDTH), .X_BITS(X_BITS)) row_ram_3 (
        .clk(clk), .enable(pixel_fire),
        .write_enable(write_bank == 3'd3),
        .address(pixel_x), .write_data(pixel_data),
        .read_data(row_reads[31:24])
    );

    orb_row_ram #(.MAX_WIDTH(MAX_WIDTH), .X_BITS(X_BITS)) row_ram_4 (
        .clk(clk), .enable(pixel_fire),
        .write_enable(write_bank == 3'd4),
        .address(pixel_x), .write_data(pixel_data),
        .read_data(row_reads[39:32])
    );

    orb_row_ram #(.MAX_WIDTH(MAX_WIDTH), .X_BITS(X_BITS)) row_ram_5 (
        .clk(clk), .enable(pixel_fire),
        .write_enable(write_bank == 3'd5),
        .address(pixel_x), .write_data(pixel_data),
        .read_data(row_reads[47:40])
    );

    always @(posedge clk) begin
        if (reset || clear) begin
            column_valid <= 1'b0;
            column_x <= 0;
            column_y <= 0;
            column_bank <= 0;
            current_pixel <= 0;
        end else if (advance) begin
            column_valid <= pixel_fire;
            if (pixel_fire) begin
                column_x <= pixel_x;
                column_y <= pixel_y;
                column_bank <= write_bank;
                current_pixel <= pixel_data;
            end
        end
    end

    always @(*) begin
        case (column_bank)
            3'd0: column_data = {current_pixel, row_reads[47:40], row_reads[39:32], row_reads[31:24], row_reads[23:16], row_reads[15:8], row_reads[7:0]};
            3'd1: column_data = {current_pixel, row_reads[7:0], row_reads[47:40], row_reads[39:32], row_reads[31:24], row_reads[23:16], row_reads[15:8]};
            3'd2: column_data = {current_pixel, row_reads[15:8], row_reads[7:0], row_reads[47:40], row_reads[39:32], row_reads[31:24], row_reads[23:16]};
            3'd3: column_data = {current_pixel, row_reads[23:16], row_reads[15:8], row_reads[7:0], row_reads[47:40], row_reads[39:32], row_reads[31:24]};
            3'd4: column_data = {current_pixel, row_reads[31:24], row_reads[23:16], row_reads[15:8], row_reads[7:0], row_reads[47:40], row_reads[39:32]};
            3'd5: column_data = {current_pixel, row_reads[39:32], row_reads[31:24], row_reads[23:16], row_reads[15:8], row_reads[7:0], row_reads[47:40]};
            default: column_data = 56'd0;
        endcase
    end
endmodule
