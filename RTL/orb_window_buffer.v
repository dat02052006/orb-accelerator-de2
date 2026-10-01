`timescale 1ns/1ps
// Keep the previous six columns. Output byte index = row*7 + column.
module orb_window_buffer #(
    parameter X_BITS = 10,
    parameter Y_BITS = 9
) (
    input wire clk,
    input wire reset,
    input wire clear,
    input wire advance,
    input wire column_valid,
    input wire [55:0] column_data,
    input wire [X_BITS-1:0] column_x,
    input wire [Y_BITS-1:0] column_y,
    output reg window_valid,
    output reg [391:0] window_data,
    output reg [X_BITS-1:0] center_x,
    output reg [Y_BITS-1:0] center_y
);
    // Each row is six 8-bit shift-register stages packed into 48 bits.
    // Oldest column occupies [7:0]; newest occupies [47:40].
    reg [47:0] history_row_0;
    reg [47:0] history_row_1;
    reg [47:0] history_row_2;
    reg [47:0] history_row_3;
    reg [47:0] history_row_4;
    reg [47:0] history_row_5;
    reg [47:0] history_row_6;
    always @(posedge clk) begin
        if (reset || clear) begin
            window_valid <= 1'b0;
            window_data <= 392'd0;
            center_x <= 0;
            center_y <= 0;
            history_row_0 <= 48'd0;
            history_row_1 <= 48'd0;
            history_row_2 <= 48'd0;
            history_row_3 <= 48'd0;
            history_row_4 <= 48'd0;
            history_row_5 <= 48'd0;
            history_row_6 <= 48'd0;
        end else if (advance) begin
            window_valid <= 1'b0;
            if (column_valid) begin
                if (column_x == 0) begin
                    history_row_0 <= {column_data[7:0], 40'd0};
                    history_row_1 <= {column_data[15:8], 40'd0};
                    history_row_2 <= {column_data[23:16], 40'd0};
                    history_row_3 <= {column_data[31:24], 40'd0};
                    history_row_4 <= {column_data[39:32], 40'd0};
                    history_row_5 <= {column_data[47:40], 40'd0};
                    history_row_6 <= {column_data[55:48], 40'd0};
                end else begin
                    history_row_0 <= {column_data[7:0], history_row_0[47:8]};
                    history_row_1 <= {column_data[15:8], history_row_1[47:8]};
                    history_row_2 <= {column_data[23:16], history_row_2[47:8]};
                    history_row_3 <= {column_data[31:24], history_row_3[47:8]};
                    history_row_4 <= {column_data[39:32], history_row_4[47:8]};
                    history_row_5 <= {column_data[47:40], history_row_5[47:8]};
                    history_row_6 <= {column_data[55:48], history_row_6[47:8]};
                end
                if (column_x >= 6 && column_y >= 6) begin
                    window_valid <= 1'b1;
                    center_x <= column_x - 3;
                    center_y <= column_y - 3;
                    window_data[55:0] <= {column_data[7:0], history_row_0};
                    window_data[111:56] <= {column_data[15:8], history_row_1};
                    window_data[167:112] <= {column_data[23:16], history_row_2};
                    window_data[223:168] <= {column_data[31:24], history_row_3};
                    window_data[279:224] <= {column_data[39:32], history_row_4};
                    window_data[335:280] <= {column_data[47:40], history_row_5};
                    window_data[391:336] <= {column_data[55:48], history_row_6};
                end
            end
        end
    end
endmodule
