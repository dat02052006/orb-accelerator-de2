`timescale 1ns/1ps
// Pixel stream -> six line RAMs -> 7x7 windows, one shared pipeline per level.
// Legal configuration: 7 <= width <= MAX_WIDTH, 7 <= height <= MAX_HEIGHT.
// Stream contract: SOF iff (x,y)==(0,0), EOL iff x==width-1.
// SOF/EOL are checked by the testbench; configured dimensions drive counters.
module orb_pixel_frontend #(
    parameter MAX_WIDTH = 640,
    parameter MAX_HEIGHT = 480,
    parameter X_BITS = 10,
    parameter Y_BITS = 9
) (
    input wire clk,
    input wire reset,
    input wire start_valid,
    output wire start_ready,
    input wire [X_BITS-1:0] cfg_width,
    input wire [Y_BITS-1:0] cfg_height,
    input wire [1:0] cfg_level,
    output reg busy,
    output reg done,
    input wire [7:0] pixel_data,
    input wire pixel_valid,
    output wire pixel_ready,
    input wire pixel_sof,
    input wire pixel_eol,
    output wire [391:0] window_data,
    output wire window_valid,
    input wire window_ready,
    output wire [X_BITS-1:0] center_x,
    output wire [Y_BITS-1:0] center_y,
    output reg [1:0] level_id
);
    reg [X_BITS-1:0] width;
    reg [Y_BITS-1:0] height;
    reg [X_BITS-1:0] x;
    reg [Y_BITS-1:0] y;
    reg [2:0] write_bank;
    reg input_finished;
    wire config_legal = cfg_width >= 7 && cfg_width <= MAX_WIDTH &&
                        cfg_height >= 7 && cfg_height <= MAX_HEIGHT &&
                        cfg_level <= 2;
    assign start_ready = !reset && !busy && config_legal;
    wire start_fire = start_valid && start_ready;
    wire advance = !reset && (!window_valid || window_ready);
    assign pixel_ready = !reset && busy && !input_finished && advance;
    wire pixel_fire = pixel_valid && pixel_ready;
    wire column_valid;
    wire [55:0] column_data;
    wire [X_BITS-1:0] column_x;
    wire [Y_BITS-1:0] column_y;

    orb_line_buffer #(.MAX_WIDTH(MAX_WIDTH), .X_BITS(X_BITS), .Y_BITS(Y_BITS))
    line_buffer (
        .clk(clk), .reset(reset), .clear(start_fire), .advance(advance),
        .pixel_fire(pixel_fire), .pixel_data(pixel_data),
        .pixel_x(x), .pixel_y(y), .write_bank(write_bank),
        .column_valid(column_valid), .column_x(column_x),
        .column_y(column_y), .column_data(column_data)
    );
    orb_window_buffer #(.X_BITS(X_BITS), .Y_BITS(Y_BITS)) window_buffer (
        .clk(clk), .reset(reset), .clear(start_fire), .advance(advance),
        .column_valid(column_valid), .column_data(column_data),
        .column_x(column_x), .column_y(column_y),
        .window_valid(window_valid), .window_data(window_data),
        .center_x(center_x), .center_y(center_y)
    );

    always @(posedge clk) begin
        if (reset) begin
            busy <= 1'b0;
            done <= 1'b0;
            width <= 0;
            height <= 0;
            level_id <= 0;
            x <= 0;
            y <= 0;
            write_bank <= 0;
            input_finished <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start_fire) begin
                busy <= 1'b1;
                width <= cfg_width;
                height <= cfg_height;
                level_id <= cfg_level;
                x <= 0;
                y <= 0;
                write_bank <= 0;
                input_finished <= 1'b0;
            end else if (busy) begin
                if (pixel_fire) begin
                    if (x == width - 1'b1) begin
                        x <= 0;
                        if (y == height - 1'b1)
                            input_finished <= 1'b1;
                        else begin
                            y <= y + 1'b1;
                            if (write_bank == 5)
                                write_bank <= 0;
                            else
                                write_bank <= write_bank + 1'b1;
                        end
                    end else
                        x <= x + 1'b1;
                end
                if (input_finished && !column_valid && advance) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end
endmodule
