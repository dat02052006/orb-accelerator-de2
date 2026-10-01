`timescale 1ns/1ps
// Complete per-level detector: pixel frontend -> FAST-9 -> exact score -> NMS.
// done waits for every internal valid stage and pending output to drain.
module orb_detector_frontend_v1 #(
    parameter MAX_WIDTH = 640,
    parameter MAX_HEIGHT = 480,
    parameter X_BITS = 10,
    parameter Y_BITS = 9
) (
    input wire clk, reset,
    input wire start_valid,
    output wire start_ready,
    input wire [X_BITS-1:0] cfg_width,
    input wire [Y_BITS-1:0] cfg_height,
    input wire [1:0] cfg_level,
    input wire [7:0] cfg_threshold,
    output reg busy,
    output reg done,
    input wire [7:0] pixel_data,
    input wire pixel_valid,
    output wire pixel_ready,
    input wire pixel_sof,
    input wire pixel_eol,
    output wire keypoint_valid,
    input wire keypoint_ready,
    output wire [7:0] keypoint_score,
    output wire [X_BITS-1:0] keypoint_x,
    output wire [Y_BITS-1:0] keypoint_y,
    output wire [1:0] keypoint_level
);
    wire pixel_start_ready;
    assign start_ready = !reset && !busy && pixel_start_ready;
    wire start_fire = start_valid && start_ready;
    reg [7:0] threshold;
    reg pixels_finished;
    wire pixel_busy, pixel_done;
    wire window_valid, window_ready;
    wire [391:0] window_data;
    wire [X_BITS-1:0] window_x;
    wire [Y_BITS-1:0] window_y;
    wire [1:0] window_level;
    wire fast_valid, fast_ready, fast_corner;
    wire [127:0] ring_data;
    wire [7:0] center_data;
    wire [X_BITS-1:0] fast_x;
    wire [Y_BITS-1:0] fast_y;
    wire [1:0] fast_level;
    wire score_valid, score_ready, score_corner;
    wire [7:0] score;
    wire [X_BITS-1:0] score_x;
    wire [Y_BITS-1:0] score_y;
    wire [1:0] score_level;
    wire score_empty, nms_empty;

    orb_pixel_frontend #(.MAX_WIDTH(MAX_WIDTH), .MAX_HEIGHT(MAX_HEIGHT),
                        .X_BITS(X_BITS), .Y_BITS(Y_BITS)) pixel_frontend (
        .clk(clk), .reset(reset), .start_valid(start_fire),
        .start_ready(pixel_start_ready), .cfg_width(cfg_width),
        .cfg_height(cfg_height), .cfg_level(cfg_level),
        .busy(pixel_busy), .done(pixel_done),
        .pixel_data(pixel_data), .pixel_valid(pixel_valid),
        .pixel_ready(pixel_ready), .pixel_sof(pixel_sof), .pixel_eol(pixel_eol),
        .window_data(window_data), .window_valid(window_valid),
        .window_ready(window_ready), .center_x(window_x), .center_y(window_y),
        .level_id(window_level)
    );
    orb_fast9 #(.X_BITS(X_BITS), .Y_BITS(Y_BITS)) fast (
        .clk(clk), .reset(reset), .clear(start_fire), .threshold(threshold),
        .window_valid(window_valid), .window_ready(window_ready),
        .window_data(window_data), .window_x(window_x), .window_y(window_y),
        .window_level(window_level), .fast_valid(fast_valid),
        .fast_ready(fast_ready), .ring_data(ring_data), .center_data(center_data),
        .is_corner(fast_corner), .fast_x(fast_x), .fast_y(fast_y), .fast_level(fast_level)
    );
    orb_fast_score #(.X_BITS(X_BITS), .Y_BITS(Y_BITS)) fast_score (
        .clk(clk), .reset(reset), .clear(start_fire), .fast_valid(fast_valid),
        .fast_ready(fast_ready), .ring_data(ring_data), .center_data(center_data),
        .is_corner(fast_corner), .fast_x(fast_x), .fast_y(fast_y), .fast_level(fast_level),
        .score_valid(score_valid), .score_ready(score_ready), .score(score),
        .score_corner(score_corner), .score_x(score_x), .score_y(score_y),
        .score_level(score_level), .empty(score_empty)
    );
    orb_nms3x3 #(.MAX_WIDTH(MAX_WIDTH), .X_BITS(X_BITS), .Y_BITS(Y_BITS)) nms (
        .clk(clk), .reset(reset), .clear(start_fire), .cfg_width(cfg_width),
        .score_valid(score_valid), .score_ready(score_ready), .score(score),
        .score_corner(score_corner), .score_x(score_x), .score_y(score_y),
        .score_level(score_level), .keypoint_valid(keypoint_valid),
        .keypoint_ready(keypoint_ready), .keypoint_score(keypoint_score),
        .keypoint_x(keypoint_x), .keypoint_y(keypoint_y),
        .keypoint_level(keypoint_level), .empty(nms_empty)
    );
    always @(posedge clk) begin
        if (reset) begin
            busy <= 1'b0;
            done <= 1'b0;
            threshold <= 8'd0;
            pixels_finished <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start_fire) begin
                busy <= 1'b1;
                threshold <= cfg_threshold;
                pixels_finished <= 1'b0;
            end else if (busy) begin
                if (pixel_done)
                    pixels_finished <= 1'b1;
                if (pixels_finished && !window_valid && !fast_valid && score_empty && nms_empty) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end
endmodule
