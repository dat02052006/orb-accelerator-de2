`timescale 1ns/1ps
// V2 hybrid full-image detector. Production port list is unchanged from V1.
// done waits for every internal valid stage and pending output to drain.
module orb_detector_frontend #(
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
    wire frontend_start_ready,frontend_done,frontend_busy;
    assign start_ready=!reset && !busy && frontend_start_ready;
    wire start_fire=start_valid && start_ready;
    reg [X_BITS-1:0] width;
    reg [Y_BITS-1:0] height;
    reg [1:0] level;
    reg [7:0] threshold;
    reg frontend_finished,nms_finished;
    wire window_valid,window_ready;
    wire [391:0] window_data;
    wire [X_BITS-1:0] window_x;
    wire [Y_BITS-1:0] window_y;
    wire [1:0] window_level;
    wire map_valid,map_ready,map_corner,map_source;
    wire [7:0] map_score;
    wire [X_BITS-1:0] map_x;
    wire [Y_BITS-1:0] map_y;
    wire [1:0] map_level;
    wire hybrid_empty,nms_empty,nms_done;
    orb_dense_frontend #(.MAX_WIDTH(MAX_WIDTH),.MAX_HEIGHT(MAX_HEIGHT),.X_BITS(X_BITS),.Y_BITS(Y_BITS)) frontend(
     .clk(clk),.reset(reset),.start_valid(start_fire),.start_ready(frontend_start_ready),
     .cfg_width(cfg_width),.cfg_height(cfg_height),.cfg_level(cfg_level),.busy(frontend_busy),.done(frontend_done),
     .pixel_data(pixel_data),.pixel_valid(pixel_valid),.pixel_ready(pixel_ready),.pixel_sof(pixel_sof),.pixel_eol(pixel_eol),
     .window_data(window_data),.window_valid(window_valid),.window_ready(window_ready),
     .center_x(window_x),.center_y(window_y),.level_id(window_level));
    orb_hybrid_detector #(.X_BITS(X_BITS),.Y_BITS(Y_BITS)) hybrid(
     .clk(clk),.reset(reset),.clear(start_fire),.width(width),.height(height),.threshold(threshold),
     .in_valid(window_valid),.in_ready(window_ready),.window_data(window_data),.in_x(window_x),.in_y(window_y),.in_level(window_level),
     .map_valid(map_valid),.map_ready(map_ready),.map_score(map_score),.map_corner(map_corner),.map_source(map_source),
     .map_x(map_x),.map_y(map_y),.map_level(map_level),.empty(hybrid_empty));
    orb_nms3x3_v2 #(.MAX_WIDTH(MAX_WIDTH),.X_BITS(X_BITS),.Y_BITS(Y_BITS)) nms(
     .clk(clk),.reset(reset),.clear(start_fire),.cfg_width(cfg_width),.cfg_height(cfg_height),.cfg_level(cfg_level),
     .score_valid(map_valid),.score_ready(map_ready),.score(map_score),.score_corner(map_corner),
     .score_x(map_x),.score_y(map_y),.score_level(map_level),
     .keypoint_valid(keypoint_valid),.keypoint_ready(keypoint_ready),.keypoint_score(keypoint_score),
     .keypoint_x(keypoint_x),.keypoint_y(keypoint_y),.keypoint_level(keypoint_level),.done(nms_done),.empty(nms_empty));
    always @(posedge clk) begin
     if(reset) begin
      busy<=0; done<=0; width<=0; height<=0; level<=0; threshold<=0; frontend_finished<=0; nms_finished<=0;
     end else begin
      done<=0;
      if(start_fire) begin
       busy<=1; width<=cfg_width; height<=cfg_height; level<=cfg_level; threshold<=cfg_threshold;
       frontend_finished<=0; nms_finished<=0;
      end else if(busy) begin
       if(frontend_done) frontend_finished<=1;
       if(nms_done) nms_finished<=1;
       if(frontend_finished && hybrid_empty && nms_finished && nms_empty) begin busy<=0; done<=1; end
      end
     end
    end
endmodule
