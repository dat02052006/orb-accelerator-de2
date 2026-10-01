`timescale 1ns/1ps
// Dense score raster x=3..width-4, y=3..height-4 -> sparse strict 3x3 maxima.
// Two rotating row RAMs. No padding: retained centers x=4..width-5.
module orb_nms3x3 #(
    parameter MAX_WIDTH = 640,
    parameter X_BITS = 10,
    parameter Y_BITS = 9
) (
    input wire clk, reset, clear,
    input wire [X_BITS-1:0] cfg_width,
    input wire score_valid,
    output wire score_ready,
    input wire [7:0] score,
    input wire score_corner,
    input wire [X_BITS-1:0] score_x,
    input wire [Y_BITS-1:0] score_y,
    input wire [1:0] score_level,
    output reg keypoint_valid,
    input wire keypoint_ready,
    output reg [7:0] keypoint_score,
    output reg [X_BITS-1:0] keypoint_x,
    output reg [Y_BITS-1:0] keypoint_y,
    output reg [1:0] keypoint_level,
    output wire empty
);
    wire advance = !reset && !clear && (!keypoint_valid || keypoint_ready);
    assign score_ready = advance;
    wire score_fire = score_valid && score_ready;
    wire [X_BITS-1:0] address = score_x - 3;
    wire [8:0] input_data = {score_corner, score};
    reg bank;
    reg [X_BITS-1:0] last_x;
    wire [8:0] read0, read1;
    orb_nms_row_ram #(.MAX_WIDTH(MAX_WIDTH-6), .X_BITS(X_BITS)) row_ram_0 (
        .clk(clk), .enable(score_fire), .write_enable(!bank),
        .address(address), .write_data(input_data), .read_data(read0)
    );
    orb_nms_row_ram #(.MAX_WIDTH(MAX_WIDTH-6), .X_BITS(X_BITS)) row_ram_1 (
        .clk(clk), .enable(score_fire), .write_enable(bank),
        .address(address), .write_data(input_data), .read_data(read1)
    );

    reg column_valid, column_bank;
    reg [8:0] column_bottom;
    reg [X_BITS-1:0] column_x;
    reg [Y_BITS-1:0] column_y;
    reg [1:0] column_level;
    wire [8:0] column_top = column_bank ? read1 : read0;
    wire [8:0] column_middle = column_bank ? read0 : read1;
    // Two previous columns: byte-like nine-bit records, oldest at [8:0].
    reg [17:0] history_top, history_middle, history_bottom;
    wire [7:0] center_score = history_middle[16:9];
    wire center_corner = history_middle[17];
    wire keep = center_corner &&
                center_score > history_top[7:0] &&
                center_score > history_top[16:9] &&
                center_score > column_top[7:0] &&
                center_score > history_middle[7:0] &&
                center_score > column_middle[7:0] &&
                center_score > history_bottom[7:0] &&
                center_score > history_bottom[16:9] &&
                center_score > column_bottom[7:0];
    assign empty = !column_valid && !keypoint_valid;

    always @(posedge clk) begin
        if (reset || clear) begin
            bank <= 1'b0;
            last_x <= cfg_width - 4;
            column_valid <= 1'b0;
            column_bank <= 1'b0;
            column_bottom <= 9'd0;
            column_x <= 0; column_y <= 0; column_level <= 0;
            history_top <= 18'd0;
            history_middle <= 18'd0;
            history_bottom <= 18'd0;
            keypoint_valid <= 1'b0;
            keypoint_score <= 8'd0;
            keypoint_x <= 0; keypoint_y <= 0; keypoint_level <= 0;
        end else if (advance) begin
            column_valid <= score_fire;
            if (score_fire) begin
                column_bank <= bank;
                column_bottom <= input_data;
                column_x <= score_x; column_y <= score_y;
                column_level <= score_level;
                if (score_x == last_x)
                    bank <= !bank;
            end
            keypoint_valid <= 1'b0;
            if (column_valid) begin
                if (column_x == 3) begin
                    history_top <= {column_top, 9'd0};
                    history_middle <= {column_middle, 9'd0};
                    history_bottom <= {column_bottom, 9'd0};
                end else begin
                    history_top <= {column_top, history_top[17:9]};
                    history_middle <= {column_middle, history_middle[17:9]};
                    history_bottom <= {column_bottom, history_bottom[17:9]};
                end
                if (column_x >= 5 && column_y >= 5 && keep) begin
                    keypoint_valid <= 1'b1;
                    keypoint_score <= center_score;
                    keypoint_x <= column_x - 1'b1;
                    keypoint_y <= column_y - 1'b1;
                    keypoint_level <= column_level;
                end
            end
        end
    end
endmodule
