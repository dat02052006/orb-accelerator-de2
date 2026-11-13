`timescale 1ns/1ps
// FAST-9 on the radius-three 16-pixel circle, strict comparisons.
// Ring index 0=(row 0,col 3), clockwise in image coordinates.
module orb_fast9 #(
    parameter X_BITS = 10,
    parameter Y_BITS = 9
) (
    input wire clk, reset, clear,
    input wire [7:0] threshold,
    input wire window_valid,
    output wire window_ready,
    input wire [391:0] window_data,
    input wire [X_BITS-1:0] window_x,
    input wire [Y_BITS-1:0] window_y,
    input wire [1:0] window_level,
    output reg fast_valid,
    input wire fast_ready,
    output reg [127:0] ring_data,
    output reg [7:0] center_data,
    output reg is_corner,
    output reg [X_BITS-1:0] fast_x,
    output reg [Y_BITS-1:0] fast_y,
    output reg [1:0] fast_level
);
    wire advance = !reset && !clear && (!fast_valid || fast_ready);
    assign window_ready = advance;
    wire [7:0] center = window_data[199:192];
    wire signed [8:0] limit = $signed({1'b0, threshold});
    wire signed [8:0] negative_limit = -limit;
    wire [15:0] bright;
    wire [15:0] dark;
    wire [127:0] circle;
    wire [7:0] p0 = window_data[31:24];
    wire signed [8:0] d0 = $signed({1'b0,p0}) - $signed({1'b0,center});
    assign bright[0] = d0 > limit;
    assign dark[0] = d0 < negative_limit;
    assign circle[7:0] = p0;
    wire [7:0] p1 = window_data[39:32];
    wire signed [8:0] d1 = $signed({1'b0,p1}) - $signed({1'b0,center});
    assign bright[1] = d1 > limit;
    assign dark[1] = d1 < negative_limit;
    assign circle[15:8] = p1;
    wire [7:0] p2 = window_data[103:96];
    wire signed [8:0] d2 = $signed({1'b0,p2}) - $signed({1'b0,center});
    assign bright[2] = d2 > limit;
    assign dark[2] = d2 < negative_limit;
    assign circle[23:16] = p2;
    wire [7:0] p3 = window_data[167:160];
    wire signed [8:0] d3 = $signed({1'b0,p3}) - $signed({1'b0,center});
    assign bright[3] = d3 > limit;
    assign dark[3] = d3 < negative_limit;
    assign circle[31:24] = p3;
    wire [7:0] p4 = window_data[223:216];
    wire signed [8:0] d4 = $signed({1'b0,p4}) - $signed({1'b0,center});
    assign bright[4] = d4 > limit;
    assign dark[4] = d4 < negative_limit;
    assign circle[39:32] = p4;
    wire [7:0] p5 = window_data[279:272];
    wire signed [8:0] d5 = $signed({1'b0,p5}) - $signed({1'b0,center});
    assign bright[5] = d5 > limit;
    assign dark[5] = d5 < negative_limit;
    assign circle[47:40] = p5;
    wire [7:0] p6 = window_data[327:320];
    wire signed [8:0] d6 = $signed({1'b0,p6}) - $signed({1'b0,center});
    assign bright[6] = d6 > limit;
    assign dark[6] = d6 < negative_limit;
    assign circle[55:48] = p6;
    wire [7:0] p7 = window_data[375:368];
    wire signed [8:0] d7 = $signed({1'b0,p7}) - $signed({1'b0,center});
    assign bright[7] = d7 > limit;
    assign dark[7] = d7 < negative_limit;
    assign circle[63:56] = p7;
    wire [7:0] p8 = window_data[367:360];
    wire signed [8:0] d8 = $signed({1'b0,p8}) - $signed({1'b0,center});
    assign bright[8] = d8 > limit;
    assign dark[8] = d8 < negative_limit;
    assign circle[71:64] = p8;
    wire [7:0] p9 = window_data[359:352];
    wire signed [8:0] d9 = $signed({1'b0,p9}) - $signed({1'b0,center});
    assign bright[9] = d9 > limit;
    assign dark[9] = d9 < negative_limit;
    assign circle[79:72] = p9;
    wire [7:0] p10 = window_data[295:288];
    wire signed [8:0] d10 = $signed({1'b0,p10}) - $signed({1'b0,center});
    assign bright[10] = d10 > limit;
    assign dark[10] = d10 < negative_limit;
    assign circle[87:80] = p10;
    wire [7:0] p11 = window_data[231:224];
    wire signed [8:0] d11 = $signed({1'b0,p11}) - $signed({1'b0,center});
    assign bright[11] = d11 > limit;
    assign dark[11] = d11 < negative_limit;
    assign circle[95:88] = p11;
    wire [7:0] p12 = window_data[175:168];
    wire signed [8:0] d12 = $signed({1'b0,p12}) - $signed({1'b0,center});
    assign bright[12] = d12 > limit;
    assign dark[12] = d12 < negative_limit;
    assign circle[103:96] = p12;
    wire [7:0] p13 = window_data[119:112];
    wire signed [8:0] d13 = $signed({1'b0,p13}) - $signed({1'b0,center});
    assign bright[13] = d13 > limit;
    assign dark[13] = d13 < negative_limit;
    assign circle[111:104] = p13;
    wire [7:0] p14 = window_data[71:64];
    wire signed [8:0] d14 = $signed({1'b0,p14}) - $signed({1'b0,center});
    assign bright[14] = d14 > limit;
    assign dark[14] = d14 < negative_limit;
    assign circle[119:112] = p14;
    wire [7:0] p15 = window_data[23:16];
    wire signed [8:0] d15 = $signed({1'b0,p15}) - $signed({1'b0,center});
    assign bright[15] = d15 > limit;
    assign dark[15] = d15 < negative_limit;
    assign circle[127:120] = p15;
    // Repeat the first eight bits for arcs wrapping across index 15 -> 0.
    wire [23:0] bright_wrap = {bright[7:0], bright};
    wire [23:0] dark_wrap = {dark[7:0], dark};
    wire [15:0] arc_pass;
    assign arc_pass[0] = (&bright_wrap[8:0]) || (&dark_wrap[8:0]);
    assign arc_pass[1] = (&bright_wrap[9:1]) || (&dark_wrap[9:1]);
    assign arc_pass[2] = (&bright_wrap[10:2]) || (&dark_wrap[10:2]);
    assign arc_pass[3] = (&bright_wrap[11:3]) || (&dark_wrap[11:3]);
    assign arc_pass[4] = (&bright_wrap[12:4]) || (&dark_wrap[12:4]);
    assign arc_pass[5] = (&bright_wrap[13:5]) || (&dark_wrap[13:5]);
    assign arc_pass[6] = (&bright_wrap[14:6]) || (&dark_wrap[14:6]);
    assign arc_pass[7] = (&bright_wrap[15:7]) || (&dark_wrap[15:7]);
    assign arc_pass[8] = (&bright_wrap[16:8]) || (&dark_wrap[16:8]);
    assign arc_pass[9] = (&bright_wrap[17:9]) || (&dark_wrap[17:9]);
    assign arc_pass[10] = (&bright_wrap[18:10]) || (&dark_wrap[18:10]);
    assign arc_pass[11] = (&bright_wrap[19:11]) || (&dark_wrap[19:11]);
    assign arc_pass[12] = (&bright_wrap[20:12]) || (&dark_wrap[20:12]);
    assign arc_pass[13] = (&bright_wrap[21:13]) || (&dark_wrap[21:13]);
    assign arc_pass[14] = (&bright_wrap[22:14]) || (&dark_wrap[22:14]);
    assign arc_pass[15] = (&bright_wrap[23:15]) || (&dark_wrap[23:15]);
    wire corner_now = |arc_pass;
    always @(posedge clk) begin
        if (reset || clear) begin
            fast_valid <= 1'b0;
            ring_data <= 128'd0;
            center_data <= 8'd0;
            is_corner <= 1'b0;
            fast_x <= 0;
            fast_y <= 0;
            fast_level <= 0;
        end else if (advance) begin
            fast_valid <= window_valid;
            if (window_valid) begin
                ring_data <= circle;
                center_data <= center;
                is_corner <= corner_now;
                fast_x <= window_x;
                fast_y <= window_y;
                fast_level <= window_level;
            end
        end
    end
endmodule
