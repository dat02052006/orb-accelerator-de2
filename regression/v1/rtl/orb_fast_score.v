`timescale 1ns/1ps
// Exact integer FAST-9 score: max over 32 signed nine-pixel arc minima, minus 1.
// Three registered stages: arc minima -> maxima in groups of four -> final score.
// A single clock-enable freezes every stage when the last stage is blocked.
module orb_fast_score #(
    parameter X_BITS = 10,
    parameter Y_BITS = 9
) (
    input wire clk, reset, clear,
    input wire fast_valid,
    output wire fast_ready,
    input wire [127:0] ring_data,
    input wire [7:0] center_data,
    input wire is_corner,
    input wire [X_BITS-1:0] fast_x,
    input wire [Y_BITS-1:0] fast_y,
    input wire [1:0] fast_level,
    output reg score_valid,
    input wire score_ready,
    output reg [7:0] score,
    output reg score_corner,
    output reg [X_BITS-1:0] score_x,
    output reg [Y_BITS-1:0] score_y,
    output reg [1:0] score_level,
    output wire empty
);
    reg valid1, valid2;
    reg corner1, corner2;
    reg [X_BITS-1:0] x1, x2;
    reg [Y_BITS-1:0] y1, y2;
    reg [1:0] level1, level2;
    wire advance = !reset && !clear && (!score_valid || score_ready);
    assign fast_ready = advance;
    assign empty = !valid1 && !valid2 && !score_valid;
    wire signed [8:0] d0 = $signed({1'b0,ring_data[7:0]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd0 = -d0;
    wire signed [8:0] d1 = $signed({1'b0,ring_data[15:8]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd1 = -d1;
    wire signed [8:0] d2 = $signed({1'b0,ring_data[23:16]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd2 = -d2;
    wire signed [8:0] d3 = $signed({1'b0,ring_data[31:24]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd3 = -d3;
    wire signed [8:0] d4 = $signed({1'b0,ring_data[39:32]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd4 = -d4;
    wire signed [8:0] d5 = $signed({1'b0,ring_data[47:40]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd5 = -d5;
    wire signed [8:0] d6 = $signed({1'b0,ring_data[55:48]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd6 = -d6;
    wire signed [8:0] d7 = $signed({1'b0,ring_data[63:56]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd7 = -d7;
    wire signed [8:0] d8 = $signed({1'b0,ring_data[71:64]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd8 = -d8;
    wire signed [8:0] d9 = $signed({1'b0,ring_data[79:72]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd9 = -d9;
    wire signed [8:0] d10 = $signed({1'b0,ring_data[87:80]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd10 = -d10;
    wire signed [8:0] d11 = $signed({1'b0,ring_data[95:88]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd11 = -d11;
    wire signed [8:0] d12 = $signed({1'b0,ring_data[103:96]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd12 = -d12;
    wire signed [8:0] d13 = $signed({1'b0,ring_data[111:104]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd13 = -d13;
    wire signed [8:0] d14 = $signed({1'b0,ring_data[119:112]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd14 = -d14;
    wire signed [8:0] d15 = $signed({1'b0,ring_data[127:120]}) - $signed({1'b0,center_data});
    wire signed [8:0] nd15 = -d15;
    wire signed [8:0] arc_min_0;
    reg signed [8:0] arc_reg_0;
    wire signed [8:0] arc_min_1;
    reg signed [8:0] arc_reg_1;
    wire signed [8:0] arc_min_2;
    reg signed [8:0] arc_reg_2;
    wire signed [8:0] arc_min_3;
    reg signed [8:0] arc_reg_3;
    wire signed [8:0] arc_min_4;
    reg signed [8:0] arc_reg_4;
    wire signed [8:0] arc_min_5;
    reg signed [8:0] arc_reg_5;
    wire signed [8:0] arc_min_6;
    reg signed [8:0] arc_reg_6;
    wire signed [8:0] arc_min_7;
    reg signed [8:0] arc_reg_7;
    wire signed [8:0] arc_min_8;
    reg signed [8:0] arc_reg_8;
    wire signed [8:0] arc_min_9;
    reg signed [8:0] arc_reg_9;
    wire signed [8:0] arc_min_10;
    reg signed [8:0] arc_reg_10;
    wire signed [8:0] arc_min_11;
    reg signed [8:0] arc_reg_11;
    wire signed [8:0] arc_min_12;
    reg signed [8:0] arc_reg_12;
    wire signed [8:0] arc_min_13;
    reg signed [8:0] arc_reg_13;
    wire signed [8:0] arc_min_14;
    reg signed [8:0] arc_reg_14;
    wire signed [8:0] arc_min_15;
    reg signed [8:0] arc_reg_15;
    wire signed [8:0] arc_min_16;
    reg signed [8:0] arc_reg_16;
    wire signed [8:0] arc_min_17;
    reg signed [8:0] arc_reg_17;
    wire signed [8:0] arc_min_18;
    reg signed [8:0] arc_reg_18;
    wire signed [8:0] arc_min_19;
    reg signed [8:0] arc_reg_19;
    wire signed [8:0] arc_min_20;
    reg signed [8:0] arc_reg_20;
    wire signed [8:0] arc_min_21;
    reg signed [8:0] arc_reg_21;
    wire signed [8:0] arc_min_22;
    reg signed [8:0] arc_reg_22;
    wire signed [8:0] arc_min_23;
    reg signed [8:0] arc_reg_23;
    wire signed [8:0] arc_min_24;
    reg signed [8:0] arc_reg_24;
    wire signed [8:0] arc_min_25;
    reg signed [8:0] arc_reg_25;
    wire signed [8:0] arc_min_26;
    reg signed [8:0] arc_reg_26;
    wire signed [8:0] arc_min_27;
    reg signed [8:0] arc_reg_27;
    wire signed [8:0] arc_min_28;
    reg signed [8:0] arc_reg_28;
    wire signed [8:0] arc_min_29;
    reg signed [8:0] arc_reg_29;
    wire signed [8:0] arc_min_30;
    reg signed [8:0] arc_reg_30;
    wire signed [8:0] arc_min_31;
    reg signed [8:0] arc_reg_31;
    orb_min9_signed arc_0 (.a0(d0), .a1(d1), .a2(d2), .a3(d3), .a4(d4), .a5(d5), .a6(d6), .a7(d7), .a8(d8), .result(arc_min_0));
    orb_min9_signed arc_1 (.a0(d1), .a1(d2), .a2(d3), .a3(d4), .a4(d5), .a5(d6), .a6(d7), .a7(d8), .a8(d9), .result(arc_min_1));
    orb_min9_signed arc_2 (.a0(d2), .a1(d3), .a2(d4), .a3(d5), .a4(d6), .a5(d7), .a6(d8), .a7(d9), .a8(d10), .result(arc_min_2));
    orb_min9_signed arc_3 (.a0(d3), .a1(d4), .a2(d5), .a3(d6), .a4(d7), .a5(d8), .a6(d9), .a7(d10), .a8(d11), .result(arc_min_3));
    orb_min9_signed arc_4 (.a0(d4), .a1(d5), .a2(d6), .a3(d7), .a4(d8), .a5(d9), .a6(d10), .a7(d11), .a8(d12), .result(arc_min_4));
    orb_min9_signed arc_5 (.a0(d5), .a1(d6), .a2(d7), .a3(d8), .a4(d9), .a5(d10), .a6(d11), .a7(d12), .a8(d13), .result(arc_min_5));
    orb_min9_signed arc_6 (.a0(d6), .a1(d7), .a2(d8), .a3(d9), .a4(d10), .a5(d11), .a6(d12), .a7(d13), .a8(d14), .result(arc_min_6));
    orb_min9_signed arc_7 (.a0(d7), .a1(d8), .a2(d9), .a3(d10), .a4(d11), .a5(d12), .a6(d13), .a7(d14), .a8(d15), .result(arc_min_7));
    orb_min9_signed arc_8 (.a0(d8), .a1(d9), .a2(d10), .a3(d11), .a4(d12), .a5(d13), .a6(d14), .a7(d15), .a8(d0), .result(arc_min_8));
    orb_min9_signed arc_9 (.a0(d9), .a1(d10), .a2(d11), .a3(d12), .a4(d13), .a5(d14), .a6(d15), .a7(d0), .a8(d1), .result(arc_min_9));
    orb_min9_signed arc_10 (.a0(d10), .a1(d11), .a2(d12), .a3(d13), .a4(d14), .a5(d15), .a6(d0), .a7(d1), .a8(d2), .result(arc_min_10));
    orb_min9_signed arc_11 (.a0(d11), .a1(d12), .a2(d13), .a3(d14), .a4(d15), .a5(d0), .a6(d1), .a7(d2), .a8(d3), .result(arc_min_11));
    orb_min9_signed arc_12 (.a0(d12), .a1(d13), .a2(d14), .a3(d15), .a4(d0), .a5(d1), .a6(d2), .a7(d3), .a8(d4), .result(arc_min_12));
    orb_min9_signed arc_13 (.a0(d13), .a1(d14), .a2(d15), .a3(d0), .a4(d1), .a5(d2), .a6(d3), .a7(d4), .a8(d5), .result(arc_min_13));
    orb_min9_signed arc_14 (.a0(d14), .a1(d15), .a2(d0), .a3(d1), .a4(d2), .a5(d3), .a6(d4), .a7(d5), .a8(d6), .result(arc_min_14));
    orb_min9_signed arc_15 (.a0(d15), .a1(d0), .a2(d1), .a3(d2), .a4(d3), .a5(d4), .a6(d5), .a7(d6), .a8(d7), .result(arc_min_15));
    orb_min9_signed arc_16 (.a0(nd0), .a1(nd1), .a2(nd2), .a3(nd3), .a4(nd4), .a5(nd5), .a6(nd6), .a7(nd7), .a8(nd8), .result(arc_min_16));
    orb_min9_signed arc_17 (.a0(nd1), .a1(nd2), .a2(nd3), .a3(nd4), .a4(nd5), .a5(nd6), .a6(nd7), .a7(nd8), .a8(nd9), .result(arc_min_17));
    orb_min9_signed arc_18 (.a0(nd2), .a1(nd3), .a2(nd4), .a3(nd5), .a4(nd6), .a5(nd7), .a6(nd8), .a7(nd9), .a8(nd10), .result(arc_min_18));
    orb_min9_signed arc_19 (.a0(nd3), .a1(nd4), .a2(nd5), .a3(nd6), .a4(nd7), .a5(nd8), .a6(nd9), .a7(nd10), .a8(nd11), .result(arc_min_19));
    orb_min9_signed arc_20 (.a0(nd4), .a1(nd5), .a2(nd6), .a3(nd7), .a4(nd8), .a5(nd9), .a6(nd10), .a7(nd11), .a8(nd12), .result(arc_min_20));
    orb_min9_signed arc_21 (.a0(nd5), .a1(nd6), .a2(nd7), .a3(nd8), .a4(nd9), .a5(nd10), .a6(nd11), .a7(nd12), .a8(nd13), .result(arc_min_21));
    orb_min9_signed arc_22 (.a0(nd6), .a1(nd7), .a2(nd8), .a3(nd9), .a4(nd10), .a5(nd11), .a6(nd12), .a7(nd13), .a8(nd14), .result(arc_min_22));
    orb_min9_signed arc_23 (.a0(nd7), .a1(nd8), .a2(nd9), .a3(nd10), .a4(nd11), .a5(nd12), .a6(nd13), .a7(nd14), .a8(nd15), .result(arc_min_23));
    orb_min9_signed arc_24 (.a0(nd8), .a1(nd9), .a2(nd10), .a3(nd11), .a4(nd12), .a5(nd13), .a6(nd14), .a7(nd15), .a8(nd0), .result(arc_min_24));
    orb_min9_signed arc_25 (.a0(nd9), .a1(nd10), .a2(nd11), .a3(nd12), .a4(nd13), .a5(nd14), .a6(nd15), .a7(nd0), .a8(nd1), .result(arc_min_25));
    orb_min9_signed arc_26 (.a0(nd10), .a1(nd11), .a2(nd12), .a3(nd13), .a4(nd14), .a5(nd15), .a6(nd0), .a7(nd1), .a8(nd2), .result(arc_min_26));
    orb_min9_signed arc_27 (.a0(nd11), .a1(nd12), .a2(nd13), .a3(nd14), .a4(nd15), .a5(nd0), .a6(nd1), .a7(nd2), .a8(nd3), .result(arc_min_27));
    orb_min9_signed arc_28 (.a0(nd12), .a1(nd13), .a2(nd14), .a3(nd15), .a4(nd0), .a5(nd1), .a6(nd2), .a7(nd3), .a8(nd4), .result(arc_min_28));
    orb_min9_signed arc_29 (.a0(nd13), .a1(nd14), .a2(nd15), .a3(nd0), .a4(nd1), .a5(nd2), .a6(nd3), .a7(nd4), .a8(nd5), .result(arc_min_29));
    orb_min9_signed arc_30 (.a0(nd14), .a1(nd15), .a2(nd0), .a3(nd1), .a4(nd2), .a5(nd3), .a6(nd4), .a7(nd5), .a8(nd6), .result(arc_min_30));
    orb_min9_signed arc_31 (.a0(nd15), .a1(nd0), .a2(nd1), .a3(nd2), .a4(nd3), .a5(nd4), .a6(nd5), .a7(nd6), .a8(nd7), .result(arc_min_31));
    wire signed [8:0] group_max_0;
    reg signed [8:0] group_reg_0;
    orb_max4_signed group_0 (.a0(arc_reg_0), .a1(arc_reg_1), .a2(arc_reg_2), .a3(arc_reg_3), .result(group_max_0));
    wire signed [8:0] group_max_1;
    reg signed [8:0] group_reg_1;
    orb_max4_signed group_1 (.a0(arc_reg_4), .a1(arc_reg_5), .a2(arc_reg_6), .a3(arc_reg_7), .result(group_max_1));
    wire signed [8:0] group_max_2;
    reg signed [8:0] group_reg_2;
    orb_max4_signed group_2 (.a0(arc_reg_8), .a1(arc_reg_9), .a2(arc_reg_10), .a3(arc_reg_11), .result(group_max_2));
    wire signed [8:0] group_max_3;
    reg signed [8:0] group_reg_3;
    orb_max4_signed group_3 (.a0(arc_reg_12), .a1(arc_reg_13), .a2(arc_reg_14), .a3(arc_reg_15), .result(group_max_3));
    wire signed [8:0] group_max_4;
    reg signed [8:0] group_reg_4;
    orb_max4_signed group_4 (.a0(arc_reg_16), .a1(arc_reg_17), .a2(arc_reg_18), .a3(arc_reg_19), .result(group_max_4));
    wire signed [8:0] group_max_5;
    reg signed [8:0] group_reg_5;
    orb_max4_signed group_5 (.a0(arc_reg_20), .a1(arc_reg_21), .a2(arc_reg_22), .a3(arc_reg_23), .result(group_max_5));
    wire signed [8:0] group_max_6;
    reg signed [8:0] group_reg_6;
    orb_max4_signed group_6 (.a0(arc_reg_24), .a1(arc_reg_25), .a2(arc_reg_26), .a3(arc_reg_27), .result(group_max_6));
    wire signed [8:0] group_max_7;
    reg signed [8:0] group_reg_7;
    orb_max4_signed group_7 (.a0(arc_reg_28), .a1(arc_reg_29), .a2(arc_reg_30), .a3(arc_reg_31), .result(group_max_7));
    wire signed [8:0] best_margin;
    orb_max8_signed final_max (.a0(group_reg_0), .a1(group_reg_1), .a2(group_reg_2), .a3(group_reg_3), .a4(group_reg_4), .a5(group_reg_5), .a6(group_reg_6), .a7(group_reg_7), .result(best_margin));
    always @(posedge clk) begin
        if (reset || clear) begin
            valid1 <= 1'b0;
            valid2 <= 1'b0;
            score_valid <= 1'b0;
            corner1 <= 1'b0;
            corner2 <= 1'b0;
            score_corner <= 1'b0;
            x1 <= 0; x2 <= 0; score_x <= 0;
            y1 <= 0; y2 <= 0; score_y <= 0;
            level1 <= 0; level2 <= 0; score_level <= 0;
            score <= 8'd0;
            arc_reg_0 <= 9'sd0;
            arc_reg_1 <= 9'sd0;
            arc_reg_2 <= 9'sd0;
            arc_reg_3 <= 9'sd0;
            arc_reg_4 <= 9'sd0;
            arc_reg_5 <= 9'sd0;
            arc_reg_6 <= 9'sd0;
            arc_reg_7 <= 9'sd0;
            arc_reg_8 <= 9'sd0;
            arc_reg_9 <= 9'sd0;
            arc_reg_10 <= 9'sd0;
            arc_reg_11 <= 9'sd0;
            arc_reg_12 <= 9'sd0;
            arc_reg_13 <= 9'sd0;
            arc_reg_14 <= 9'sd0;
            arc_reg_15 <= 9'sd0;
            arc_reg_16 <= 9'sd0;
            arc_reg_17 <= 9'sd0;
            arc_reg_18 <= 9'sd0;
            arc_reg_19 <= 9'sd0;
            arc_reg_20 <= 9'sd0;
            arc_reg_21 <= 9'sd0;
            arc_reg_22 <= 9'sd0;
            arc_reg_23 <= 9'sd0;
            arc_reg_24 <= 9'sd0;
            arc_reg_25 <= 9'sd0;
            arc_reg_26 <= 9'sd0;
            arc_reg_27 <= 9'sd0;
            arc_reg_28 <= 9'sd0;
            arc_reg_29 <= 9'sd0;
            arc_reg_30 <= 9'sd0;
            arc_reg_31 <= 9'sd0;
            group_reg_0 <= 9'sd0;
            group_reg_1 <= 9'sd0;
            group_reg_2 <= 9'sd0;
            group_reg_3 <= 9'sd0;
            group_reg_4 <= 9'sd0;
            group_reg_5 <= 9'sd0;
            group_reg_6 <= 9'sd0;
            group_reg_7 <= 9'sd0;
        end else if (advance) begin
            valid1 <= fast_valid;
            valid2 <= valid1;
            score_valid <= valid2;
            if (fast_valid) begin
                corner1 <= is_corner;
                x1 <= fast_x; y1 <= fast_y; level1 <= fast_level;
                arc_reg_0 <= arc_min_0;
                arc_reg_1 <= arc_min_1;
                arc_reg_2 <= arc_min_2;
                arc_reg_3 <= arc_min_3;
                arc_reg_4 <= arc_min_4;
                arc_reg_5 <= arc_min_5;
                arc_reg_6 <= arc_min_6;
                arc_reg_7 <= arc_min_7;
                arc_reg_8 <= arc_min_8;
                arc_reg_9 <= arc_min_9;
                arc_reg_10 <= arc_min_10;
                arc_reg_11 <= arc_min_11;
                arc_reg_12 <= arc_min_12;
                arc_reg_13 <= arc_min_13;
                arc_reg_14 <= arc_min_14;
                arc_reg_15 <= arc_min_15;
                arc_reg_16 <= arc_min_16;
                arc_reg_17 <= arc_min_17;
                arc_reg_18 <= arc_min_18;
                arc_reg_19 <= arc_min_19;
                arc_reg_20 <= arc_min_20;
                arc_reg_21 <= arc_min_21;
                arc_reg_22 <= arc_min_22;
                arc_reg_23 <= arc_min_23;
                arc_reg_24 <= arc_min_24;
                arc_reg_25 <= arc_min_25;
                arc_reg_26 <= arc_min_26;
                arc_reg_27 <= arc_min_27;
                arc_reg_28 <= arc_min_28;
                arc_reg_29 <= arc_min_29;
                arc_reg_30 <= arc_min_30;
                arc_reg_31 <= arc_min_31;
            end
            if (valid1) begin
                corner2 <= corner1;
                x2 <= x1; y2 <= y1; level2 <= level1;
                group_reg_0 <= group_max_0;
                group_reg_1 <= group_max_1;
                group_reg_2 <= group_max_2;
                group_reg_3 <= group_max_3;
                group_reg_4 <= group_max_4;
                group_reg_5 <= group_max_5;
                group_reg_6 <= group_max_6;
                group_reg_7 <= group_max_7;
            end
            if (valid2) begin
                score_corner <= corner2;
                score_x <= x2; score_y <= y2; score_level <= level2;
                if (corner2 && best_margin > 9'sd0)
                    score <= best_margin[7:0] - 8'd1;
                else
                    score <= 8'd0;
            end
        end
    end
endmodule
