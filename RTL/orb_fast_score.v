`timescale 1ns/1ps
// Exact integer FAST-9 score: max over 32 signed nine-pixel arc minima, minus 1.
// Three registered stages: arc minima -> maxima in groups of four -> final score.
// A single clock-enable freezes every stage when the last stage is blocked.
//
// Packed-bus version: arc_min/arc_reg are 288-bit buses (32 x 9 bit),
// group_max/group_reg are 72-bit buses (8 x 9 bit). No for/generate.
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
    // ---------------------------------------------------------------
    // Stage bookkeeping
    // ---------------------------------------------------------------
    reg valid1, valid2;
    reg corner1, corner2;
    reg [X_BITS-1:0] x1, x2;
    reg [Y_BITS-1:0] y1, y2;
    reg [1:0] level1, level2;

    wire advance = !reset && !clear && (!score_valid || score_ready);
    assign fast_ready = advance;
    assign empty = !valid1 && !valid2 && !score_valid;

    // ---------------------------------------------------------------
    // 16 signed differences, plus their negations used inline for dark arcs
    // ---------------------------------------------------------------
    wire signed [8:0] d0  = $signed({1'b0,ring_data[7:0]})    - $signed({1'b0,center_data});
    wire signed [8:0] d1  = $signed({1'b0,ring_data[15:8]})   - $signed({1'b0,center_data});
    wire signed [8:0] d2  = $signed({1'b0,ring_data[23:16]})  - $signed({1'b0,center_data});
    wire signed [8:0] d3  = $signed({1'b0,ring_data[31:24]})  - $signed({1'b0,center_data});
    wire signed [8:0] d4  = $signed({1'b0,ring_data[39:32]})  - $signed({1'b0,center_data});
    wire signed [8:0] d5  = $signed({1'b0,ring_data[47:40]})  - $signed({1'b0,center_data});
    wire signed [8:0] d6  = $signed({1'b0,ring_data[55:48]})  - $signed({1'b0,center_data});
    wire signed [8:0] d7  = $signed({1'b0,ring_data[63:56]})  - $signed({1'b0,center_data});
    wire signed [8:0] d8  = $signed({1'b0,ring_data[71:64]})  - $signed({1'b0,center_data});
    wire signed [8:0] d9  = $signed({1'b0,ring_data[79:72]})  - $signed({1'b0,center_data});
    wire signed [8:0] d10 = $signed({1'b0,ring_data[87:80]})  - $signed({1'b0,center_data});
    wire signed [8:0] d11 = $signed({1'b0,ring_data[95:88]})  - $signed({1'b0,center_data});
    wire signed [8:0] d12 = $signed({1'b0,ring_data[103:96]}) - $signed({1'b0,center_data});
    wire signed [8:0] d13 = $signed({1'b0,ring_data[111:104]})- $signed({1'b0,center_data});
    wire signed [8:0] d14 = $signed({1'b0,ring_data[119:112]})- $signed({1'b0,center_data});
    wire signed [8:0] d15 = $signed({1'b0,ring_data[127:120]})- $signed({1'b0,center_data});

    // ---------------------------------------------------------------
    // Packed buses
    //   arc_min  : 32 x 9 bit = 288 bit   (combinational arc minima)
    //   arc_reg  : 32 x 9 bit = 288 bit   (stage-1 pipeline register)
    //   group_max:  8 x 9 bit =  72 bit   (combinational group maxima)
    //   group_reg:  8 x 9 bit =  72 bit   (stage-2 pipeline register)
    // ---------------------------------------------------------------
    wire signed [287:0] arc_min;
    reg  signed [287:0] arc_reg;
    wire signed [71:0]  group_max;
    reg  signed [71:0]  group_reg;

    // ---------------------------------------------------------------
    // Stage 1 arithmetic: 32 signed arc minima
    //   arcs  0..15 : bright, a_k = d_k
    //   arcs 16..31 : dark,   a_k = -d_k
    // ---------------------------------------------------------------
    orb_min9_signed arc_0  (.a0(d0),  .a1(d1),  .a2(d2),  .a3(d3),  .a4(d4),  .a5(d5),  .a6(d6),  .a7(d7),  .a8(d8),  .result(arc_min[8:0]));
    orb_min9_signed arc_1  (.a0(d1),  .a1(d2),  .a2(d3),  .a3(d4),  .a4(d5),  .a5(d6),  .a6(d7),  .a7(d8),  .a8(d9),  .result(arc_min[17:9]));
    orb_min9_signed arc_2  (.a0(d2),  .a1(d3),  .a2(d4),  .a3(d5),  .a4(d6),  .a5(d7),  .a6(d8),  .a7(d9),  .a8(d10), .result(arc_min[26:18]));
    orb_min9_signed arc_3  (.a0(d3),  .a1(d4),  .a2(d5),  .a3(d6),  .a4(d7),  .a5(d8),  .a6(d9),  .a7(d10), .a8(d11), .result(arc_min[35:27]));
    orb_min9_signed arc_4  (.a0(d4),  .a1(d5),  .a2(d6),  .a3(d7),  .a4(d8),  .a5(d9),  .a6(d10), .a7(d11), .a8(d12), .result(arc_min[44:36]));
    orb_min9_signed arc_5  (.a0(d5),  .a1(d6),  .a2(d7),  .a3(d8),  .a4(d9),  .a5(d10), .a6(d11), .a7(d12), .a8(d13), .result(arc_min[53:45]));
    orb_min9_signed arc_6  (.a0(d6),  .a1(d7),  .a2(d8),  .a3(d9),  .a4(d10), .a5(d11), .a6(d12), .a7(d13), .a8(d14), .result(arc_min[62:54]));
    orb_min9_signed arc_7  (.a0(d7),  .a1(d8),  .a2(d9),  .a3(d10), .a4(d11), .a5(d12), .a6(d13), .a7(d14), .a8(d15), .result(arc_min[71:63]));
    orb_min9_signed arc_8  (.a0(d8),  .a1(d9),  .a2(d10), .a3(d11), .a4(d12), .a5(d13), .a6(d14), .a7(d15), .a8(d0),  .result(arc_min[80:72]));
    orb_min9_signed arc_9  (.a0(d9),  .a1(d10), .a2(d11), .a3(d12), .a4(d13), .a5(d14), .a6(d15), .a7(d0),  .a8(d1),  .result(arc_min[89:81]));
    orb_min9_signed arc_10 (.a0(d10), .a1(d11), .a2(d12), .a3(d13), .a4(d14), .a5(d15), .a6(d0),  .a7(d1),  .a8(d2),  .result(arc_min[98:90]));
    orb_min9_signed arc_11 (.a0(d11), .a1(d12), .a2(d13), .a3(d14), .a4(d15), .a5(d0),  .a6(d1),  .a7(d2),  .a8(d3),  .result(arc_min[107:99]));
    orb_min9_signed arc_12 (.a0(d12), .a1(d13), .a2(d14), .a3(d15), .a4(d0),  .a5(d1),  .a6(d2),  .a7(d3),  .a8(d4),  .result(arc_min[116:108]));
    orb_min9_signed arc_13 (.a0(d13), .a1(d14), .a2(d15), .a3(d0),  .a4(d1),  .a5(d2),  .a6(d3),  .a7(d4),  .a8(d5),  .result(arc_min[125:117]));
    orb_min9_signed arc_14 (.a0(d14), .a1(d15), .a2(d0),  .a3(d1),  .a4(d2),  .a5(d3),  .a6(d4),  .a7(d5),  .a8(d6),  .result(arc_min[134:126]));
    orb_min9_signed arc_15 (.a0(d15), .a1(d0),  .a2(d1),  .a3(d2),  .a4(d3),  .a5(d4),  .a6(d5),  .a7(d6),  .a8(d7),  .result(arc_min[143:135]));
    orb_min9_signed arc_16 (.a0(-d0),  .a1(-d1),  .a2(-d2),  .a3(-d3),  .a4(-d4),  .a5(-d5),  .a6(-d6),  .a7(-d7),  .a8(-d8),  .result(arc_min[152:144]));
    orb_min9_signed arc_17 (.a0(-d1),  .a1(-d2),  .a2(-d3),  .a3(-d4),  .a4(-d5),  .a5(-d6),  .a6(-d7),  .a7(-d8),  .a8(-d9),  .result(arc_min[161:153]));
    orb_min9_signed arc_18 (.a0(-d2),  .a1(-d3),  .a2(-d4),  .a3(-d5),  .a4(-d6),  .a5(-d7),  .a6(-d8),  .a7(-d9),  .a8(-d10), .result(arc_min[170:162]));
    orb_min9_signed arc_19 (.a0(-d3),  .a1(-d4),  .a2(-d5),  .a3(-d6),  .a4(-d7),  .a5(-d8),  .a6(-d9),  .a7(-d10), .a8(-d11), .result(arc_min[179:171]));
    orb_min9_signed arc_20 (.a0(-d4),  .a1(-d5),  .a2(-d6),  .a3(-d7),  .a4(-d8),  .a5(-d9),  .a6(-d10), .a7(-d11), .a8(-d12), .result(arc_min[188:180]));
    orb_min9_signed arc_21 (.a0(-d5),  .a1(-d6),  .a2(-d7),  .a3(-d8),  .a4(-d9),  .a5(-d10), .a6(-d11), .a7(-d12), .a8(-d13), .result(arc_min[197:189]));
    orb_min9_signed arc_22 (.a0(-d6),  .a1(-d7),  .a2(-d8),  .a3(-d9),  .a4(-d10), .a5(-d11), .a6(-d12), .a7(-d13), .a8(-d14), .result(arc_min[206:198]));
    orb_min9_signed arc_23 (.a0(-d7),  .a1(-d8),  .a2(-d9),  .a3(-d10), .a4(-d11), .a5(-d12), .a6(-d13), .a7(-d14), .a8(-d15), .result(arc_min[215:207]));
    orb_min9_signed arc_24 (.a0(-d8),  .a1(-d9),  .a2(-d10), .a3(-d11), .a4(-d12), .a5(-d13), .a6(-d14), .a7(-d15), .a8(-d0),  .result(arc_min[224:216]));
    orb_min9_signed arc_25 (.a0(-d9),  .a1(-d10), .a2(-d11), .a3(-d12), .a4(-d13), .a5(-d14), .a6(-d15), .a7(-d0),  .a8(-d1),  .result(arc_min[233:225]));
    orb_min9_signed arc_26 (.a0(-d10), .a1(-d11), .a2(-d12), .a3(-d13), .a4(-d14), .a5(-d15), .a6(-d0),  .a7(-d1),  .a8(-d2),  .result(arc_min[242:234]));
    orb_min9_signed arc_27 (.a0(-d11), .a1(-d12), .a2(-d13), .a3(-d14), .a4(-d15), .a5(-d0),  .a6(-d1),  .a7(-d2),  .a8(-d3),  .result(arc_min[251:243]));
    orb_min9_signed arc_28 (.a0(-d12), .a1(-d13), .a2(-d14), .a3(-d15), .a4(-d0),  .a5(-d1),  .a6(-d2),  .a7(-d3),  .a8(-d4),  .result(arc_min[260:252]));
    orb_min9_signed arc_29 (.a0(-d13), .a1(-d14), .a2(-d15), .a3(-d0),  .a4(-d1),  .a5(-d2),  .a6(-d3),  .a7(-d4),  .a8(-d5),  .result(arc_min[269:261]));
    orb_min9_signed arc_30 (.a0(-d14), .a1(-d15), .a2(-d0),  .a3(-d1),  .a4(-d2),  .a5(-d3),  .a6(-d4),  .a7(-d5),  .a8(-d6),  .result(arc_min[278:270]));
    orb_min9_signed arc_31 (.a0(-d15), .a1(-d0),  .a2(-d1),  .a3(-d2),  .a4(-d3),  .a5(-d4),  .a6(-d5),  .a7(-d6),  .a8(-d7),  .result(arc_min[287:279]));

    // ---------------------------------------------------------------
    // Stage 2 arithmetic: 8 group maxima over registered arc minima
    // ---------------------------------------------------------------
    orb_max4_signed group_0 (.a0(arc_reg[8:0]),    .a1(arc_reg[17:9]),   .a2(arc_reg[26:18]),  .a3(arc_reg[35:27]),  .result(group_max[8:0]));
    orb_max4_signed group_1 (.a0(arc_reg[44:36]),  .a1(arc_reg[53:45]),  .a2(arc_reg[62:54]),  .a3(arc_reg[71:63]),  .result(group_max[17:9]));
    orb_max4_signed group_2 (.a0(arc_reg[80:72]),  .a1(arc_reg[89:81]),  .a2(arc_reg[98:90]),  .a3(arc_reg[107:99]), .result(group_max[26:18]));
    orb_max4_signed group_3 (.a0(arc_reg[116:108]),.a1(arc_reg[125:117]),.a2(arc_reg[134:126]),.a3(arc_reg[143:135]),.result(group_max[35:27]));
    orb_max4_signed group_4 (.a0(arc_reg[152:144]),.a1(arc_reg[161:153]),.a2(arc_reg[170:162]),.a3(arc_reg[179:171]),.result(group_max[44:36]));
    orb_max4_signed group_5 (.a0(arc_reg[188:180]),.a1(arc_reg[197:189]),.a2(arc_reg[206:198]),.a3(arc_reg[215:207]),.result(group_max[53:45]));
    orb_max4_signed group_6 (.a0(arc_reg[224:216]),.a1(arc_reg[233:225]),.a2(arc_reg[242:234]),.a3(arc_reg[251:243]),.result(group_max[62:54]));
    orb_max4_signed group_7 (.a0(arc_reg[260:252]),.a1(arc_reg[269:261]),.a2(arc_reg[278:270]),.a3(arc_reg[287:279]),.result(group_max[71:63]));

    // ---------------------------------------------------------------
    // Stage 3 arithmetic: final max over the 8 registered group maxima
    // ---------------------------------------------------------------
    wire signed [8:0] best_margin;
    orb_max8_signed final_max (
        .a0(group_reg[8:0]),   .a1(group_reg[17:9]),
        .a2(group_reg[26:18]), .a3(group_reg[35:27]),
        .a4(group_reg[44:36]), .a5(group_reg[53:45]),
        .a6(group_reg[62:54]), .a7(group_reg[71:63]),
        .result(best_margin)
    );

    // ---------------------------------------------------------------
    // Pipeline registers
    // ---------------------------------------------------------------
    always @(posedge clk) begin
        if (reset || clear) begin
            valid1       <= 1'b0;
            valid2       <= 1'b0;
            score_valid  <= 1'b0;
            corner1      <= 1'b0;
            corner2      <= 1'b0;
            score_corner <= 1'b0;
            x1 <= 0; x2 <= 0; score_x <= 0;
            y1 <= 0; y2 <= 0; score_y <= 0;
            level1 <= 0; level2 <= 0; score_level <= 0;
            score  <= 8'd0;

            arc_reg   <= 288'sd0;
            group_reg <= 72'sd0;
        end else if (advance) begin
            valid1      <= fast_valid;
            valid2      <= valid1;
            score_valid <= valid2;

            if (fast_valid) begin
                corner1 <= is_corner;
                x1 <= fast_x;
                y1 <= fast_y;
                level1 <= fast_level;
                arc_reg <= arc_min;
            end
            if (valid1) begin
                corner2 <= corner1;
                x2 <= x1;
                y2 <= y1;
                level2 <= level1;
                group_reg <= group_max;
            end
            if (valid2) begin
                score_corner <= corner2;
                score_x <= x2;
                score_y <= y2;
                score_level <= level2;
                if (corner2 && best_margin > 9'sd0)
                    score <= best_margin[7:0] - 8'd1;
                else
                    score <= 8'd0;
            end
        end
    end
endmodule