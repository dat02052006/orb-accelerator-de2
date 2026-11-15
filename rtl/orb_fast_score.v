// ============================================================================
// orb_fast_score.v
// FAST-9 corner score cho ORB accelerator (Cyclone II / DE2)
//
// Vi tri trong he thong:
//   orb_fast_detector --(out_valid, is_corner, out_x, pack)--> orb_fast_score
//   orb_fast_score --(score_valid, score_x, score_value, row_start, row_done)
//                  --> candidate_buffer --> nms_top
//
// Dinh nghia score (giong OpenCV FAST score, cung dang "arc" nhu detector):
//   bright_k = max(Ik - Ip, 0)          dark_k = max(Ip - Ik, 0)
//   arc_min  = min cua 9 pixel lien tiep (vong tron 16, co wrap)
//   raw      = max cua 32 arc_min (16 arc sang + 16 arc toi)
//   score    = raw - SCORE_DEC          (SCORE_DEC = 1: threshold lon nhat ma
//                                        diem van la corner; 0: dung raw)
//   Diem la corner o nguong T  <=>  raw > T  (khop voi orb_fast_detector).
//
// Dau vao pack (136 bit) = {p15, ..., p1, p0, ip} giong orb_fast_detector.
//
// Chi phat score_valid cho corner (is_corner = 1). Candidate buffer luu thua.
//
// Khung hang cho candidate_buffer (phat cho MOI hang, ke ca hang khong co corner):
//   row_start : 1 xung, som hon score_valid dau tien cua hang 5 chu ky
//   row_done  : 1 xung, 1 chu ky sau score_valid cuoi cung cua hang
//   Gia dinh: giua 2 hang co khoang trong (in_valid = 0) it nhat 1 chu ky.
//
// Pipeline (throughput 1 cua so / chu ky), do tu in_valid:
//   S1: hieu sang / toi (32 x 8 bit)
//   S2: min 3 pixel lien tiep
//   S3: min 9 pixel lien tiep (arc_min)
//   S4: max theo nhom 4 arc (4 nhom sang + 4 nhom toi)
//   S5: max 4 nhom -> bmax, dmax
//   S6: score = max(bmax, dmax) - SCORE_DEC   (thanh ghi dau ra)
// ============================================================================
module orb_fast_score #(
    parameter COORD_W   = 10,
    parameter SCORE_DEC = 1
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // tu orb_fast_detector
    input  wire                 in_valid,
    input  wire                 in_is_corner,
    input  wire [COORD_W-1:0]   in_x,
    input  wire [135:0]         in_pack,        // {p15..p0, ip}

    // sang candidate_buffer
    output reg                  score_valid,
    output reg  [COORD_W-1:0]   score_x,
    output reg  [7:0]           score_value,
    output reg                  row_start,
    output reg                  row_done
);

    // ------------------------------------------------------------------
    // Tach pack
    // ------------------------------------------------------------------
    wire [7:0] ip = in_pack[7:0];
    wire [7:0] p0 = in_pack[15:8];
    wire [7:0] p1 = in_pack[23:16];
    wire [7:0] p2 = in_pack[31:24];
    wire [7:0] p3 = in_pack[39:32];
    wire [7:0] p4 = in_pack[47:40];
    wire [7:0] p5 = in_pack[55:48];
    wire [7:0] p6 = in_pack[63:56];
    wire [7:0] p7 = in_pack[71:64];
    wire [7:0] p8 = in_pack[79:72];
    wire [7:0] p9 = in_pack[87:80];
    wire [7:0] p10 = in_pack[95:88];
    wire [7:0] p11 = in_pack[103:96];
    wire [7:0] p12 = in_pack[111:104];
    wire [7:0] p13 = in_pack[119:112];
    wire [7:0] p14 = in_pack[127:120];
    wire [7:0] p15 = in_pack[135:128];

    // ------------------------------------------------------------------
    // Thanh ghi dieu khien (valid / corner / x) qua tung stage
    // ------------------------------------------------------------------
    reg valid_s1;
    reg valid_s2;
    reg valid_s3;
    reg valid_s4;
    reg valid_s5;
    reg valid_s6;
    reg corner_s1;
    reg [COORD_W-1:0] x_s1;
    reg corner_s2;
    reg [COORD_W-1:0] x_s2;
    reg corner_s3;
    reg [COORD_W-1:0] x_s3;
    reg corner_s4;
    reg [COORD_W-1:0] x_s4;
    reg corner_s5;
    reg [COORD_W-1:0] x_s5;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1  <= 1'b0;
            valid_s2  <= 1'b0;
            valid_s3  <= 1'b0;
            valid_s4  <= 1'b0;
            valid_s5  <= 1'b0;
            valid_s6  <= 1'b0;
            corner_s1 <= 1'b0;
            x_s1      <= {COORD_W{1'b0}};
            corner_s2 <= 1'b0;
            x_s2      <= {COORD_W{1'b0}};
            corner_s3 <= 1'b0;
            x_s3      <= {COORD_W{1'b0}};
            corner_s4 <= 1'b0;
            x_s4      <= {COORD_W{1'b0}};
            corner_s5 <= 1'b0;
            x_s5      <= {COORD_W{1'b0}};
            score_valid <= 1'b0;
            score_x     <= {COORD_W{1'b0}};
            row_start   <= 1'b0;
            row_done    <= 1'b0;
        end else begin
            valid_s1  <= in_valid;
            corner_s1 <= in_is_corner;
            x_s1      <= in_x;
            valid_s2  <= valid_s1;
            corner_s2 <= corner_s1;
            x_s2      <= x_s1;
            valid_s3  <= valid_s2;
            corner_s3 <= corner_s2;
            x_s3      <= x_s2;
            valid_s4  <= valid_s3;
            corner_s4 <= corner_s3;
            x_s4      <= x_s3;
            valid_s5  <= valid_s4;
            corner_s5 <= corner_s4;
            x_s5      <= x_s4;
            valid_s6  <= valid_s5;

            // ngo ra: chi phat cho corner
            score_valid <= valid_s5 & corner_s5;
            score_x     <= x_s5;

            // row_start: cua so dau tien cua hang vao pipeline (stage 1 dang trong)
            row_start <= in_valid & ~valid_s1;
            // row_done: phan tu cuoi cua hang vua ra khoi stage 5 -> 1 chu ky sau score cuoi
            row_done  <= valid_s6 & ~valid_s5;
        end
    end

    // ------------------------------------------------------------------
    // S1: hieu sang / toi, bao hoa ve 0
    // ------------------------------------------------------------------
    reg [7:0] db0_s1;
    reg [7:0] dd0_s1;
    reg [7:0] db1_s1;
    reg [7:0] dd1_s1;
    reg [7:0] db2_s1;
    reg [7:0] dd2_s1;
    reg [7:0] db3_s1;
    reg [7:0] dd3_s1;
    reg [7:0] db4_s1;
    reg [7:0] dd4_s1;
    reg [7:0] db5_s1;
    reg [7:0] dd5_s1;
    reg [7:0] db6_s1;
    reg [7:0] dd6_s1;
    reg [7:0] db7_s1;
    reg [7:0] dd7_s1;
    reg [7:0] db8_s1;
    reg [7:0] dd8_s1;
    reg [7:0] db9_s1;
    reg [7:0] dd9_s1;
    reg [7:0] db10_s1;
    reg [7:0] dd10_s1;
    reg [7:0] db11_s1;
    reg [7:0] dd11_s1;
    reg [7:0] db12_s1;
    reg [7:0] dd12_s1;
    reg [7:0] db13_s1;
    reg [7:0] dd13_s1;
    reg [7:0] db14_s1;
    reg [7:0] dd14_s1;
    reg [7:0] db15_s1;
    reg [7:0] dd15_s1;

    always @(posedge clk) begin
        db0_s1 <= (p0 > ip) ? (p0 - ip) : 8'd0;
        dd0_s1 <= (ip > p0) ? (ip - p0) : 8'd0;
        db1_s1 <= (p1 > ip) ? (p1 - ip) : 8'd0;
        dd1_s1 <= (ip > p1) ? (ip - p1) : 8'd0;
        db2_s1 <= (p2 > ip) ? (p2 - ip) : 8'd0;
        dd2_s1 <= (ip > p2) ? (ip - p2) : 8'd0;
        db3_s1 <= (p3 > ip) ? (p3 - ip) : 8'd0;
        dd3_s1 <= (ip > p3) ? (ip - p3) : 8'd0;
        db4_s1 <= (p4 > ip) ? (p4 - ip) : 8'd0;
        dd4_s1 <= (ip > p4) ? (ip - p4) : 8'd0;
        db5_s1 <= (p5 > ip) ? (p5 - ip) : 8'd0;
        dd5_s1 <= (ip > p5) ? (ip - p5) : 8'd0;
        db6_s1 <= (p6 > ip) ? (p6 - ip) : 8'd0;
        dd6_s1 <= (ip > p6) ? (ip - p6) : 8'd0;
        db7_s1 <= (p7 > ip) ? (p7 - ip) : 8'd0;
        dd7_s1 <= (ip > p7) ? (ip - p7) : 8'd0;
        db8_s1 <= (p8 > ip) ? (p8 - ip) : 8'd0;
        dd8_s1 <= (ip > p8) ? (ip - p8) : 8'd0;
        db9_s1 <= (p9 > ip) ? (p9 - ip) : 8'd0;
        dd9_s1 <= (ip > p9) ? (ip - p9) : 8'd0;
        db10_s1 <= (p10 > ip) ? (p10 - ip) : 8'd0;
        dd10_s1 <= (ip > p10) ? (ip - p10) : 8'd0;
        db11_s1 <= (p11 > ip) ? (p11 - ip) : 8'd0;
        dd11_s1 <= (ip > p11) ? (ip - p11) : 8'd0;
        db12_s1 <= (p12 > ip) ? (p12 - ip) : 8'd0;
        dd12_s1 <= (ip > p12) ? (ip - p12) : 8'd0;
        db13_s1 <= (p13 > ip) ? (p13 - ip) : 8'd0;
        dd13_s1 <= (ip > p13) ? (ip - p13) : 8'd0;
        db14_s1 <= (p14 > ip) ? (p14 - ip) : 8'd0;
        dd14_s1 <= (ip > p14) ? (ip - p14) : 8'd0;
        db15_s1 <= (p15 > ip) ? (p15 - ip) : 8'd0;
        dd15_s1 <= (ip > p15) ? (ip - p15) : 8'd0;
    end

    // ------------------------------------------------------------------
    // S2: m3[i] = min(d[i], d[i+1], d[i+2])  (wrap 16)
    // ------------------------------------------------------------------
    wire [7:0] b3a0 = (db0_s1 < db1_s1) ? db0_s1 : db1_s1;
    wire [7:0] b3w0 = (b3a0 < db2_s1) ? b3a0 : db2_s1;
    wire [7:0] b3a1 = (db1_s1 < db2_s1) ? db1_s1 : db2_s1;
    wire [7:0] b3w1 = (b3a1 < db3_s1) ? b3a1 : db3_s1;
    wire [7:0] b3a2 = (db2_s1 < db3_s1) ? db2_s1 : db3_s1;
    wire [7:0] b3w2 = (b3a2 < db4_s1) ? b3a2 : db4_s1;
    wire [7:0] b3a3 = (db3_s1 < db4_s1) ? db3_s1 : db4_s1;
    wire [7:0] b3w3 = (b3a3 < db5_s1) ? b3a3 : db5_s1;
    wire [7:0] b3a4 = (db4_s1 < db5_s1) ? db4_s1 : db5_s1;
    wire [7:0] b3w4 = (b3a4 < db6_s1) ? b3a4 : db6_s1;
    wire [7:0] b3a5 = (db5_s1 < db6_s1) ? db5_s1 : db6_s1;
    wire [7:0] b3w5 = (b3a5 < db7_s1) ? b3a5 : db7_s1;
    wire [7:0] b3a6 = (db6_s1 < db7_s1) ? db6_s1 : db7_s1;
    wire [7:0] b3w6 = (b3a6 < db8_s1) ? b3a6 : db8_s1;
    wire [7:0] b3a7 = (db7_s1 < db8_s1) ? db7_s1 : db8_s1;
    wire [7:0] b3w7 = (b3a7 < db9_s1) ? b3a7 : db9_s1;
    wire [7:0] b3a8 = (db8_s1 < db9_s1) ? db8_s1 : db9_s1;
    wire [7:0] b3w8 = (b3a8 < db10_s1) ? b3a8 : db10_s1;
    wire [7:0] b3a9 = (db9_s1 < db10_s1) ? db9_s1 : db10_s1;
    wire [7:0] b3w9 = (b3a9 < db11_s1) ? b3a9 : db11_s1;
    wire [7:0] b3a10 = (db10_s1 < db11_s1) ? db10_s1 : db11_s1;
    wire [7:0] b3w10 = (b3a10 < db12_s1) ? b3a10 : db12_s1;
    wire [7:0] b3a11 = (db11_s1 < db12_s1) ? db11_s1 : db12_s1;
    wire [7:0] b3w11 = (b3a11 < db13_s1) ? b3a11 : db13_s1;
    wire [7:0] b3a12 = (db12_s1 < db13_s1) ? db12_s1 : db13_s1;
    wire [7:0] b3w12 = (b3a12 < db14_s1) ? b3a12 : db14_s1;
    wire [7:0] b3a13 = (db13_s1 < db14_s1) ? db13_s1 : db14_s1;
    wire [7:0] b3w13 = (b3a13 < db15_s1) ? b3a13 : db15_s1;
    wire [7:0] b3a14 = (db14_s1 < db15_s1) ? db14_s1 : db15_s1;
    wire [7:0] b3w14 = (b3a14 < db0_s1) ? b3a14 : db0_s1;
    wire [7:0] b3a15 = (db15_s1 < db0_s1) ? db15_s1 : db0_s1;
    wire [7:0] b3w15 = (b3a15 < db1_s1) ? b3a15 : db1_s1;
    wire [7:0] d3a0 = (dd0_s1 < dd1_s1) ? dd0_s1 : dd1_s1;
    wire [7:0] d3w0 = (d3a0 < dd2_s1) ? d3a0 : dd2_s1;
    wire [7:0] d3a1 = (dd1_s1 < dd2_s1) ? dd1_s1 : dd2_s1;
    wire [7:0] d3w1 = (d3a1 < dd3_s1) ? d3a1 : dd3_s1;
    wire [7:0] d3a2 = (dd2_s1 < dd3_s1) ? dd2_s1 : dd3_s1;
    wire [7:0] d3w2 = (d3a2 < dd4_s1) ? d3a2 : dd4_s1;
    wire [7:0] d3a3 = (dd3_s1 < dd4_s1) ? dd3_s1 : dd4_s1;
    wire [7:0] d3w3 = (d3a3 < dd5_s1) ? d3a3 : dd5_s1;
    wire [7:0] d3a4 = (dd4_s1 < dd5_s1) ? dd4_s1 : dd5_s1;
    wire [7:0] d3w4 = (d3a4 < dd6_s1) ? d3a4 : dd6_s1;
    wire [7:0] d3a5 = (dd5_s1 < dd6_s1) ? dd5_s1 : dd6_s1;
    wire [7:0] d3w5 = (d3a5 < dd7_s1) ? d3a5 : dd7_s1;
    wire [7:0] d3a6 = (dd6_s1 < dd7_s1) ? dd6_s1 : dd7_s1;
    wire [7:0] d3w6 = (d3a6 < dd8_s1) ? d3a6 : dd8_s1;
    wire [7:0] d3a7 = (dd7_s1 < dd8_s1) ? dd7_s1 : dd8_s1;
    wire [7:0] d3w7 = (d3a7 < dd9_s1) ? d3a7 : dd9_s1;
    wire [7:0] d3a8 = (dd8_s1 < dd9_s1) ? dd8_s1 : dd9_s1;
    wire [7:0] d3w8 = (d3a8 < dd10_s1) ? d3a8 : dd10_s1;
    wire [7:0] d3a9 = (dd9_s1 < dd10_s1) ? dd9_s1 : dd10_s1;
    wire [7:0] d3w9 = (d3a9 < dd11_s1) ? d3a9 : dd11_s1;
    wire [7:0] d3a10 = (dd10_s1 < dd11_s1) ? dd10_s1 : dd11_s1;
    wire [7:0] d3w10 = (d3a10 < dd12_s1) ? d3a10 : dd12_s1;
    wire [7:0] d3a11 = (dd11_s1 < dd12_s1) ? dd11_s1 : dd12_s1;
    wire [7:0] d3w11 = (d3a11 < dd13_s1) ? d3a11 : dd13_s1;
    wire [7:0] d3a12 = (dd12_s1 < dd13_s1) ? dd12_s1 : dd13_s1;
    wire [7:0] d3w12 = (d3a12 < dd14_s1) ? d3a12 : dd14_s1;
    wire [7:0] d3a13 = (dd13_s1 < dd14_s1) ? dd13_s1 : dd14_s1;
    wire [7:0] d3w13 = (d3a13 < dd15_s1) ? d3a13 : dd15_s1;
    wire [7:0] d3a14 = (dd14_s1 < dd15_s1) ? dd14_s1 : dd15_s1;
    wire [7:0] d3w14 = (d3a14 < dd0_s1) ? d3a14 : dd0_s1;
    wire [7:0] d3a15 = (dd15_s1 < dd0_s1) ? dd15_s1 : dd0_s1;
    wire [7:0] d3w15 = (d3a15 < dd1_s1) ? d3a15 : dd1_s1;

    reg [7:0] mb3_0_s2;
    reg [7:0] mb3_1_s2;
    reg [7:0] mb3_2_s2;
    reg [7:0] mb3_3_s2;
    reg [7:0] mb3_4_s2;
    reg [7:0] mb3_5_s2;
    reg [7:0] mb3_6_s2;
    reg [7:0] mb3_7_s2;
    reg [7:0] mb3_8_s2;
    reg [7:0] mb3_9_s2;
    reg [7:0] mb3_10_s2;
    reg [7:0] mb3_11_s2;
    reg [7:0] mb3_12_s2;
    reg [7:0] mb3_13_s2;
    reg [7:0] mb3_14_s2;
    reg [7:0] mb3_15_s2;
    reg [7:0] md3_0_s2;
    reg [7:0] md3_1_s2;
    reg [7:0] md3_2_s2;
    reg [7:0] md3_3_s2;
    reg [7:0] md3_4_s2;
    reg [7:0] md3_5_s2;
    reg [7:0] md3_6_s2;
    reg [7:0] md3_7_s2;
    reg [7:0] md3_8_s2;
    reg [7:0] md3_9_s2;
    reg [7:0] md3_10_s2;
    reg [7:0] md3_11_s2;
    reg [7:0] md3_12_s2;
    reg [7:0] md3_13_s2;
    reg [7:0] md3_14_s2;
    reg [7:0] md3_15_s2;

    always @(posedge clk) begin
        mb3_0_s2 <= b3w0;
        mb3_1_s2 <= b3w1;
        mb3_2_s2 <= b3w2;
        mb3_3_s2 <= b3w3;
        mb3_4_s2 <= b3w4;
        mb3_5_s2 <= b3w5;
        mb3_6_s2 <= b3w6;
        mb3_7_s2 <= b3w7;
        mb3_8_s2 <= b3w8;
        mb3_9_s2 <= b3w9;
        mb3_10_s2 <= b3w10;
        mb3_11_s2 <= b3w11;
        mb3_12_s2 <= b3w12;
        mb3_13_s2 <= b3w13;
        mb3_14_s2 <= b3w14;
        mb3_15_s2 <= b3w15;
        md3_0_s2 <= d3w0;
        md3_1_s2 <= d3w1;
        md3_2_s2 <= d3w2;
        md3_3_s2 <= d3w3;
        md3_4_s2 <= d3w4;
        md3_5_s2 <= d3w5;
        md3_6_s2 <= d3w6;
        md3_7_s2 <= d3w7;
        md3_8_s2 <= d3w8;
        md3_9_s2 <= d3w9;
        md3_10_s2 <= d3w10;
        md3_11_s2 <= d3w11;
        md3_12_s2 <= d3w12;
        md3_13_s2 <= d3w13;
        md3_14_s2 <= d3w14;
        md3_15_s2 <= d3w15;
    end

    // ------------------------------------------------------------------
    // S3: m9[i] = min(m3[i], m3[i+3], m3[i+6])  = min cua 9 pixel i..i+8
    // ------------------------------------------------------------------
    wire [7:0] b9a0 = (mb3_0_s2 < mb3_3_s2) ? mb3_0_s2 : mb3_3_s2;
    wire [7:0] b9w0 = (b9a0 < mb3_6_s2) ? b9a0 : mb3_6_s2;
    wire [7:0] b9a1 = (mb3_1_s2 < mb3_4_s2) ? mb3_1_s2 : mb3_4_s2;
    wire [7:0] b9w1 = (b9a1 < mb3_7_s2) ? b9a1 : mb3_7_s2;
    wire [7:0] b9a2 = (mb3_2_s2 < mb3_5_s2) ? mb3_2_s2 : mb3_5_s2;
    wire [7:0] b9w2 = (b9a2 < mb3_8_s2) ? b9a2 : mb3_8_s2;
    wire [7:0] b9a3 = (mb3_3_s2 < mb3_6_s2) ? mb3_3_s2 : mb3_6_s2;
    wire [7:0] b9w3 = (b9a3 < mb3_9_s2) ? b9a3 : mb3_9_s2;
    wire [7:0] b9a4 = (mb3_4_s2 < mb3_7_s2) ? mb3_4_s2 : mb3_7_s2;
    wire [7:0] b9w4 = (b9a4 < mb3_10_s2) ? b9a4 : mb3_10_s2;
    wire [7:0] b9a5 = (mb3_5_s2 < mb3_8_s2) ? mb3_5_s2 : mb3_8_s2;
    wire [7:0] b9w5 = (b9a5 < mb3_11_s2) ? b9a5 : mb3_11_s2;
    wire [7:0] b9a6 = (mb3_6_s2 < mb3_9_s2) ? mb3_6_s2 : mb3_9_s2;
    wire [7:0] b9w6 = (b9a6 < mb3_12_s2) ? b9a6 : mb3_12_s2;
    wire [7:0] b9a7 = (mb3_7_s2 < mb3_10_s2) ? mb3_7_s2 : mb3_10_s2;
    wire [7:0] b9w7 = (b9a7 < mb3_13_s2) ? b9a7 : mb3_13_s2;
    wire [7:0] b9a8 = (mb3_8_s2 < mb3_11_s2) ? mb3_8_s2 : mb3_11_s2;
    wire [7:0] b9w8 = (b9a8 < mb3_14_s2) ? b9a8 : mb3_14_s2;
    wire [7:0] b9a9 = (mb3_9_s2 < mb3_12_s2) ? mb3_9_s2 : mb3_12_s2;
    wire [7:0] b9w9 = (b9a9 < mb3_15_s2) ? b9a9 : mb3_15_s2;
    wire [7:0] b9a10 = (mb3_10_s2 < mb3_13_s2) ? mb3_10_s2 : mb3_13_s2;
    wire [7:0] b9w10 = (b9a10 < mb3_0_s2) ? b9a10 : mb3_0_s2;
    wire [7:0] b9a11 = (mb3_11_s2 < mb3_14_s2) ? mb3_11_s2 : mb3_14_s2;
    wire [7:0] b9w11 = (b9a11 < mb3_1_s2) ? b9a11 : mb3_1_s2;
    wire [7:0] b9a12 = (mb3_12_s2 < mb3_15_s2) ? mb3_12_s2 : mb3_15_s2;
    wire [7:0] b9w12 = (b9a12 < mb3_2_s2) ? b9a12 : mb3_2_s2;
    wire [7:0] b9a13 = (mb3_13_s2 < mb3_0_s2) ? mb3_13_s2 : mb3_0_s2;
    wire [7:0] b9w13 = (b9a13 < mb3_3_s2) ? b9a13 : mb3_3_s2;
    wire [7:0] b9a14 = (mb3_14_s2 < mb3_1_s2) ? mb3_14_s2 : mb3_1_s2;
    wire [7:0] b9w14 = (b9a14 < mb3_4_s2) ? b9a14 : mb3_4_s2;
    wire [7:0] b9a15 = (mb3_15_s2 < mb3_2_s2) ? mb3_15_s2 : mb3_2_s2;
    wire [7:0] b9w15 = (b9a15 < mb3_5_s2) ? b9a15 : mb3_5_s2;
    wire [7:0] d9a0 = (md3_0_s2 < md3_3_s2) ? md3_0_s2 : md3_3_s2;
    wire [7:0] d9w0 = (d9a0 < md3_6_s2) ? d9a0 : md3_6_s2;
    wire [7:0] d9a1 = (md3_1_s2 < md3_4_s2) ? md3_1_s2 : md3_4_s2;
    wire [7:0] d9w1 = (d9a1 < md3_7_s2) ? d9a1 : md3_7_s2;
    wire [7:0] d9a2 = (md3_2_s2 < md3_5_s2) ? md3_2_s2 : md3_5_s2;
    wire [7:0] d9w2 = (d9a2 < md3_8_s2) ? d9a2 : md3_8_s2;
    wire [7:0] d9a3 = (md3_3_s2 < md3_6_s2) ? md3_3_s2 : md3_6_s2;
    wire [7:0] d9w3 = (d9a3 < md3_9_s2) ? d9a3 : md3_9_s2;
    wire [7:0] d9a4 = (md3_4_s2 < md3_7_s2) ? md3_4_s2 : md3_7_s2;
    wire [7:0] d9w4 = (d9a4 < md3_10_s2) ? d9a4 : md3_10_s2;
    wire [7:0] d9a5 = (md3_5_s2 < md3_8_s2) ? md3_5_s2 : md3_8_s2;
    wire [7:0] d9w5 = (d9a5 < md3_11_s2) ? d9a5 : md3_11_s2;
    wire [7:0] d9a6 = (md3_6_s2 < md3_9_s2) ? md3_6_s2 : md3_9_s2;
    wire [7:0] d9w6 = (d9a6 < md3_12_s2) ? d9a6 : md3_12_s2;
    wire [7:0] d9a7 = (md3_7_s2 < md3_10_s2) ? md3_7_s2 : md3_10_s2;
    wire [7:0] d9w7 = (d9a7 < md3_13_s2) ? d9a7 : md3_13_s2;
    wire [7:0] d9a8 = (md3_8_s2 < md3_11_s2) ? md3_8_s2 : md3_11_s2;
    wire [7:0] d9w8 = (d9a8 < md3_14_s2) ? d9a8 : md3_14_s2;
    wire [7:0] d9a9 = (md3_9_s2 < md3_12_s2) ? md3_9_s2 : md3_12_s2;
    wire [7:0] d9w9 = (d9a9 < md3_15_s2) ? d9a9 : md3_15_s2;
    wire [7:0] d9a10 = (md3_10_s2 < md3_13_s2) ? md3_10_s2 : md3_13_s2;
    wire [7:0] d9w10 = (d9a10 < md3_0_s2) ? d9a10 : md3_0_s2;
    wire [7:0] d9a11 = (md3_11_s2 < md3_14_s2) ? md3_11_s2 : md3_14_s2;
    wire [7:0] d9w11 = (d9a11 < md3_1_s2) ? d9a11 : md3_1_s2;
    wire [7:0] d9a12 = (md3_12_s2 < md3_15_s2) ? md3_12_s2 : md3_15_s2;
    wire [7:0] d9w12 = (d9a12 < md3_2_s2) ? d9a12 : md3_2_s2;
    wire [7:0] d9a13 = (md3_13_s2 < md3_0_s2) ? md3_13_s2 : md3_0_s2;
    wire [7:0] d9w13 = (d9a13 < md3_3_s2) ? d9a13 : md3_3_s2;
    wire [7:0] d9a14 = (md3_14_s2 < md3_1_s2) ? md3_14_s2 : md3_1_s2;
    wire [7:0] d9w14 = (d9a14 < md3_4_s2) ? d9a14 : md3_4_s2;
    wire [7:0] d9a15 = (md3_15_s2 < md3_2_s2) ? md3_15_s2 : md3_2_s2;
    wire [7:0] d9w15 = (d9a15 < md3_5_s2) ? d9a15 : md3_5_s2;

    reg [7:0] mb9_0_s3;
    reg [7:0] mb9_1_s3;
    reg [7:0] mb9_2_s3;
    reg [7:0] mb9_3_s3;
    reg [7:0] mb9_4_s3;
    reg [7:0] mb9_5_s3;
    reg [7:0] mb9_6_s3;
    reg [7:0] mb9_7_s3;
    reg [7:0] mb9_8_s3;
    reg [7:0] mb9_9_s3;
    reg [7:0] mb9_10_s3;
    reg [7:0] mb9_11_s3;
    reg [7:0] mb9_12_s3;
    reg [7:0] mb9_13_s3;
    reg [7:0] mb9_14_s3;
    reg [7:0] mb9_15_s3;
    reg [7:0] md9_0_s3;
    reg [7:0] md9_1_s3;
    reg [7:0] md9_2_s3;
    reg [7:0] md9_3_s3;
    reg [7:0] md9_4_s3;
    reg [7:0] md9_5_s3;
    reg [7:0] md9_6_s3;
    reg [7:0] md9_7_s3;
    reg [7:0] md9_8_s3;
    reg [7:0] md9_9_s3;
    reg [7:0] md9_10_s3;
    reg [7:0] md9_11_s3;
    reg [7:0] md9_12_s3;
    reg [7:0] md9_13_s3;
    reg [7:0] md9_14_s3;
    reg [7:0] md9_15_s3;

    always @(posedge clk) begin
        mb9_0_s3 <= b9w0;
        mb9_1_s3 <= b9w1;
        mb9_2_s3 <= b9w2;
        mb9_3_s3 <= b9w3;
        mb9_4_s3 <= b9w4;
        mb9_5_s3 <= b9w5;
        mb9_6_s3 <= b9w6;
        mb9_7_s3 <= b9w7;
        mb9_8_s3 <= b9w8;
        mb9_9_s3 <= b9w9;
        mb9_10_s3 <= b9w10;
        mb9_11_s3 <= b9w11;
        mb9_12_s3 <= b9w12;
        mb9_13_s3 <= b9w13;
        mb9_14_s3 <= b9w14;
        mb9_15_s3 <= b9w15;
        md9_0_s3 <= d9w0;
        md9_1_s3 <= d9w1;
        md9_2_s3 <= d9w2;
        md9_3_s3 <= d9w3;
        md9_4_s3 <= d9w4;
        md9_5_s3 <= d9w5;
        md9_6_s3 <= d9w6;
        md9_7_s3 <= d9w7;
        md9_8_s3 <= d9w8;
        md9_9_s3 <= d9w9;
        md9_10_s3 <= d9w10;
        md9_11_s3 <= d9w11;
        md9_12_s3 <= d9w12;
        md9_13_s3 <= d9w13;
        md9_14_s3 <= d9w14;
        md9_15_s3 <= d9w15;
    end

    // ------------------------------------------------------------------
    // S4: max theo nhom 4 arc
    // ------------------------------------------------------------------
    wire [7:0] bg0a = (mb9_0_s3 > mb9_1_s3) ? mb9_0_s3 : mb9_1_s3;
    wire [7:0] bg0b = (mb9_2_s3 > mb9_3_s3) ? mb9_2_s3 : mb9_3_s3;
    wire [7:0] bg0w = (bg0a > bg0b) ? bg0a : bg0b;
    wire [7:0] bg1a = (mb9_4_s3 > mb9_5_s3) ? mb9_4_s3 : mb9_5_s3;
    wire [7:0] bg1b = (mb9_6_s3 > mb9_7_s3) ? mb9_6_s3 : mb9_7_s3;
    wire [7:0] bg1w = (bg1a > bg1b) ? bg1a : bg1b;
    wire [7:0] bg2a = (mb9_8_s3 > mb9_9_s3) ? mb9_8_s3 : mb9_9_s3;
    wire [7:0] bg2b = (mb9_10_s3 > mb9_11_s3) ? mb9_10_s3 : mb9_11_s3;
    wire [7:0] bg2w = (bg2a > bg2b) ? bg2a : bg2b;
    wire [7:0] bg3a = (mb9_12_s3 > mb9_13_s3) ? mb9_12_s3 : mb9_13_s3;
    wire [7:0] bg3b = (mb9_14_s3 > mb9_15_s3) ? mb9_14_s3 : mb9_15_s3;
    wire [7:0] bg3w = (bg3a > bg3b) ? bg3a : bg3b;
    wire [7:0] dg0a = (md9_0_s3 > md9_1_s3) ? md9_0_s3 : md9_1_s3;
    wire [7:0] dg0b = (md9_2_s3 > md9_3_s3) ? md9_2_s3 : md9_3_s3;
    wire [7:0] dg0w = (dg0a > dg0b) ? dg0a : dg0b;
    wire [7:0] dg1a = (md9_4_s3 > md9_5_s3) ? md9_4_s3 : md9_5_s3;
    wire [7:0] dg1b = (md9_6_s3 > md9_7_s3) ? md9_6_s3 : md9_7_s3;
    wire [7:0] dg1w = (dg1a > dg1b) ? dg1a : dg1b;
    wire [7:0] dg2a = (md9_8_s3 > md9_9_s3) ? md9_8_s3 : md9_9_s3;
    wire [7:0] dg2b = (md9_10_s3 > md9_11_s3) ? md9_10_s3 : md9_11_s3;
    wire [7:0] dg2w = (dg2a > dg2b) ? dg2a : dg2b;
    wire [7:0] dg3a = (md9_12_s3 > md9_13_s3) ? md9_12_s3 : md9_13_s3;
    wire [7:0] dg3b = (md9_14_s3 > md9_15_s3) ? md9_14_s3 : md9_15_s3;
    wire [7:0] dg3w = (dg3a > dg3b) ? dg3a : dg3b;

    reg [7:0] gb0_s4;
    reg [7:0] gb1_s4;
    reg [7:0] gb2_s4;
    reg [7:0] gb3_s4;
    reg [7:0] gd0_s4;
    reg [7:0] gd1_s4;
    reg [7:0] gd2_s4;
    reg [7:0] gd3_s4;

    always @(posedge clk) begin
        gb0_s4 <= bg0w;
        gb1_s4 <= bg1w;
        gb2_s4 <= bg2w;
        gb3_s4 <= bg3w;
        gd0_s4 <= dg0w;
        gd1_s4 <= dg1w;
        gd2_s4 <= dg2w;
        gd3_s4 <= dg3w;
    end

    // ------------------------------------------------------------------
    // S5: max 4 nhom -> bmax / dmax
    // ------------------------------------------------------------------
    wire [7:0] bma = (gb0_s4 > gb1_s4) ? gb0_s4 : gb1_s4;
    wire [7:0] bmb = (gb2_s4 > gb3_s4) ? gb2_s4 : gb3_s4;
    wire [7:0] bmw = (bma > bmb) ? bma : bmb;
    wire [7:0] dma = (gd0_s4 > gd1_s4) ? gd0_s4 : gd1_s4;
    wire [7:0] dmb = (gd2_s4 > gd3_s4) ? gd2_s4 : gd3_s4;
    wire [7:0] dmw = (dma > dmb) ? dma : dmb;

    reg [7:0] bmax_s5;
    reg [7:0] dmax_s5;

    always @(posedge clk) begin
        bmax_s5 <= bmw;
        dmax_s5 <= dmw;
    end

    // ------------------------------------------------------------------
    // S6: score = max(bmax, dmax) - SCORE_DEC  (khong am)
    // ------------------------------------------------------------------
    wire [7:0] raw_w   = (bmax_s5 > dmax_s5) ? bmax_s5 : dmax_s5;
    wire [7:0] score_w = ((SCORE_DEC != 0) && (raw_w != 8'd0)) ? (raw_w - 8'd1) : raw_w;

    always @(posedge clk) begin
        score_value <= score_w;
    end

endmodule
