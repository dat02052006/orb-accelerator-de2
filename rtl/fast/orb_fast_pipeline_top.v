// ============================================================================
// orb_fast_pipeline_top.v
// Top-level ket noi pipeline phat hien va tinh diem dac trung FAST-9:
//   image_reader <-> orb_line_buffer_ctrl <-> orb_line_buffer
//                --> orb_window_generator --> orb_fast_detector
//                --> orb_fast_score --> (candidate_buffer)
// ============================================================================

module orb_fast_pipeline_top #(
    parameter IMG_W      = 640,
    parameter IMG_H      = 480,
    parameter BORDER     = 21,
    parameter COORD_W    = 10,
    parameter COL_OFFSET = 10'd18,
    parameter SCORE_DEC  = 1
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // ---- Dieu khien & Cau hinh ----
    input  wire                 start,          // Bat dau xu ly frame
    input  wire [7:0]           threshold,      // Nguong FAST T
    output wire                 busy,
    output wire                 frame_done,     // Xung bao xong 1 frame
    output wire                 error,          // Bao loi tu image_reader

    // ---- Giao tiep voi image_reader ----
    output wire                 ir_start,
    output wire [8:0]           ir_first_row,
    output wire [4:0]           ir_first_bank,
    output wire [4:0]           ir_num_rows,
    input  wire                 ir_done,
    input  wire                 ir_error,

    // Cong ghi tu image_reader vao line buffer
    input  wire                 lb_we,
    input  wire [4:0]           lb_row,
    input  wire [8:0]           lb_col_word,
    input  wire [15:0]          lb_wdata,

    // ---- Dau ra sang candidate_buffer ----
    output wire                 score_valid,
    output wire [COORD_W-1:0]   score_x,
    output wire [COORD_W-1:0]   score_y,
    output wire [7:0]           score_value,
    output wire                 row_start,
    output wire                 row_done
);

    // ========================================================================
    // Day noi noi bo (Internal Wires)
    // ========================================================================

    // 1. ctrl <-> line_buffer (cong doc)
    wire        lb_rd_en;
    wire [9:0]  lb_rd_col;
    wire        lb_rd_valid;
    wire [7:0]  lb_p0, lb_p1, lb_p2, lb_p3, lb_p4, lb_p5, lb_p6;
    wire [8:0]  ctrl_center_y;
    wire        ctrl_row_done;

    // 2. ctrl <-> window_generator
    wire        win_clear;
    wire        win_in_valid;
    wire [7:0]  win_p0, win_p1, win_p2, win_p3, win_p4, win_p5, win_p6;

    // 3. window_generator <-> fast_detector
    wire        window_valid;
    wire [9:0]  win_center_x;
    wire [391:0] window_flat; // Khong dung truc tiep nhung noi ra de tranh warning

    wire [7:0]  w00, w01, w02, w03, w04, w05, w06;
    wire [7:0]  w10, w11, w12, w13, w14, w15, w16;
    wire [7:0]  w20, w21, w22, w23, w24, w25, w26;
    wire [7:0]  w30, w31, w32, w33, w34, w35, w36;
    wire [7:0]  w40, w41, w42, w43, w44, w45, w46;
    wire [7:0]  w50, w51, w52, w53, w54, w55, w56;
    wire [7:0]  w60, w61, w62, w63, w64, w65, w66;

    // 4. fast_detector <-> fast_score
    wire                det_out_valid;
    wire                det_is_corner;
    wire [COORD_W-1:0]  det_out_x;
    wire [COORD_W-1:0]  det_out_y;
    wire [135:0]        det_pack;

    // ========================================================================
    // Khoi 1: Bo dieu khien Line Buffer (FSM)
    // ========================================================================
    orb_line_buffer_ctrl u_line_buffer_ctrl (
        .clk           (clk),
        .rst_n         (rst_n),
        // Dieu khien chung
        .start         (start),
        .busy          (busy),
        .frame_done    (frame_done),
        .error         (error),
        .row_done      (ctrl_row_done),
        .center_y      (ctrl_center_y),
        // Sang image_reader
        .ir_start      (ir_start),
        .ir_first_row  (ir_first_row),
        .ir_first_bank (ir_first_bank),
        .ir_num_rows   (ir_num_rows),
        .ir_done       (ir_done),
        .ir_error      (ir_error),
        // Sang orb_line_buffer (cong doc)
        .lb_rd_en      (lb_rd_en),
        .lb_rd_col     (lb_rd_col),
        .lb_rd_valid   (lb_rd_valid),
        .lb_p0         (lb_p0),
        .lb_p1         (lb_p1),
        .lb_p2         (lb_p2),
        .lb_p3         (lb_p3),
        .lb_p4         (lb_p4),
        .lb_p5         (lb_p5),
        .lb_p6         (lb_p6),
        // Sang orb_window_generator
        .win_clear     (win_clear),
        .win_in_valid  (win_in_valid),
        .win_p0        (win_p0),
        .win_p1        (win_p1),
        .win_p2        (win_p2),
        .win_p3        (win_p3),
        .win_p4        (win_p4),
        .win_p5        (win_p5),
        .win_p6        (win_p6)
    );

    // ========================================================================
    // Khoi 2: Bo dem 7 hang (M4K RAM)
    // ========================================================================
    orb_line_buffer u_line_buffer (
        .clk         (clk),
        // Cong ghi tu image_reader
        .lb_we       (lb_we),
        .lb_row      (lb_row),
        .lb_col_word (lb_col_word),
        .lb_wdata    (lb_wdata),
        // Cong doc sang ctrl
        .rd_en       (lb_rd_en),
        .rd_col      (lb_rd_col),
        .rd_valid    (lb_rd_valid),
        .rd_p0       (lb_p0),
        .rd_p1       (lb_p1),
        .rd_p2       (lb_p2),
        .rd_p3       (lb_p3),
        .rd_p4       (lb_p4),
        .rd_p5       (lb_p5),
        .rd_p6       (lb_p6)
    );

    // ========================================================================
    // Khoi 3: Bo tao cua so truot 7x7
    // ========================================================================
    orb_window_generator #(
        .COL_OFFSET (COL_OFFSET)
    ) u_window_generator (
        .clk          (clk),
        .rst_n        (rst_n),
        .clear        (win_clear),
        .in_valid     (win_in_valid),
        .in_p0        (win_p0),
        .in_p1        (win_p1),
        .in_p2        (win_p2),
        .in_p3        (win_p3),
        .in_p4        (win_p4),
        .in_p5        (win_p5),
        .in_p6        (win_p6),
        .window_valid (window_valid),
        .win_center_x (win_center_x),
        .window_flat  (window_flat),
        // 49 thanh ghi cua so
        .w00(w00), .w01(w01), .w02(w02), .w03(w03), .w04(w04), .w05(w05), .w06(w06),
        .w10(w10), .w11(w11), .w12(w12), .w13(w13), .w14(w14), .w15(w15), .w16(w16),
        .w20(w20), .w21(w21), .w22(w22), .w23(w23), .w24(w24), .w25(w25), .w26(w26),
        .w30(w30), .w31(w31), .w32(w32), .w33(w33), .w34(w34), .w35(w35), .w36(w36),
        .w40(w40), .w41(w41), .w42(w42), .w43(w43), .w44(w44), .w45(w45), .w46(w46),
        .w50(w50), .w51(w51), .w52(w52), .w53(w53), .w54(w54), .w55(w55), .w56(w56),
        .w60(w60), .w61(w61), .w62(w62), .w63(w63), .w64(w64), .w65(w65), .w66(w66)
    );

    // ========================================================================
    // Khoi 4: Bo phat hien goc FAST-9 (4 stage pipeline)
    // ========================================================================
    orb_fast_detector #(
        .IMG_W   (IMG_W),
        .IMG_H   (IMG_H),
        .BORDER  (BORDER),
        .COORD_W (COORD_W)
    ) u_fast_detector (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (window_valid),
        .in_x      (win_center_x),
        .in_y      ({1'b0, ctrl_center_y}),
        .threshold (threshold),
        // Pixel tam (3,3)
        .ip        (w33),
        // 16 pixel tren duong tron ban kinh 3 theo chieu kim dong ho
        .p0        (w03), // ( 0, -3)
        .p1        (w04), // ( 1, -3)
        .p2        (w15), // ( 2, -2)
        .p3        (w26), // ( 3, -1)
        .p4        (w36), // ( 3,  0)
        .p5        (w46), // ( 3,  1)
        .p6        (w55), // ( 2,  2)
        .p7        (w64), // ( 1,  3)
        .p8        (w63), // ( 0,  3)
        .p9        (w62), // (-1,  3)
        .p10       (w51), // (-2,  2)
        .p11       (w40), // (-3,  1)
        .p12       (w30), // (-3,  0)
        .p13       (w20), // (-3, -1)
        .p14       (w11), // (-2, -2)
        .p15       (w02), // (-1, -3)
        // Dau ra sang fast_score
        .out_valid (det_out_valid),
        .is_corner (det_is_corner),
        .out_x     (det_out_x),
        .out_y     (det_out_y),
        .pack      (det_pack)
    );

    // ========================================================================
    // Khoi 5: Bo tinh diem goc FAST-9 (6 stage pipeline)
    // ========================================================================
    orb_fast_score #(
        .COORD_W   (COORD_W),
        .SCORE_DEC (SCORE_DEC)
    ) u_fast_score (
        .clk          (clk),
        .rst_n        (rst_n),
        .in_valid     (det_out_valid),
        .in_is_corner (det_is_corner),
        .in_x         (det_out_x),
        .in_pack      (det_pack),
        .score_valid  (score_valid),
        .score_x      (score_x),
        .score_value  (score_value),
        .row_start    (row_start),
        .row_done     (row_done)
    );

    // ========================================================================
    // Chot toa do Y cho tung hang quet
    // ========================================================================
    // Giu on dinh center_y trong suot qua trinh pipeline detector + score
    // xu ly het hang do (chi cap nhat khi row_start bat dau hang moi).
    reg [COORD_W-1:0] row_y_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_y_reg <= {COORD_W{1'b0}};
        end else if (row_start) begin
            row_y_reg <= det_out_y;
        end
    end

    assign score_y = row_y_reg;

endmodule