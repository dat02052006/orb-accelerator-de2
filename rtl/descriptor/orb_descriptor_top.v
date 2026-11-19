// ============================================================================
// orb_descriptor_top
//   Top-module boc toan bo pipeline tinh huong (Intensity Centroid) va
//   mo ta dac trung (rBRIEF 256-bit) de tong hop va kiem chung Fmax.
//   Yeu cau: Dat cung thu muc voi 6 file .v va file khoi tao ROM "rbrief_pattern.mif".
// ============================================================================
module orb_descriptor_top #(
    parameter ADDR_W = 10,
    parameter P      = 8,
    parameter K0     = 18,
    parameter K1     = 34,
    parameter K2     = 48,
    parameter K3     = 56
)(
    input  wire                clk,
    input  wire                rst_n,

    // ---- Dieu khien cap frame ----
    input  wire                ori_start,
    input  wire [ADDR_W:0]     kp_count,
    output wire                ori_busy,
    output wire                ori_done,
    output wire [ADDR_W:0]     out_count,

    // ---- Giao tiep buffer keypoint dau vao (BRAM) ----
    output wire                kp_rd_en,
    output wire [ADDR_W-1:0]   kp_rd_addr,
    input  wire [26:0]         kp_rd_data,      // {score[7:0], y[8:0], x[9:0]}

    // ---- Giao tiep SRAM anh goc qua Arbiter ----
    output wire                sram_req,
    input  wire                sram_gnt,
    output wire [17:0]         sram_addr,
    output wire                sram_re,
    input  wire [15:0]         sram_rd_data,
    input  wire                sram_valid,

    // ---- Giao tiep buffer ket qua dau ra ----
    output wire                out_we,
    output wire [ADDR_W-1:0]   out_addr,
    output wire [9:0]          out_x,
    output wire [8:0]          out_y,
    output wire [7:0]          out_score,
    output wire [4:0]          out_angle,
    output wire [255:0]        out_desc
);

    // ========================================================================
    // Tin hieu noi bo giua centroid_ctrl va cac khoi xu ly
    // ========================================================================
    // Patch reader
    wire        rd_start;
    wire [9:0]  rd_kx;
    wire [8:0]  rd_ky;
    wire        rd_busy;
    wire        rd_done;
    wire        rd_error;

    // Luong du lieu patch streaming (patch_reader -> moment_acc & gauss_window)
    wire        pb_we;
    wire [5:0]  pb_row;
    wire [4:0]  pb_word;
    wire [10:0] pb_addr;
    wire [15:0] pb_wdata;
    wire        patch_x0_odd;

    // Moment accumulator
    wire               acc_clear;
    wire signed [23:0] m10;
    wire signed [23:0] m01;
    wire               mom_valid;

    // Centroid angle
    wire        ang_start;
    wire        ang_busy;
    wire        ang_done;
    wire [4:0]  ang_bin;

    // Gaussian window -> rBRIEF descriptor
    wire        gauss_busy;
    wire        gauss_done;
    wire        sm_we;
    wire [10:0] sm_waddr;
    wire [7:0]  sm_wdata;

    // rBRIEF descriptor
    wire         rb_start;
    wire [4:0]   rb_angle;
    wire         rb_busy;
    wire         rb_done;
    wire [255:0] rb_desc;

    // ========================================================================
    // 1. Khoi dieu khien trung tam
    // ========================================================================
    centroid_ctrl #(
        .ADDR_W(ADDR_W)
    ) u_ctrl (
        .clk        (clk),
        .rst_n      (rst_n),
        .ori_start  (ori_start),
        .kp_count   (kp_count),
        .ori_busy   (ori_busy),
        .ori_done   (ori_done),
        .out_count  (out_count),
        .kp_rd_en   (kp_rd_en),
        .kp_rd_addr (kp_rd_addr),
        .kp_rd_data (kp_rd_data),
        .sram_req   (sram_req),
        .sram_gnt   (sram_gnt),
        .rd_start   (rd_start),
        .rd_kx      (rd_kx),
        .rd_ky      (rd_ky),
        .rd_busy    (rd_busy),
        .rd_done    (rd_done),
        .rd_error   (rd_error),
        .acc_clear  (acc_clear),
        .mom_valid  (mom_valid),
        .ang_start  (ang_start),
        .ang_done   (ang_done),
        .ang_bin    (ang_bin),
        .gauss_done (gauss_done),
        .rb_start   (rb_start),
        .rb_angle   (rb_angle),
        .rb_done    (rb_done),
        .rb_desc    (rb_desc),
        .out_we     (out_we),
        .out_addr   (out_addr),
        .out_x      (out_x),
        .out_y      (out_y),
        .out_score  (out_score),
        .out_angle  (out_angle),
        .out_desc   (out_desc)
    );

    // ========================================================================
    // 2. Khoi doc cua so 43x43 tu SRAM
    // ========================================================================
    centroid_patch_reader u_patch_reader (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (rd_start),
        .kx           (rd_kx),
        .ky           (rd_ky),
        .busy         (rd_busy),
        .done         (rd_done),
        .error        (rd_error),
        .sram_addr    (sram_addr),
        .sram_re      (sram_re),
        .sram_rd_data (sram_rd_data),
        .sram_valid   (sram_valid),
        .pb_we        (pb_we),
        .pb_row       (pb_row),
        .pb_word      (pb_word),
        .pb_addr      (pb_addr),
        .pb_wdata     (pb_wdata),
        .patch_x0_odd (patch_x0_odd)
    );

    // ========================================================================
    // 3. Khoi tich luy moment m10, m01
    // ========================================================================
    centroid_moment_accumulator u_mom_acc (
        .clk          (clk),
        .rst_n        (rst_n),
        .clear        (acc_clear),
        .patch_done   (rd_done),
        .pb_we        (pb_we),
        .pb_row       (pb_row),
        .pb_word      (pb_word),
        .pb_wdata     (pb_wdata),
        .patch_x0_odd (patch_x0_odd),
        .m10          (m10),
        .m01          (m01),
        .mom_valid    (mom_valid)
    );

    // ========================================================================
    // 4. Khoi tinh bin goc (0..29) bang tim kiem nhi phan
    // ========================================================================
    centroid_angle u_angle (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (ang_start),
        .m10       (m10),
        .m01       (m01),
        .busy      (ang_busy),
        .done      (ang_done),
        .angle_bin (ang_bin)
    );

    // ========================================================================
    // 5. Khoi lam mo Gaussian 7x7 tach 2 chieu (43x43 -> 37x37)
    // ========================================================================
    rbrief_gauss_window #(
        .P  (P),
        .K0 (K0),
        .K1 (K1),
        .K2 (K2),
        .K3 (K3)
    ) u_gauss_win (
        .clk          (clk),
        .rst_n        (rst_n),
        .pb_we        (pb_we),
        .pb_row       (pb_row),
        .pb_word      (pb_word),
        .pb_wdata     (pb_wdata),
        .patch_x0_odd (patch_x0_odd),
        .patch_done   (rd_done),
        .busy         (gauss_busy),
        .done         (gauss_done),
        .sm_we        (sm_we),
        .sm_waddr     (sm_waddr),
        .sm_wdata     (sm_wdata)
    );

    // ========================================================================
    // 6. Khoi trich xuat descriptor rBRIEF 256-bit
    // ========================================================================
    rbrief_descriptor u_rbrief_desc (
        .clk       (clk),
        .rst_n     (rst_n),
        .sm_we     (sm_we),
        .sm_waddr  (sm_waddr),
        .sm_wdata  (sm_wdata),
        .start     (rb_start),
        .angle_bin (rb_angle),
        .busy      (rb_busy),
        .done      (rb_done),
        .desc      (rb_desc)
    );

endmodule