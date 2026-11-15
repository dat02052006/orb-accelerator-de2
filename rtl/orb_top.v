// ============================================================================
// orb_top.v
// Top-level ORB accelerator (Cyclone II / DE2): noi toan bo pipeline.
//
//   Nios II --> image_loader --> loader_sram_pack --\
//                                                    +--> sram_controller --> SRAM
//   image_reader (FAST) -----------------------------+ (mux theo pha)
//   centroid_patch_reader ----------------------------/
//
//   image_reader -> orb_line_buffer -> orb_line_buffer_ctrl -> orb_window_generator
//     -> orb_fast_detector -> orb_fast_score -> candidate_buffer <-> nms_top
//     -> keypoint_buffer -> centroid_ctrl -> { centroid_patch_reader,
//        centroid_moment_accumulator, centroid_angle, rbrief_gauss_window,
//        rbrief_descriptor } -> ket qua (out_*)
//
// Xu ly TUNG FRAME, tuan tu (khong chong lan FAST/NMS voi centroid), nen SRAM
// chi co 1 chu so huu o moi pha:
//   T_IDLE                     : loader (Nios II nap anh)
//   T_START..T_NMS             : image_reader (FAST)
//   T_ORI_START, T_ORI_WAIT    : centroid_patch_reader (sram_gnt = 1)
//
// Dieu khien frame (FSM 3 khoi):
//   T_START     : phat frame_clear (candidate_buffer), kp_clear (keypoint_buffer),
//                 lb_start (line buffer ctrl) cung 1 chu ky. row_start dau tien
//                 den sau khi nap 7 hang tu SRAM (hang tram chu ky) nen
//                 frame_clear luon som hon row_start it nhat 1 chu ky.
//   T_SCAN      : cho orb_line_buffer_ctrl bao frame_done (loi -> T_ERR)
//   T_DRAIN     : cho DRAIN_CYC chu ky de pipeline window/FAST/score xa het
//                 (hang cuoi vao candidate_buffer)
//   T_NMS       : cho nms_busy = 0 va bank_ready = 0 lien tuc IDLE_CYC chu ky
//   T_ORI_START : phat ori_start (centroid_ctrl)
//   T_ORI_WAIT  : cho ori_done
//   T_DONE      : phat orb_done 1 chu ky
//
// Luu y: sau T_ERR cac bank candidate / pipeline co the con du lieu do dang
// dang, nen can reset (rst_n) truoc khi chay frame ke tiep.
// image_dma chua duoc noi (xem ghi chu cuoi file).
// ============================================================================
module orb_top #(
    parameter DRAIN_CYC = 64,           // chu ky cho pipeline xa sau frame_done (< 128)
    parameter IDLE_CYC  = 8             // so chu ky NMS phai rong lien tuc (< 128)
)(
    input  wire         clk,
    input  wire         rst_n,

    // ---- Nios II: nap anh ----
    input  wire         load_start,     // xung: bat dau nap anh moi
    input  wire [9:0]   img_width,      // phai = 640
    input  wire [9:0]   img_height,     // phai = 480
    input  wire         pixel_valid,
    input  wire [7:0]   pixel_data,
    output wire         pixel_ready,
    output wire         image_loaded,   // muc 1 khi da nap du 307200 byte
    output wire         size_error,

    // ---- Nios II: dieu khien xu ly ----
    input  wire         orb_start,      // xung: xu ly 1 frame (bo qua neu image_loaded = 0)
    input  wire [7:0]   fast_threshold, // nguong FAST T
    output wire         orb_busy,
    output wire         orb_done,       // xung 1 chu ky: xong frame
    output wire         orb_error,      // xung 1 chu ky: loi line buffer / image_reader
    output wire         kp_overflow,    // keypoint_buffer bi day (co keypoint bi bo)

    // ---- ket qua (tu centroid_ctrl, chua co buffer dau ra) ----
    output wire         out_we,
    output wire [9:0]   out_addr,
    output wire [9:0]   out_x,
    output wire [8:0]   out_y,
    output wire [7:0]   out_score,
    output wire [4:0]   out_angle,
    output wire [255:0] out_desc,
    output wire [10:0]  out_count,      // so keypoint da ghi ra

    // ---- chan SRAM (DE2) ----
    output wire [17:0]  sram_addr,
    inout  wire [15:0]  sram_dq,
    output wire         sram_ce_n,
    output wire         sram_oe_n,
    output wire         sram_we_n,
    output wire         sram_ub_n,
    output wire         sram_lb_n
);

    // ========================================================================
    // FSM dieu khien frame
    // ========================================================================
    localparam [3:0] T_IDLE      = 4'd0,
                     T_START     = 4'd1,
                     T_SCAN      = 4'd2,
                     T_DRAIN     = 4'd3,
                     T_NMS       = 4'd4,
                     T_ORI_START = 4'd5,
                     T_ORI_WAIT  = 4'd6,
                     T_DONE      = 4'd7,
                     T_ERR       = 4'd8;

    reg [3:0] state, next_state;
    reg [6:0] cnt;                      // dem cho T_DRAIN / T_NMS

    // tin hieu tu cac khoi (khai bao truoc khi dung)
    wire lb_frame_done;
    wire lb_error;
    wire nms_busy;
    wire bank_ready;
    wire ori_done;

    wire nms_idle  = ~nms_busy & ~bank_ready;
    wire drain_end = (cnt == (DRAIN_CYC - 1));
    wire idle_ok   = nms_idle && (cnt == (IDLE_CYC - 1));

    // ------------------------------------------------------------------
    // FSM phan 1: next-state transition logic
    // ------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            T_IDLE: begin
                if (orb_start && image_loaded) next_state = T_START;
            end
            T_START: next_state = T_SCAN;
            T_SCAN: begin
                if (lb_error)           next_state = T_ERR;
                else if (lb_frame_done) next_state = T_DRAIN;
            end
            T_DRAIN: begin
                if (drain_end) next_state = T_NMS;
            end
            T_NMS: begin
                if (idle_ok) next_state = T_ORI_START;
            end
            T_ORI_START: next_state = T_ORI_WAIT;
            T_ORI_WAIT: begin
                if (ori_done) next_state = T_DONE;
            end
            T_DONE:  next_state = T_IDLE;
            T_ERR:   next_state = T_IDLE;
            default: next_state = T_IDLE;
        endcase
    end

    // ------------------------------------------------------------------
    // FSM phan 2: state register update
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= T_IDLE;
        else        state <= next_state;
    end

    // ------------------------------------------------------------------
    // Bo dem cho: T_DRAIN dem het DRAIN_CYC; T_NMS dem so chu ky rong lien tuc
    // (ve 0 khi roi state, nen vao state moi luon bat dau tu 0)
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 7'd0;
        end else begin
            case (state)
                T_DRAIN: begin
                    if (drain_end) cnt <= 7'd0;
                    else           cnt <= cnt + 7'd1;
                end
                T_NMS: begin
                    if (!nms_idle)                 cnt <= 7'd0;
                    else if (cnt == (IDLE_CYC - 1)) cnt <= 7'd0;
                    else                            cnt <= cnt + 7'd1;
                end
                default: cnt <= 7'd0;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // FSM phan 3: combinational output logic
    // ------------------------------------------------------------------
    wire frame_clear = (state == T_START);      // -> candidate_buffer
    wire kp_clear    = (state == T_START);      // -> keypoint_buffer
    wire lb_start    = (state == T_START);      // -> orb_line_buffer_ctrl
    wire ori_start   = (state == T_ORI_START);  // -> centroid_ctrl

    wire sel_fast = (state == T_START) || (state == T_SCAN) ||
                    (state == T_DRAIN) || (state == T_NMS);
    wire sel_ori  = (state == T_ORI_START) || (state == T_ORI_WAIT);

    wire sram_gnt = sel_ori;                    // -> centroid_ctrl (FAST da xong)

    assign orb_busy  = (state != T_IDLE);
    assign orb_done  = (state == T_DONE);
    assign orb_error = (state == T_ERR);

    // ========================================================================
    // SRAM: controller + mux 3 chu so huu
    // ========================================================================
    // loader (qua bo gop byte -> word)
    wire [18:0] ld_addr;
    wire [7:0]  ld_wdata;
    wire        ld_we;
    wire        ld_ready;
    wire [17:0] pk_addr;
    wire [15:0] pk_wdata;
    wire        pk_we;

    // image_reader (FAST)
    wire [17:0] ir_sram_addr;
    wire        ir_sram_re;
    wire        ir_sram_valid;

    // centroid_patch_reader
    wire [17:0] pr_sram_addr;
    wire        pr_sram_re;
    wire        pr_sram_valid;

    // sram_controller
    wire [17:0] m_addr;
    wire [15:0] m_wdata;
    wire        m_we;
    wire        m_re;
    wire [15:0] m_rd_data;
    wire        m_ready;
    wire        m_valid;

    assign m_addr  = sel_ori  ? pr_sram_addr :
                     sel_fast ? ir_sram_addr : pk_addr;
    assign m_re    = sel_ori  ? pr_sram_re   :
                     sel_fast ? ir_sram_re   : 1'b0;
    assign m_we    = (sel_ori || sel_fast) ? 1'b0 : pk_we;
    assign m_wdata = pk_wdata;

    assign ir_sram_valid = m_valid & sel_fast;
    assign pr_sram_valid = m_valid & sel_ori;

    sram_controller u_sram (
        .clk       (clk),
        .rst_n     (rst_n),
        .addr      (m_addr),
        .wr_data   (m_wdata),
        .we        (m_we),
        .re        (m_re),
        .rd_data   (m_rd_data),
        .ready     (m_ready),
        .valid     (m_valid),
        .done      (),
        .sram_addr (sram_addr),
        .sram_dq   (sram_dq),
        .sram_ce_n (sram_ce_n),
        .sram_oe_n (sram_oe_n),
        .sram_we_n (sram_we_n),
        .sram_ub_n (sram_ub_n),
        .sram_lb_n (sram_lb_n)
    );

    // ========================================================================
    // Nap anh
    // ========================================================================
    image_loader u_loader (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (load_start),
        .img_width    (img_width),
        .img_height   (img_height),
        .pixel_valid  (pixel_valid),
        .pixel_data   (pixel_data),
        .pixel_ready  (pixel_ready),
        .sram_addr    (ld_addr),
        .sram_wr_data (ld_wdata),
        .sram_we      (ld_we),
        .sram_ready   (ld_ready),
        .image_loaded (image_loaded),
        .size_error   (size_error)
    );

    loader_sram_pack u_pack (
        .clk      (clk),
        .rst_n    (rst_n),
        .ld_addr  (ld_addr),
        .ld_wdata (ld_wdata),
        .ld_we    (ld_we),
        .ld_ready (ld_ready),
        .sr_addr  (pk_addr),
        .sr_wdata (pk_wdata),
        .sr_we    (pk_we),
        .sr_ready (m_ready)
    );

    // ========================================================================
    // FAST: image_reader -> line buffer -> window 7x7
    // ========================================================================
    wire        ir_start;
    wire [8:0]  ir_first_row;
    wire [4:0]  ir_first_bank;
    wire [4:0]  ir_num_rows;
    wire        ir_done;
    wire        ir_error;

    wire        lb_we;
    wire [4:0]  lb_row;
    wire [8:0]  lb_col_word;
    wire [15:0] lb_wdata;

    wire        lb_rd_en;
    wire [9:0]  lb_rd_col;
    wire        lb_rd_valid;
    wire [7:0]  lb_p0, lb_p1, lb_p2, lb_p3, lb_p4, lb_p5, lb_p6;

    wire        win_clear;
    wire        win_in_valid;
    wire [7:0]  win_p0, win_p1, win_p2, win_p3, win_p4, win_p5, win_p6;

    wire [8:0]  lb_center_y;
    wire        lb_busy;
    wire        lb_row_done;            // hang tam cua line buffer (khong dung: score tu phat row_*)

    orb_line_buffer_ctrl u_lbctrl (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (lb_start),
        .busy         (lb_busy),
        .frame_done   (lb_frame_done),
        .error        (lb_error),
        .row_done     (lb_row_done),
        .center_y     (lb_center_y),
        .ir_start     (ir_start),
        .ir_first_row (ir_first_row),
        .ir_first_bank(ir_first_bank),
        .ir_num_rows  (ir_num_rows),
        .ir_done      (ir_done),
        .ir_error     (ir_error),
        .lb_rd_en     (lb_rd_en),
        .lb_rd_col    (lb_rd_col),
        .lb_rd_valid  (lb_rd_valid),
        .lb_p0 (lb_p0), .lb_p1 (lb_p1), .lb_p2 (lb_p2), .lb_p3 (lb_p3),
        .lb_p4 (lb_p4), .lb_p5 (lb_p5), .lb_p6 (lb_p6),
        .win_clear    (win_clear),
        .win_in_valid (win_in_valid),
        .win_p0 (win_p0), .win_p1 (win_p1), .win_p2 (win_p2), .win_p3 (win_p3),
        .win_p4 (win_p4), .win_p5 (win_p5), .win_p6 (win_p6)
    );

    image_reader u_imgrd (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (ir_start),
        .first_row    (ir_first_row),
        .num_rows     (ir_num_rows),
        .first_bank   (ir_first_bank),
        .busy         (),
        .done         (ir_done),
        .error        (ir_error),
        .sram_addr    (ir_sram_addr),
        .sram_re      (ir_sram_re),
        .sram_rd_data (m_rd_data),
        .sram_valid   (ir_sram_valid),
        .lb_we        (lb_we),
        .lb_row       (lb_row),
        .lb_col_word  (lb_col_word),
        .lb_wdata     (lb_wdata)
    );

    orb_line_buffer u_linebuf (
        .clk         (clk),
        .lb_we       (lb_we),
        .lb_row      (lb_row),
        .lb_col_word (lb_col_word),
        .lb_wdata    (lb_wdata),
        .rd_en       (lb_rd_en),
        .rd_col      (lb_rd_col),
        .rd_valid    (lb_rd_valid),
        .rd_p0 (lb_p0), .rd_p1 (lb_p1), .rd_p2 (lb_p2), .rd_p3 (lb_p3),
        .rd_p4 (lb_p4), .rd_p5 (lb_p5), .rd_p6 (lb_p6)
    );

    // cua so 7x7: wRC, R = hang (0 = tren cung), C = cot (6 = moi nhat); tam = w33
    wire        window_valid;
    wire [9:0]  win_center_x;
    wire [7:0]  w02, w03, w04;
    wire [7:0]  w11, w15;
    wire [7:0]  w20, w26;
    wire [7:0]  w30, w33, w36;
    wire [7:0]  w40, w46;
    wire [7:0]  w51, w55;
    wire [7:0]  w62, w63, w64;

    orb_window_generator u_window (
        .clk          (clk),
        .rst_n        (rst_n),
        .clear        (win_clear),
        .in_valid     (win_in_valid),
        .in_p0 (win_p0), .in_p1 (win_p1), .in_p2 (win_p2), .in_p3 (win_p3),
        .in_p4 (win_p4), .in_p5 (win_p5), .in_p6 (win_p6),
        .window_valid (window_valid),
        .win_center_x (win_center_x),
        .window_flat  (),
        .w02 (w02), .w03 (w03), .w04 (w04),
        .w11 (w11), .w15 (w15),
        .w20 (w20), .w26 (w26),
        .w30 (w30), .w33 (w33), .w36 (w36),
        .w40 (w40), .w46 (w46),
        .w51 (w51), .w55 (w55),
        .w62 (w62), .w63 (w63), .w64 (w64)
    );

    // ========================================================================
    // FAST-9 detector + score
    //   p0=(0,-3) p1=(1,-3) p2=(2,-2) p3=(3,-1) p4=(3,0) p5=(3,1) p6=(2,2) p7=(1,3)
    //   p8=(0,3) p9=(-1,3) p10=(-2,2) p11=(-3,1) p12=(-3,0) p13=(-3,-1)
    //   p14=(-2,-2) p15=(-1,-3);  pixel (dx,dy) = w[3+dy][3+dx]
    // ========================================================================
    wire         det_valid;
    wire         det_is_corner;
    wire [9:0]   det_x;
    wire [9:0]   det_y;
    wire [135:0] det_pack;

    orb_fast_detector u_fast (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (window_valid),
        .in_x      (win_center_x),
        .in_y      ({1'b0, lb_center_y}),
        .threshold (fast_threshold),
        .ip        (w33),
        .p0  (w03), .p1  (w04), .p2  (w15), .p3  (w26),
        .p4  (w36), .p5  (w46), .p6  (w55), .p7  (w64),
        .p8  (w63), .p9  (w62), .p10 (w51), .p11 (w40),
        .p12 (w30), .p13 (w20), .p14 (w11), .p15 (w02),
        .out_valid (det_valid),
        .is_corner (det_is_corner),
        .out_x     (det_x),
        .out_y     (det_y),
        .pack      (det_pack)
    );

    wire        score_valid;
    wire [9:0]  score_x;
    wire [7:0]  score_value;
    wire        score_row_start;
    wire        score_row_done;

    orb_fast_score u_score (
        .clk          (clk),
        .rst_n        (rst_n),
        .in_valid     (det_valid),
        .in_is_corner (det_is_corner),
        .in_x         (det_x),
        .in_pack      (det_pack),
        .score_valid  (score_valid),
        .score_x      (score_x),
        .score_value  (score_value),
        .row_start    (score_row_start),
        .row_done     (score_row_done)
    );

    // ========================================================================
    // Candidate buffer + NMS + keypoint buffer
    // ========================================================================
    wire        nms_done;
    wire [1:0]  nms_done_bank;
    wire [9:0]  bank_count_0, bank_count_1, bank_count_2;
    wire [9:0]  bank_row_0,   bank_row_1,   bank_row_2;
    wire [2:0]  bank_state_0, bank_state_1, bank_state_2;
    wire        cb_rd_en_0, cb_rd_en_1, cb_rd_en_2;
    wire [9:0]  cb_rd_addr_0, cb_rd_addr_1, cb_rd_addr_2;
    wire [17:0] cb_rd_data_0, cb_rd_data_1, cb_rd_data_2;

    candidate_buffer u_candbuf (
        .clk          (clk),
        .rst_n        (rst_n),
        .score_valid  (score_valid),
        .score_x      (score_x),
        .score_value  (score_value),
        .row_start    (score_row_start),
        .row_done     (score_row_done),
        .frame_clear  (frame_clear),
        .nms_done     (nms_done),
        .nms_done_bank(nms_done_bank),
        .rd_en_0      (cb_rd_en_0),
        .rd_addr_0    (cb_rd_addr_0),
        .rd_data_0    (cb_rd_data_0),
        .rd_en_1      (cb_rd_en_1),
        .rd_addr_1    (cb_rd_addr_1),
        .rd_data_1    (cb_rd_data_1),
        .rd_en_2      (cb_rd_en_2),
        .rd_addr_2    (cb_rd_addr_2),
        .rd_data_2    (cb_rd_data_2),
        .bank_ready   (bank_ready),
        .bank_count_0 (bank_count_0),
        .bank_count_1 (bank_count_1),
        .bank_count_2 (bank_count_2),
        .bank_row_0   (bank_row_0),
        .bank_row_1   (bank_row_1),
        .bank_row_2   (bank_row_2),
        .bank_state_0 (bank_state_0),
        .bank_state_1 (bank_state_1),
        .bank_state_2 (bank_state_2),
        .dbg_wr_bank  ()
    );

    wire        kp_valid;
    wire [27:0] kp_data;
    wire        kp_ready;

    nms_top u_nms (
        .clk          (clk),
        .rst_n        (rst_n),
        .bank_ready   (bank_ready),
        .bank_count_0 (bank_count_0),
        .bank_count_1 (bank_count_1),
        .bank_count_2 (bank_count_2),
        .bank_row_0   (bank_row_0),
        .bank_row_1   (bank_row_1),
        .bank_row_2   (bank_row_2),
        .bank_state_0 (bank_state_0),
        .bank_state_1 (bank_state_1),
        .bank_state_2 (bank_state_2),
        .rd_en_0      (cb_rd_en_0),
        .rd_addr_0    (cb_rd_addr_0),
        .rd_data_0    (cb_rd_data_0),
        .rd_en_1      (cb_rd_en_1),
        .rd_addr_1    (cb_rd_addr_1),
        .rd_data_1    (cb_rd_data_1),
        .rd_en_2      (cb_rd_en_2),
        .rd_addr_2    (cb_rd_addr_2),
        .rd_data_2    (cb_rd_data_2),
        .nms_done     (nms_done),
        .nms_done_bank(nms_done_bank),
        .kp_valid     (kp_valid),
        .kp_data      (kp_data),
        .kp_ready     (kp_ready),
        .nms_busy     (nms_busy)
    );

    wire [10:0] kp_count;
    wire        kp_rd_en;
    wire [9:0]  kp_rd_addr;
    wire [26:0] kp_rd_data;

    keypoint_buffer #(
        .ADDR_W   (10),
        .Y_OFFSET (3)
    ) u_kpbuf (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (kp_clear),
        .kp_valid (kp_valid),
        .kp_data  (kp_data),
        .kp_ready (kp_ready),
        .kp_count (kp_count),
        .overflow (kp_overflow),
        .rd_en    (kp_rd_en),
        .rd_addr  (kp_rd_addr),
        .rd_data  (kp_rd_data)
    );

    // ========================================================================
    // Huong (intensity centroid) + rBRIEF
    // ========================================================================
    wire        ori_busy;

    wire        pr_start;
    wire [9:0]  pr_kx;
    wire [8:0]  pr_ky;
    wire        pr_busy;
    wire        pr_done;
    wire        pr_error;

    wire        acc_clear;
    wire        mom_valid;

    wire        ang_start;
    wire        ang_done;
    wire [4:0]  ang_bin;

    wire        gauss_done;

    wire        rb_start;
    wire [4:0]  rb_angle;
    wire        rb_done;
    wire [255:0] rb_desc;

    centroid_ctrl #(
        .ADDR_W (10)
    ) u_cctrl (
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
        .sram_req   (),
        .sram_gnt   (sram_gnt),
        .rd_start   (pr_start),
        .rd_kx      (pr_kx),
        .rd_ky      (pr_ky),
        .rd_busy    (pr_busy),
        .rd_done    (pr_done),
        .rd_error   (pr_error),
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

    wire        pb_we;
    wire [5:0]  pb_row;
    wire [4:0]  pb_word;
    wire [10:0] pb_addr;
    wire [15:0] pb_wdata;
    wire        patch_x0_odd;

    centroid_patch_reader u_patch (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (pr_start),
        .kx           (pr_kx),
        .ky           (pr_ky),
        .busy         (pr_busy),
        .done         (pr_done),
        .error        (pr_error),
        .sram_addr    (pr_sram_addr),
        .sram_re      (pr_sram_re),
        .sram_rd_data (m_rd_data),
        .sram_valid   (pr_sram_valid),
        .pb_we        (pb_we),
        .pb_row       (pb_row),
        .pb_word      (pb_word),
        .pb_addr      (pb_addr),
        .pb_wdata     (pb_wdata),
        .patch_x0_odd (patch_x0_odd)
    );

    wire signed [23:0] m10;
    wire signed [23:0] m01;

    centroid_moment_accumulator u_moment (
        .clk          (clk),
        .rst_n        (rst_n),
        .clear        (acc_clear),
        .patch_done   (pr_done),
        .pb_we        (pb_we),
        .pb_row       (pb_row),
        .pb_word      (pb_word),
        .pb_wdata     (pb_wdata),
        .patch_x0_odd (patch_x0_odd),
        .m10          (m10),
        .m01          (m01),
        .mom_valid    (mom_valid)
    );

    centroid_angle u_angle (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (ang_start),
        .m10       (m10),
        .m01       (m01),
        .busy      (),
        .done      (ang_done),
        .angle_bin (ang_bin)
    );

    wire        sm_we;
    wire [10:0] sm_waddr;
    wire [7:0]  sm_wdata;

    rbrief_gauss_window u_gauss (
        .clk          (clk),
        .rst_n        (rst_n),
        .pb_we        (pb_we),
        .pb_row       (pb_row),
        .pb_word      (pb_word),
        .pb_wdata     (pb_wdata),
        .patch_x0_odd (patch_x0_odd),
        .patch_done   (pr_done),
        .busy         (),
        .done         (gauss_done),
        .sm_we        (sm_we),
        .sm_waddr     (sm_waddr),
        .sm_wdata     (sm_wdata)
    );

    rbrief_descriptor u_rbrief (
        .clk       (clk),
        .rst_n     (rst_n),
        .sm_we     (sm_we),
        .sm_waddr  (sm_waddr),
        .sm_wdata  (sm_wdata),
        .start     (rb_start),
        .angle_bin (rb_angle),
        .busy      (),
        .done      (rb_done),
        .desc      (rb_desc)
    );

    // ------------------------------------------------------------------
    // Ghi chu: image_dma khong duoc noi vao top. Module nay doc/ghi tung byte
    // (dia chi byte 19 bit, du lieu 8 bit) trong khi sram_controller la
    // 16 bit va UB/LB luon = 0, nen ghi 1 byte se de len ca word. Muon dung
    // phai co them bo doc-sua-ghi (read-modify-write) theo word.
    // ------------------------------------------------------------------

endmodule
