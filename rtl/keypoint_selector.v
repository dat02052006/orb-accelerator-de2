// =============================================================================
// keypoint_selector
//  - Input : keypoint (score, y, x) tu NMS, thu tu raster scan
//  - Output: top-5 score cho moi cell 80x60 (8x8 cell) ghi ra buffer
//            dia chi = cell_id*5 + slot, cell_id = {cy, cx}, 320 word
//
//  Y tuong:
//   Trong raster scan, 1 "dai" (strip) cao 60 hang anh chua 8 cell (theo cx)
//   xen ke nhau, nhung cy chi doi 8 lan / frame.
//   => Moi strip giu 8 bo top-5 (chi so theo cx). Khi cy doi (hoac nms_last)
//      thi flush 8 bo ra buffer (40 lan ghi).
//   => Ping-pong 2 bank: bank dang tich luy (acc_sel) va bank dang flush.
//
//  Gia dinh:
//   - nms_last di kem keypoint hop le cuoi cung cua frame.
//   - Frame ke tiep chi bat dau sau frame_done.
//   - Hai lan doi strip lien tiep cach nhau >= ~41 chu ky
//     (thoi gian flush 1 strip = 40 chu ky + 1).
// =============================================================================
module keypoint_selector #(
    parameter GRID_W  = 8,
    parameter GRID_H  = 8,
    parameter NCELL   = 64,
    parameter SLOT    = 5,               // code duoc unroll cho 5 slot
    parameter SCORE_W = 8,
    parameter YW      = 10,
    parameter XW      = 10,
    parameter CID_W   = 6,               // 64 cell
    parameter ADDR_W  = 9,               // 320 word
    parameter DATA_W  = SCORE_W + YW + XW   // 28
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                nms_valid,
    input  wire [SCORE_W-1:0]  nms_score,
    input  wire [YW-1:0]       nms_y,
    input  wire [XW-1:0]       nms_x,
    input  wire                nms_last,
    output reg                 buf_wr_en,
    output reg  [ADDR_W-1:0]   buf_wr_addr,
    output reg  [DATA_W-1:0]   buf_wr_data,
    output reg                 frame_done
);

    localparam WORD_W   = 5 * DATA_W;    // 5 slot / 1 cell, slot0 o bit thap nhat

    localparam ST_IDLE  = 2'd0;
    localparam ST_FLUSH = 2'd1;
    localparam ST_SCAN  = 2'd2;
    localparam ST_DONE  = 2'd3;

    // =========================================================================
    // 1. Cell ID (combinational)
    // =========================================================================
    wire [2:0] cx = (nms_x >= 10'd560) ? 3'd7 :
                    (nms_x >= 10'd480) ? 3'd6 :
                    (nms_x >= 10'd400) ? 3'd5 :
                    (nms_x >= 10'd320) ? 3'd4 :
                    (nms_x >= 10'd240) ? 3'd3 :
                    (nms_x >= 10'd160) ? 3'd2 :
                    (nms_x >= 10'd80)  ? 3'd1 : 3'd0;

    wire [2:0] cy = (nms_y >= 10'd420) ? 3'd7 :
                    (nms_y >= 10'd360) ? 3'd6 :
                    (nms_y >= 10'd300) ? 3'd5 :
                    (nms_y >= 10'd240) ? 3'd4 :
                    (nms_y >= 10'd180) ? 3'd3 :
                    (nms_y >= 10'd120) ? 3'd2 :
                    (nms_y >= 10'd60)  ? 3'd1 : 3'd0;

    // =========================================================================
    // 2. Khai bao thanh ghi
    // =========================================================================
    // --- luu tru 2 bank x 8 cell: index = {bank, cx} ---
    reg [WORD_W-1:0] kp_mem [0:15];      // 5 slot / word, khong can reset (co cnt bao ve)
    reg [47:0]       cnt_all;            // 16 cnt x 3 bit, cnt[idx] = cnt_all[idx*3 +: 3]
    reg [7:0]        strip_written;      // strip nao da duoc flush that trong frame nay

    // --- theo doi strip ---
    reg              acc_sel;            // bank dang tich luy
    reg [2:0]        cy_prev;
    reg              have_prev;
    reg              last_seen;          // da nhan nms_last, cho flush strip cuoi

    // --- yeu cau flush dang cho (1 entry) ---
    reg              pend_valid;
    reg              pend_bank;
    reg [2:0]        pend_cy;

    // --- FSM flush ---
    reg [1:0]        state, next_state;
    reg              flush_bank, next_flush_bank;
    reg [2:0]        flush_cy,   next_flush_cy;
    reg [2:0]        flush_cx,   next_flush_cx;
    reg [2:0]        flush_slot, next_flush_slot;
    reg              flush_zero, next_flush_zero;   // flush strip rong: ghi toan 0
    reg              flush_last, next_flush_last;   // flush strip cuoi frame
    reg [3:0]        scan_cy,    next_scan_cy;      // quet strip chua ghi (0..8)

    // =========================================================================
    // 3. Top-5 insert (combinational) cho cell {wr_sel, cx}
    // =========================================================================
    wire        evt    = nms_valid && have_prev && (cy != cy_prev);   // doi strip
    wire        wr_sel = evt ? ~acc_sel : acc_sel;    // keypoint nay thuoc strip moi neu evt
    wire [3:0]  wr_idx = {wr_sel, cx};
    wire [5:0]  wr_lsb = {1'b0, wr_idx, 1'b0} + {2'b00, wr_idx};      // idx*3

    wire [WORD_W-1:0] rd_word = kp_mem[wr_idx];
    wire [2:0]        rd_cnt  = cnt_all[wr_lsb +: 3];

    wire [DATA_W-1:0] e0 = rd_word[DATA_W*0 +: DATA_W];
    wire [DATA_W-1:0] e1 = rd_word[DATA_W*1 +: DATA_W];
    wire [DATA_W-1:0] e2 = rd_word[DATA_W*2 +: DATA_W];
    wire [DATA_W-1:0] e3 = rd_word[DATA_W*3 +: DATA_W];
    wire [DATA_W-1:0] e4 = rd_word[DATA_W*4 +: DATA_W];

    wire [SCORE_W-1:0] sc0 = e0[DATA_W-1 -: SCORE_W];
    wire [SCORE_W-1:0] sc1 = e1[DATA_W-1 -: SCORE_W];
    wire [SCORE_W-1:0] sc2 = e2[DATA_W-1 -: SCORE_W];
    wire [SCORE_W-1:0] sc3 = e3[DATA_W-1 -: SCORE_W];
    wire [SCORE_W-1:0] sc4 = e4[DATA_W-1 -: SCORE_W];

    wire [DATA_W-1:0] new_kp = {nms_score, nms_y, nms_x};

    // slot rong (i >= cnt) luon bi chen, nen khong can xoa du lieu cu
    wire gt0 = (rd_cnt <= 3'd0) || (nms_score > sc0);
    wire gt1 = (rd_cnt <= 3'd1) || (nms_score > sc1);
    wire gt2 = (rd_cnt <= 3'd2) || (nms_score > sc2);
    wire gt3 = (rd_cnt <= 3'd3) || (nms_score > sc3);
    wire gt4 = (rd_cnt <= 3'd4) || (nms_score > sc4);

    reg [2:0] pos;
    always @(*) begin
        if      (gt0) pos = 3'd0;
        else if (gt1) pos = 3'd1;
        else if (gt2) pos = 3'd2;
        else if (gt3) pos = 3'd3;
        else if (gt4) pos = 3'd4;
        else          pos = 3'd5;       // day va khong du lon: bo qua
    end

    reg [WORD_W-1:0] new_word;
    always @(*) begin
        case (pos)
            3'd0:    new_word = {e3, e2, e1, e0, new_kp};
            3'd1:    new_word = {e3, e2, e1, new_kp, e0};
            3'd2:    new_word = {e3, e2, new_kp, e1, e0};
            3'd3:    new_word = {e3, new_kp, e2, e1, e0};
            3'd4:    new_word = {new_kp, e3, e2, e1, e0};
            default: new_word = rd_word;
        endcase
    end

    wire       do_ins  = nms_valid && (pos != 3'd5);
    wire [2:0] new_cnt = (rd_cnt < SLOT) ? (rd_cnt + 3'd1) : rd_cnt;

    // =========================================================================
    // 4. Theo doi strip / yeu cau flush
    // =========================================================================
    // take_pend / take_last: FSM nhan yeu cau (dung chung dieu kien voi FSM ben duoi)
    wire strip_end  = (flush_slot == 3'd4) && (flush_cx == 3'd7);
    wire take_pend  = pend_valid &&
                      ((state == ST_IDLE) || ((state == ST_FLUSH) && strip_end));
    wire take_last  = (state == ST_IDLE) && !pend_valid && last_seen;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cy_prev   <= 3'd0;
            have_prev <= 1'b0;
            acc_sel   <= 1'b0;
        end else begin
            if (nms_valid) begin
                cy_prev   <= cy;
                have_prev <= ~nms_last;      // het frame: lan sau la keypoint dau frame moi
            end
            if (evt)
                acc_sel <= ~acc_sel;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pend_valid <= 1'b0;
            pend_bank  <= 1'b0;
            pend_cy    <= 3'd0;
        end else if (evt) begin              // strip cu (acc_sel, cy_prev) can flush
            pend_valid <= 1'b1;
            pend_bank  <= acc_sel;
            pend_cy    <= cy_prev;
        end else if (take_pend) begin
            pend_valid <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            last_seen <= 1'b0;
        else if (take_last)
            last_seen <= 1'b0;
        else if (nms_valid && nms_last)
            last_seen <= 1'b1;
    end

    // =========================================================================
    // 5. Ghi bo nho top-5 / cnt
    // =========================================================================
    wire [3:0] fl_idx = {flush_bank, flush_cx};
    wire [5:0] fl_lsb = {1'b0, fl_idx, 1'b0} + {2'b00, fl_idx};

    // xoa cnt cua cell vua flush xong (o chu ky doc slot cuoi)
    wire clr_cnt   = (state == ST_FLUSH) && !flush_zero && (flush_slot == 3'd4);
    // danh dau strip that da duoc flush (chu ky dau tien cua flush)
    wire mark_strp = (state == ST_FLUSH) && !flush_zero &&
                     (flush_slot == 3'd0) && (flush_cx == 3'd0);

    always @(posedge clk) begin
        if (do_ins)
            kp_mem[wr_idx] <= new_word;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_all       <= 48'd0;
            strip_written <= 8'd0;
        end else begin
            if (do_ins)
                cnt_all[wr_lsb +: 3] <= new_cnt;
            if (clr_cnt)
                cnt_all[fl_lsb +: 3] <= 3'd0;

            if (mark_strp)
                strip_written <= strip_written | (8'd1 << flush_cy);
            else if (state == ST_DONE)
                strip_written <= 8'd0;
        end
    end

    // =========================================================================
    // 6. FSM flush - 3 phan tach biet
    // =========================================================================
    // -------------------------------------------------------------------------
    // 6.1  Logic chuyen trang thai (combinational)
    // -------------------------------------------------------------------------
    always @(*) begin
        // default
        next_state      = state;
        next_flush_bank = flush_bank;
        next_flush_cy   = flush_cy;
        next_flush_cx   = flush_cx;
        next_flush_slot = flush_slot;
        next_flush_zero = flush_zero;
        next_flush_last = flush_last;
        next_scan_cy    = scan_cy;

        case (state)
            ST_IDLE: begin
                next_flush_cx   = 3'd0;
                next_flush_slot = 3'd0;
                if (pend_valid) begin
                    next_flush_bank = pend_bank;
                    next_flush_cy   = pend_cy;
                    next_flush_zero = 1'b0;
                    next_flush_last = 1'b0;
                    next_state      = ST_FLUSH;
                end else if (last_seen) begin
                    next_flush_bank = acc_sel;
                    next_flush_cy   = cy_prev;
                    next_flush_zero = 1'b0;
                    next_flush_last = 1'b1;
                    next_scan_cy    = 4'd0;
                    next_state      = ST_FLUSH;
                end
            end

            ST_FLUSH: begin
                if (flush_slot == 3'd4) begin
                    next_flush_slot = 3'd0;
                    if (flush_cx == 3'd7) begin
                        next_flush_cx = 3'd0;
                        if (pend_valid) begin              // co strip khac dang cho
                            next_flush_bank = pend_bank;
                            next_flush_cy   = pend_cy;
                            next_flush_zero = 1'b0;
                            next_flush_last = 1'b0;
                        end else if (flush_zero || flush_last) begin
                            next_state = ST_SCAN;
                        end else begin
                            next_state = ST_IDLE;
                        end
                    end else begin
                        next_flush_cx = flush_cx + 3'd1;
                    end
                end else begin
                    next_flush_slot = flush_slot + 3'd1;
                end
            end

            ST_SCAN: begin
                if (scan_cy == 4'd8) begin
                    next_state = ST_DONE;
                end else begin
                    next_scan_cy = scan_cy + 4'd1;
                    if (!strip_written[scan_cy[2:0]]) begin   // strip khong co keypoint
                        next_flush_cy   = scan_cy[2:0];
                        next_flush_cx   = 3'd0;
                        next_flush_slot = 3'd0;
                        next_flush_zero = 1'b1;
                        next_state      = ST_FLUSH;
                    end
                end
            end

            ST_DONE: begin
                next_flush_zero = 1'b0;
                next_flush_last = 1'b0;
                next_scan_cy    = 4'd0;
                next_state      = ST_IDLE;
            end

            default: next_state = ST_IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // 6.2  Cap nhat trang thai (sequential)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            flush_bank <= 1'b0;
            flush_cy   <= 3'd0;
            flush_cx   <= 3'd0;
            flush_slot <= 3'd0;
            flush_zero <= 1'b0;
            flush_last <= 1'b0;
            scan_cy    <= 4'd0;
        end else begin
            state      <= next_state;
            flush_bank <= next_flush_bank;
            flush_cy   <= next_flush_cy;
            flush_cx   <= next_flush_cx;
            flush_slot <= next_flush_slot;
            flush_zero <= next_flush_zero;
            flush_last <= next_flush_last;
            scan_cy    <= next_scan_cy;
        end
    end

    // -------------------------------------------------------------------------
    // 6.3  To hop output logic (combinational) + thanh ghi output
    // -------------------------------------------------------------------------
    wire [WORD_W-1:0] fl_word = kp_mem[fl_idx];
    wire [2:0]        fl_cnt  = cnt_all[fl_lsb +: 3];

    // dia chi = {cy,cx}*5 + slot = cell + 4*cell + slot
    wire [8:0] cell_x5  = {3'b000, flush_cy, flush_cx} + {1'b0, flush_cy, flush_cx, 2'b00};
    wire [8:0] wr_addr9 = cell_x5 + {6'b000000, flush_slot};

    reg                 next_buf_wr_en;
    reg  [ADDR_W-1:0]   next_buf_wr_addr;
    reg  [DATA_W-1:0]   next_buf_wr_data;
    reg                 next_frame_done;

    always @(*) begin
        // defaults
        next_buf_wr_en   = 1'b0;
        next_buf_wr_addr = {ADDR_W{1'b0}};
        next_buf_wr_data = {DATA_W{1'b0}};
        next_frame_done  = 1'b0;

        case (state)
            ST_FLUSH: begin
                next_buf_wr_en   = 1'b1;
                next_buf_wr_addr = wr_addr9;

                if (!flush_zero) begin
                    case (flush_slot)
                        3'd0: next_buf_wr_data = (fl_cnt > 3'd0) ? fl_word[DATA_W*0 +: DATA_W] : {DATA_W{1'b0}};
                        3'd1: next_buf_wr_data = (fl_cnt > 3'd1) ? fl_word[DATA_W*1 +: DATA_W] : {DATA_W{1'b0}};
                        3'd2: next_buf_wr_data = (fl_cnt > 3'd2) ? fl_word[DATA_W*2 +: DATA_W] : {DATA_W{1'b0}};
                        3'd3: next_buf_wr_data = (fl_cnt > 3'd3) ? fl_word[DATA_W*3 +: DATA_W] : {DATA_W{1'b0}};
                        3'd4: next_buf_wr_data = (fl_cnt > 3'd4) ? fl_word[DATA_W*4 +: DATA_W] : {DATA_W{1'b0}};
                        default: next_buf_wr_data = {DATA_W{1'b0}};
                    endcase
                end
            end

            ST_DONE: begin
                next_frame_done = 1'b1;
            end

            default: ;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buf_wr_en   <= 1'b0;
            buf_wr_addr <= {ADDR_W{1'b0}};
            buf_wr_data <= {DATA_W{1'b0}};
            frame_done  <= 1'b0;
        end else begin
            buf_wr_en   <= next_buf_wr_en;
            buf_wr_addr <= next_buf_wr_addr;
            buf_wr_data <= next_buf_wr_data;
            frame_done  <= next_frame_done;
        end
    end

endmodule