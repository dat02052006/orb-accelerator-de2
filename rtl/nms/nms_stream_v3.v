//==============================================================================
// nms_stream_v3.v  (ban hoan thien)
// NMS 3x3 streaming - kien truc 5-Pass Fixed-Position 1-Line Buffer (10 bit/pixel)
//   - Khong luu toa do (x, y) vao RAM/FF.
//   - Dich deu theo nhip pixel:  Ngo vao -> R_left -> Line RAM -> W_tr -> W_tc -> W_tl
//   - So sanh truc tiep 4 huong qua khu co dinh trong 1 chu ky.
//   - Tu tinh toa do ngo ra (kp_x, kp_y) tu bo dem vi tri quet.
//
// Quy tac NMS (thu tu raster, giong v2):
//   - Candidate moi thua neu diem <= diem cua bat ky hang xom qua khu nao.
//   - Hang xom qua khu bi loai neu candidate moi co diem lon hon han.
//   => Giu lai pixel neu diem > moi hang xom den truoc va >= moi hang xom den sau.
//
// Can chinh do tre (so voi ban cu): RAM_DEPTH = IMG_W - 4
//   rl -> ram_q mat RAM_DEPTH+1 nhip, rl -> W_tr mat RAM_DEPTH+2 = IMG_W-2 nhip
//   => W_tr = (x+1,y-1), W_tc = (x,y-1), W_tl = (x-1,y-1)  (W_tl tre IMG_W+1 so voi ngo vao)
//
// Hop dong giao tiep:
//   - Sau reset, module tu chay 1 lan xa (IMG_W+2 nhip) de don sach Line RAM.
//     flush_done len 1 xung cuoi lan xa nay; KHONG cap pixel truoc khi thay flush_done.
//   - pix_valid = 1 dung IMG_W*IMG_H lan/frame theo thu tu raster (co the ngat quang).
//   - in_valid chi duoc = 1 khi pix_valid = 1; in_score > 0 khi in_valid = 1.
//   - frame_done: xung sau pixel cuoi; cho flush_done truoc khi bat dau frame moi.
//   - kp_x, kp_y la toa do TUONG DOI trong vung hoat dong (0..IMG_W-1, 0..IMG_H-1);
//     khoi phia sau tu cong offset (vd +21).
//==============================================================================
module nms_stream_v3 #(
    parameter integer    ADDR_WIDTH = 10,
    parameter [9:0]      IMG_W      = 10'd598,   // Chieu rong vung hoat dong (640 - 2*21)
    parameter [8:0]      IMG_H      = 9'd438
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        pix_valid,
    input  wire [7:0]  in_score,  // diem candidate
    input  wire        in_valid,  // = 1 khi pixel hien tai la candidate

    input  wire        frame_done,

    output reg         kp_valid,
    output reg  [9:0]  kp_x,
    output reg  [8:0]  kp_y,
    output reg  [7:0]  kp_score,

    output wire        flush_done
);

    localparam RAM_DEPTH = IMG_W - 4;   // da hieu chinh (cu: IMG_W - 3 bi lech 1 nhip)
    localparam DRAIN_MAX = IMG_W + 2;   // so nhip xa: dua pixel cuoi ra W_tl + 1 nhip ngo ra + du

    //==========================================================================
    // 1. FSM xa (3 khoi): next-state / state register / output logic
    //    Sau reset vao S_DRAIN de don sach Line RAM truoc frame dau tien.
    //==========================================================================
    localparam S_RUN   = 1'b0;
    localparam S_DRAIN = 1'b1;

    reg                state;
    reg                next_state;
    reg [ADDR_WIDTH:0] drain_cnt;
    reg                drain_active;
    reg                drain_last;
    reg                step;            // nhip dich toan bo duong ong
    reg                warm;            // = 1 sau lan xa dau tien (chan kp_valid luc khoi dong)

    // Khoi 1: next-state (to hop)
    always @(*) begin
        next_state = state;
        case (state)
            S_RUN:   if (frame_done) next_state = S_DRAIN;
            S_DRAIN: if (drain_last) next_state = S_RUN;
            default:                 next_state = S_RUN;
        endcase
    end

    // Khoi 2: state register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_DRAIN;
        else        state <= next_state;
    end

    // Khoi 3: output logic (to hop)
    always @(*) begin
        drain_active = (state == S_DRAIN);
        drain_last   = drain_active && (drain_cnt == DRAIN_MAX - 1);
        step         = pix_valid || drain_active;
    end

    assign flush_done = drain_last;

    // Bo dem nhip xa (khoi du lieu rieng, khong nam trong khoi state register)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)           drain_cnt <= 0;
        else if (drain_last)  drain_cnt <= 0;
        else if (drain_active) drain_cnt <= drain_cnt + 1'b1;
        else                  drain_cnt <= 0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)           warm <= 1'b0;
        else if (drain_last)  warm <= 1'b1;
    end

    //==========================================================================
    // 2. Bo dem toa do quet noi bo
    //==========================================================================
    reg [9:0] cur_x;
    reg [8:0] cur_y;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_x <= 10'd0;
            cur_y <= 9'd0;
        end else if (flush_done) begin
            cur_x <= 10'd0;
            cur_y <= 9'd0;
        end else if (step) begin
            if (cur_x == IMG_W - 1) begin
                cur_x <= 10'd0;
                cur_y <= cur_y + 9'd1;
            end else begin
                cur_x <= cur_x + 10'd1;
            end
        end
    end

    // Toa do cua diem dang nam o W_tl (tre IMG_W + 1 buoc so voi cur_x, cur_y)
    wire [9:0] tl_x = (cur_x == 10'd0) ? (IMG_W - 10'd1) : (cur_x - 10'd1);
    wire [8:0] tl_y = (cur_x == 10'd0) ? (cur_y - 9'd2)  : (cur_y - 9'd1);

    //==========================================================================
    // 3. Cac thanh ghi co dinh vi tri (moi o: vld, score[7:0], alv)
    //    R_left (x-1,y) | W_tr (x+1,y-1) | W_tc (x,y-1) | W_tl (x-1,y-1)
    //==========================================================================
    reg       rl_vld, wtr_vld, wtc_vld, wtl_vld;
    reg [7:0] rl_sc,  wtr_sc,  wtc_sc,  wtl_sc;
    reg       rl_alv, wtr_alv, wtc_alv, wtl_alv;

    wire       cand_vld = pix_valid && in_valid && !drain_active;
    wire [7:0] cand_sc  = in_score;

    // Kiem tra bien anh (tranh mep trai/phai va ccham nham qua dong)
    wire has_left  = (cur_x > 10'd0);
    wire has_top   = (cur_y > 9'd0);
    wire has_right = (cur_x < IMG_W - 10'd1);

    wire hit_rl  = cand_vld && rl_vld  && has_left;
    wire hit_wtl = cand_vld && wtl_vld && has_left  && has_top;
    wire hit_wtc = cand_vld && wtc_vld && has_top;
    wire hit_wtr = cand_vld && wtr_vld && has_right && has_top;

    // Candidate moi thua neu <= diem bat ky hang xom qua khu nao
    wire cur_beaten = (hit_rl  && (cand_sc <= rl_sc))  ||
                      (hit_wtl && (cand_sc <= wtl_sc)) ||
                      (hit_wtc && (cand_sc <= wtc_sc)) ||
                      (hit_wtr && (cand_sc <= wtr_sc));

    // Hang xom qua khu bi loai neu candidate moi lon hon han
    wire kill_rl  = hit_rl  && (cand_sc > rl_sc);
    wire kill_wtl = hit_wtl && (cand_sc > wtl_sc);
    wire kill_wtc = hit_wtc && (cand_sc > wtc_sc);
    wire kill_wtr = hit_wtr && (cand_sc > wtr_sc);

    wire rl_alv_u  = rl_alv  && !kill_rl;
    wire wtl_alv_u = wtl_alv && !kill_wtl;
    wire wtc_alv_u = wtc_alv && !kill_wtc;
    wire wtr_alv_u = wtr_alv && !kill_wtr;

    //==========================================================================
    // 4. Line buffer RAM (M4K, 10 bit x RAM_DEPTH)
    //==========================================================================
    (* ramstyle = "M4K, no_rw_check" *) reg [9:0] line_ram [0:RAM_DEPTH-1];
    reg [ADDR_WIDTH-1:0] ptr;
    reg [9:0]            ram_q;

    wire [9:0] rl_pack = {rl_vld, rl_sc, rl_alv_u};

    always @(posedge clk) begin
        if (step) begin
            line_ram[ptr] <= rl_pack;
            ram_q         <= line_ram[ptr];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ptr <= {ADDR_WIDTH{1'b0}};
        else if (flush_done)
            ptr <= {ADDR_WIDTH{1'b0}};
        else if (step)
            ptr <= (ptr == RAM_DEPTH[ADDR_WIDTH-1:0] - {{(ADDR_WIDTH-1){1'b0}}, 1'b1})
                   ? {ADDR_WIDTH{1'b0}}
                   : (ptr + {{(ADDR_WIDTH-1){1'b0}}, 1'b1});
    end

    //==========================================================================
    // 5. Dich chuyen duong ong
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rl_vld  <= 1'b0; rl_sc  <= 8'd0; rl_alv  <= 1'b0;
            wtr_vld <= 1'b0; wtr_sc <= 8'd0; wtr_alv <= 1'b0;
            wtc_vld <= 1'b0; wtc_sc <= 8'd0; wtc_alv <= 1'b0;
            wtl_vld <= 1'b0; wtl_sc <= 8'd0; wtl_alv <= 1'b0;
        end else if (flush_done) begin
            rl_vld  <= 1'b0; wtr_vld <= 1'b0; wtc_vld <= 1'b0; wtl_vld <= 1'b0;
        end else if (step) begin
            // Ngo vao -> R_left
            rl_vld <= cand_vld;
            rl_sc  <= cand_sc;
            rl_alv <= cand_vld && !cur_beaten;

            // Ngo ra Line RAM -> W_tr
            {wtr_vld, wtr_sc, wtr_alv} <= ram_q;

            // W_tr -> W_tc (cap nhat alive truoc khi dich)
            wtc_vld <= wtr_vld;
            wtc_sc  <= wtr_sc;
            wtc_alv <= wtr_alv_u;

            // W_tc -> W_tl (cap nhat alive truoc khi dich)
            wtl_vld <= wtc_vld;
            wtl_sc  <= wtc_sc;
            wtl_alv <= wtc_alv_u;
        end
    end

    //==========================================================================
    // 6. Ngo ra keypoint: phan tu o W_tl da qua du 5 pass va roi cua so
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_valid <= 1'b0;
            kp_x     <= 10'd0;
            kp_y     <= 9'd0;
            kp_score <= 8'd0;
        end else begin
            kp_valid <= warm && step && wtl_vld && wtl_alv_u;
            kp_x     <= tl_x;
            kp_y     <= tl_y;
            kp_score <= wtl_sc;
        end
    end

endmodule
