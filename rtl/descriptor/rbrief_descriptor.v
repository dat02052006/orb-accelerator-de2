// ============================================================================
// rbrief_descriptor
//   Tao descriptor rBRIEF 256 bit tu smoothed buffer 37x37 va angle_bin (0..29).
//
//   Khac ban cu: KHONG con ROM 30 bin x 256 cap da xoay san (~169 kbit).
//   Chi luu 256 cap toa do goc chua xoay (rbrief_base.mif, 24 bit/cap = 6 kbit)
//   va xoay truc tiep bang cos/sin cua bin (rbrief_trig_rom.v, nam trong logic):
//       x' = x*cos - y*sin ,  y' = x*sin + y*cos     (cos/sin dang Q, FRAC bit le)
//       dia chi smoothed buffer = (18 + y') * 37 + (18 + x')
//   Lam tron: floor + inc; khi phan le dung 0.5 thi theo luat rieng cua bin (tie_q).
//   Hai file rbrief_base.mif/.hex va rbrief_trig_rom.v do gen_rbrief_rot.py sinh ra
//   tu bang cu; tham so FRAC o day PHAI bang FRAC script bao.
//
//   Pipeline 1 cap/chu ky (8 phep nhan co dau 6 x (FRAC+2) bit):
//     RUN: pair_cnt -> ROM goc | v1: nhan | v2: cong+lam tron | v3: tinh dia chi |
//     v4: doc smoothed buffer | v5: so sanh pix_p < pix_q, day vao desc
//   desc[k] = bit cap k (cap dau tien ket thuc o desc[0]). Khoang 263 chu ky tu start den done.
//   Smoothed buffer khong duoc ghi trong luc khoi nay chay (ctrl tuan tu hoa).
//   Yeu cau: rbrief_gauss_window.v (module rbrief_sdp_ram) cung nam trong project.
// ============================================================================
module rbrief_descriptor #(
    parameter FRAC = 16                 // so bit phan le cua cos/sin (khop rbrief_trig_rom.v)
)(
    input  wire         clk,
    input  wire         rst_n,

    // ---- cong ghi smoothed buffer (tu rbrief_gauss_window) ----
    input  wire         sm_we,
    input  wire [10:0]  sm_waddr,
    input  wire [7:0]   sm_wdata,

    // ---- dieu khien ----
    input  wire         start,          // xung; smoothed buffer da day du, angle_bin hop le
    input  wire [4:0]   angle_bin,      // 0..29
    output wire         busy,
    output wire         done,           // xung 1 chu ky; desc hop le tu day den start ke tiep
    output wire [255:0] desc
);

    localparam CW = FRAC + 2;           // do rong he so cos/sin (co dau)
    localparam PW = CW + 6;             // do rong tich 6 bit x CW bit (co dau)
    localparam SW = PW + 1;             // do rong tong/hieu 2 tich

    // ---------------- state encoding ----------------
    localparam [1:0] R_IDLE  = 2'd0,
                     R_RUN   = 2'd1,
                     R_DRAIN = 2'd2,
                     R_DONE  = 2'd3;

    reg [1:0] state, next_state;

    reg [4:0]   bin_r;
    reg [7:0]   pair_cnt;
    reg [255:0] desc_r;

    wire last_pair = (pair_cnt == 8'd255);

    // ======================================================
    // ROM 256 cap toa do goc: {xp[5:0], yp[5:0], xq[5:0], yq[5:0]}, co dau
    // ======================================================
    wire [23:0] base_q;

    altsyncram #(
        .operation_mode         ("ROM"),
        .width_a                (24),
        .widthad_a              (8),
        .numwords_a             (256),
        .outdata_reg_a          ("UNREGISTERED"),
        .init_file              ("rbrief_base.mif"),
        .intended_device_family ("Cyclone II")
    ) u_base_rom (
        .clock0    (clk),
        .address_a (pair_cnt),
        .q_a       (base_q)
    );

    wire signed [5:0] xp = base_q[23:18];
    wire signed [5:0] yp = base_q[17:12];
    wire signed [5:0] xq = base_q[11:6];
    wire signed [5:0] yq = base_q[5:0];

    // ======================================================
    // cos/sin cua bin (hang so theo bin_r) - dang thanh ghi de cat duong tre
    // ======================================================
    wire signed [CW-1:0] cos_w;
    wire signed [CW-1:0] sin_w;
    wire        [2:0]    tie_w;

    rbrief_trig_rom u_trig (
        .bin   (bin_r),
        .cos_q (cos_w),
        .sin_q (sin_w),
        .tie_q (tie_w)
    );

    reg signed [CW-1:0] cos_r;
    reg signed [CW-1:0] sin_r;
    reg        [2:0]    tie_r;

    always @(posedge clk) begin
        cos_r <= cos_w;
        sin_r <= sin_w;
        tie_r <= tie_w;
    end

    // ======================================================
    // Stage 1: 8 phep nhan (registered)
    // ======================================================
    reg signed [PW-1:0] p_xc, p_ys, p_xs, p_yc;
    reg signed [PW-1:0] q_xc, q_ys, q_xs, q_yc;
    reg        [2:0]    tie_s1;

    always @(posedge clk) begin
        p_xc   <= xp * cos_r;
        p_ys   <= yp * sin_r;
        p_xs   <= xp * sin_r;
        p_yc   <= yp * cos_r;
        q_xc   <= xq * cos_r;
        q_ys   <= yq * sin_r;
        q_xs   <= xq * sin_r;
        q_yc   <= yq * cos_r;
        tie_s1 <= tie_r;
    end

    // ======================================================
    // Stage 2: cong/tru + lam tron ve so nguyen 6 bit (registered)
    // ======================================================
    wire signed [SW-1:0] sum_xp = p_xc - p_ys;      // x*cos - y*sin
    wire signed [SW-1:0] sum_yp = p_xs + p_yc;      // x*sin + y*cos
    wire signed [SW-1:0] sum_xq = q_xc - q_ys;
    wire signed [SW-1:0] sum_yq = q_xs + q_yc;

    wire signed [5:0] rnd_xp;
    wire signed [5:0] rnd_yp;
    wire signed [5:0] rnd_xq;
    wire signed [5:0] rnd_yq;

    rbrief_rnd #(.FRAC(FRAC), .SW(SW)) u_rnd_xp (.sum(sum_xp), .tie(tie_s1), .r(rnd_xp));
    rbrief_rnd #(.FRAC(FRAC), .SW(SW)) u_rnd_yp (.sum(sum_yp), .tie(tie_s1), .r(rnd_yp));
    rbrief_rnd #(.FRAC(FRAC), .SW(SW)) u_rnd_xq (.sum(sum_xq), .tie(tie_s1), .r(rnd_xq));
    rbrief_rnd #(.FRAC(FRAC), .SW(SW)) u_rnd_yq (.sum(sum_yq), .tie(tie_s1), .r(rnd_yq));

    reg signed [5:0] xr_p, yr_p, xr_q, yr_q;

    always @(posedge clk) begin
        xr_p <= rnd_xp;
        yr_p <= rnd_yp;
        xr_q <= rnd_xq;
        yr_q <= rnd_yq;
    end

    // ======================================================
    // Stage 3: dia chi = (18 + y') * 37 + (18 + x'),  37 = 32 + 4 + 1 (khong dung phep nhan)
    // ======================================================
    wire [5:0] up = yr_p + 6'd18;
    wire [5:0] vp = xr_p + 6'd18;
    wire [5:0] uq = yr_q + 6'd18;
    wire [5:0] vq = xr_q + 6'd18;

    wire [10:0] addr_p_c = {up, 5'b00000} + {3'b000, up, 2'b00} + {5'b00000, up} + {5'b00000, vp};
    wire [10:0] addr_q_c = {uq, 5'b00000} + {3'b000, uq, 2'b00} + {5'b00000, uq} + {5'b00000, vq};

    reg [10:0] addr_p;
    reg [10:0] addr_q;

    always @(posedge clk) begin
        addr_p <= addr_p_c;
        addr_q <= addr_q_c;
    end

    // ======================================================
    // Smoothed buffer: 2 ban sao, moi ban 1 cong doc (doc dong bo, tre 1 chu ky)
    // ======================================================
    wire [7:0] pix_p;
    wire [7:0] pix_q;

    rbrief_sdp_ram #(
        .DW   (8),
        .AW   (11),
        .DEPTH(1369)
    ) u_sm_p (
        .clk  (clk),
        .we   (sm_we),
        .waddr(sm_waddr),
        .wdata(sm_wdata),
        .raddr(addr_p),
        .rdata(pix_p)
    );

    rbrief_sdp_ram #(
        .DW   (8),
        .AW   (11),
        .DEPTH(1369)
    ) u_sm_q (
        .clk  (clk),
        .we   (sm_we),
        .waddr(sm_waddr),
        .wdata(sm_wdata),
        .raddr(addr_q),
        .rdata(pix_q)
    );

    // ======================================================
    // Chuoi co hop le v1..v5 va thanh ghi dich desc
    // ======================================================
    reg v1, v2, v3, v4, v5;

    wire cmp_bit = (pix_p < pix_q);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v1     <= 1'b0;
            v2     <= 1'b0;
            v3     <= 1'b0;
            v4     <= 1'b0;
            v5     <= 1'b0;
            desc_r <= 256'd0;
        end else begin
            v1 <= (state == R_RUN);
            v2 <= v1;
            v3 <= v2;
            v4 <= v3;
            v5 <= v4;
            if (v5) desc_r <= {cmp_bit, desc_r[255:1]};     // cap dau tien ket thuc o desc[0]
        end
    end

    wire pipe_busy = v1 | v2 | v3 | v4 | v5;

    // ======================================================
    // FSM phan 1: next-state transition logic
    // ======================================================
    always @(*) begin
        next_state = state;
        case (state)
            R_IDLE: begin
                if (start) next_state = R_RUN;
            end
            R_RUN: begin
                if (last_pair) next_state = R_DRAIN;
            end
            R_DRAIN: begin
                if (!pipe_busy) next_state = R_DONE;
            end
            R_DONE:  next_state = R_IDLE;
            default: next_state = R_IDLE;
        endcase
    end

    // ======================================================
    // FSM phan 2: state register update
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= R_IDLE;
        else        state <= next_state;
    end

    // ======================================================
    // Datapath: chot bin, dem cap
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bin_r    <= 5'd0;
            pair_cnt <= 8'd0;
        end else begin
            case (state)
                R_IDLE: begin
                    if (start) begin
                        bin_r    <= angle_bin;
                        pair_cnt <= 8'd0;
                    end
                end
                R_RUN: begin
                    pair_cnt <= pair_cnt + 8'd1;
                end
                default: ;
            endcase
        end
    end

    // ======================================================
    // FSM phan 3: combinational output logic
    // ======================================================
    assign busy = (state != R_IDLE);
    assign done = (state == R_DONE);
    assign desc = desc_r;

endmodule

// ============================================================================
// rbrief_rnd: lam tron tong co FRAC bit le ve so nguyen 6 bit co dau (to hop)
//   fl = sum >>> FRAC (floor), frac = FRAC bit thap. inc = 1 neu frac > 0.5;
//   neu frac == 0.5 thi theo tie:  0 half-even, 1 half-up, 2 half-down,
//   3 xa 0, 4 gan 0.
// ============================================================================
module rbrief_rnd #(
    parameter FRAC = 16,
    parameter SW   = 25
)(
    input  wire signed [SW-1:0] sum,
    input  wire        [2:0]    tie,
    output wire signed [5:0]    r
);
    localparam [FRAC-1:0] HALF = {1'b1, {(FRAC-1){1'b0}}};

    wire signed [SW-1:0] fl   = sum >>> FRAC;
    wire        [FRAC-1:0] frac = sum[FRAC-1:0];
    wire neg  = sum[SW-1];
    wire gt   = (frac > HALF);
    wire eq   = (frac == HALF);

    reg tie_inc;
    always @(*) begin
        case (tie)
            3'd0:    tie_inc = fl[0];
            3'd1:    tie_inc = 1'b1;
            3'd2:    tie_inc = 1'b0;
            3'd3:    tie_inc = ~neg;
            default: tie_inc = neg;
        endcase
    end

    wire inc = gt | (eq & tie_inc);

    assign r = fl[5:0] + {5'b00000, inc};

endmodule
