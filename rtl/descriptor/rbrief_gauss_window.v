// ============================================================================
// rbrief_gauss_window
//   Lam mo Gaussian 7x7 (sigma = 2) cho cua so keypoint, tach 2 chieu:
//     patch tho 43x43  ->  smoothed 37x37   (tam smoothed = (18,18))
//
//   Pass 1 (ngang) : nghe luong pb_* cua patch reader (2 pixel / word), loc 7 tap ngang,
//                    ghi H[row][hc] (do chinh xac day du) vao RAM noi bo hram
//   Pass 2 (doc)   : sau patch_done, doc hram theo tung cot, loc 7 tap doc,
//                    lam tron 1 lan o cuoi, ghi ra cong ghi smoothed buffer (sm_*)
//
//   He so mac dinh P = 8: {18,34,48,56,48,34,18}/256, lam tron 1 lan o cuoi
//   -> khop tung bit voi cv2.GaussianBlur(7x7, sigma 2) tren anh 8 bit.
//   Muon khop bo loc dau cham dong (ORB cua cv2 5.x): P = 14,
//   K0..K3 = 1149, 2148, 3125, 3540.
//
//   Gia dinh: pb_we cach nhau it nhat 2 chu ky; patch reader xuat 43 hang (0..42),
//   22 word moi hang (0..21), tuong ung 44 cot, cot -1 / cot 43 bi bo qua.
//   Smoothed buffer dia chi = srow*37 + scol.
// ============================================================================
module rbrief_gauss_window #(
    parameter integer P  = 8,
    parameter [P-1:0] K0 = 18,          // |x| = 3
    parameter [P-1:0] K1 = 34,          // |x| = 2
    parameter [P-1:0] K2 = 48,          // |x| = 1
    parameter [P-1:0] K3 = 56           // x = 0
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- luong ghi patch tu patch reader (cung y nghia voi centroid_patch_reader) ----
    input  wire        pb_we,
    input  wire [5:0]  pb_row,          // 0..42
    input  wire [4:0]  pb_word,         // 0..21
    input  wire [15:0] pb_wdata,        // [7:0] = pixel cot chan, [15:8] = pixel cot le
    input  wire        patch_x0_odd,
    input  wire        patch_done,      // done cua patch reader

    // ---- trang thai ----
    output wire        busy,
    output wire        done,            // xung 1 chu ky: smoothed buffer da ghi du 37x37

    // ---- cong ghi smoothed buffer (RAM nam o khoi rBRIEF) ----
    output wire        sm_we,
    output wire [10:0] sm_waddr,        // srow*37 + scol
    output wire [7:0]  sm_wdata
);

    localparam HW    = P + 8;                   // do rong H sau pass ngang
    localparam VW    = 2*P + 8;                 // do rong tong sau pass doc
    localparam [VW-1:0] ROUND = {{(VW-1){1'b0}}, 1'b1} << (2*P - 1);

    // ======================================================
    // Pass 1, stage 1: chot word tu luong pb_*
    // ======================================================
    reg        in_v;
    reg        in_v_d;
    reg [5:0]  in_row;
    reg [4:0]  in_word;
    reg [15:0] in_data;
    reg        in_odd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_v    <= 1'b0;
            in_v_d  <= 1'b0;
            in_row  <= 6'd0;
            in_word <= 5'd0;
            in_data <= 16'd0;
            in_odd  <= 1'b0;
        end else begin
            in_v   <= pb_we;
            in_v_d <= in_v;
            if (pb_we) begin
                in_row  <= pb_row;
                in_word <= pb_word;
                in_data <= pb_wdata;
                in_odd  <= patch_x0_odd;
            end
        end
    end

    // Moi word duoc day vao cua so 7 pixel thanh 2 lan:
    //   in_v   : pixel thap (cot chan) , cot = 2*word - x0_odd
    //   in_v_d : pixel cao  (cot le)   , cot = 2*word - x0_odd + 1
    wire        push    = in_v | in_v_d;
    wire [5:0]  w2      = {in_word, 1'b0};
    wire signed [7:0] col_s = $signed({2'b00, w2}) - $signed({7'd0, in_odd}) + $signed({7'd0, in_v_d});
    wire        col_ok  = (col_s >= 8'sd0) && (col_s <= 8'sd42);     // bo cot -1 va cot 43
    wire        col_ge6 = (col_s >= 8'sd6);                          // du 7 pixel cung hang
    wire [7:0]  pix_in  = in_v_d ? in_data[15:8] : in_data[7:0];

    // ======================================================
    // Pass 1, stage 2: thanh ghi dich 7 pixel
    // ======================================================
    reg [7:0] win0, win1, win2, win3, win4, win5, win6;      // win0 = moi nhat, win6 = cu nhat
    reg       h_v1;
    reg [5:0] h_row1;
    reg [5:0] h_hc1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win0 <= 8'd0; win1 <= 8'd0; win2 <= 8'd0; win3 <= 8'd0;
            win4 <= 8'd0; win5 <= 8'd0; win6 <= 8'd0;
            h_v1   <= 1'b0;
            h_row1 <= 6'd0;
            h_hc1  <= 6'd0;
        end else begin
            h_v1   <= push && col_ok && col_ge6;
            h_row1 <= in_row;
            h_hc1  <= col_s[5:0] - 6'd6;                    // chi so cot ngang 0..36
            if (push && col_ok) begin
                win0 <= pix_in;
                win1 <= win0;
                win2 <= win1;
                win3 <= win2;
                win4 <= win3;
                win5 <= win4;
                win6 <= win5;
            end
        end
    end

    // ======================================================
    // Pass 1, stage 3..5: tong cap doi xung, nhan hang so, cong
    // ======================================================
    wire [10:0] h_row37 = ({5'd0, h_row1} << 5) + ({5'd0, h_row1} << 2) + {5'd0, h_row1};

    reg            h_v2, h_v3, h_v4;
    reg [10:0]     h_a2, h_a3, h_a4;
    reg [8:0]      hs0, hs1, hs2;
    reg [7:0]      hs3;
    reg [HW-1:0]   hm0, hm1, hm2, hm3;
    reg [HW-1:0]   hsum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_v2 <= 1'b0; h_v3 <= 1'b0; h_v4 <= 1'b0;
            h_a2 <= 11'd0; h_a3 <= 11'd0; h_a4 <= 11'd0;
            hs0 <= 9'd0; hs1 <= 9'd0; hs2 <= 9'd0; hs3 <= 8'd0;
            hm0 <= {HW{1'b0}}; hm1 <= {HW{1'b0}}; hm2 <= {HW{1'b0}}; hm3 <= {HW{1'b0}};
            hsum <= {HW{1'b0}};
        end else begin
            // stage 3: tong cap
            h_v2 <= h_v1;
            h_a2 <= h_row37 + {5'd0, h_hc1};
            hs0  <= {1'b0, win0} + {1'b0, win6};
            hs1  <= {1'b0, win1} + {1'b0, win5};
            hs2  <= {1'b0, win2} + {1'b0, win4};
            hs3  <= win3;
            // stage 4: nhan hang so
            h_v3 <= h_v2;
            h_a3 <= h_a2;
            hm0  <= K0 * hs0;
            hm1  <= K1 * hs1;
            hm2  <= K2 * hs2;
            hm3  <= K3 * hs3;
            // stage 5: cong
            h_v4 <= h_v3;
            h_a4 <= h_a3;
            hsum <= hm0 + hm1 + hm2 + hm3;
        end
    end

    wire hpipe_busy = pb_we | in_v | in_v_d | h_v1 | h_v2 | h_v3 | h_v4;

    // ======================================================
    // hram: H[row][hc], 43 x 37 phan tu, ghi o pass 1, doc o pass 2
    // ======================================================
    wire [10:0]   v_raddr;
    wire [HW-1:0] hr_q;

    rbrief_sdp_ram #(
        .DW   (HW),
        .AW   (11),
        .DEPTH(1591)
    ) u_hram (
        .clk  (clk),
        .we   (h_v4),
        .waddr(h_a4),
        .wdata(hsum),
        .raddr(v_raddr),
        .rdata(hr_q)
    );

    // ======================================================
    // Pass 2: FSM dieu khien doc hram theo tung cot
    // ======================================================
    localparam [2:0] V_IDLE  = 3'd0,
                     V_WAIT  = 3'd1,
                     V_RUN   = 3'd2,
                     V_DRAIN = 3'd3,
                     V_DONE  = 3'd4;

    reg [2:0] state, next_state;

    reg [5:0]  r_cnt;               // hang 0..42
    reg [5:0]  c_cnt;               // cot 0..36
    reg [10:0] raddr_r;

    wire last_r = (r_cnt == 6'd42);
    wire last_c = (c_cnt == 6'd36);

    wire        vpipe_busy;

    // ---- FSM phan 1: next-state transition logic ----
    always @(*) begin
        next_state = state;
        case (state)
            V_IDLE: begin
                if (patch_done) next_state = V_WAIT;
            end
            V_WAIT: begin
                if (!hpipe_busy) next_state = V_RUN;
            end
            V_RUN: begin
                if (last_r && last_c) next_state = V_DRAIN;
            end
            V_DRAIN: begin
                if (!vpipe_busy) next_state = V_DONE;
            end
            V_DONE:  next_state = V_IDLE;
            default: next_state = V_IDLE;
        endcase
    end

    // ---- FSM phan 2: state register update ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= V_IDLE;
        else        state <= next_state;
    end

    // ---- datapath: bo dem hang/cot va dia chi doc ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_cnt   <= 6'd0;
            c_cnt   <= 6'd0;
            raddr_r <= 11'd0;
        end else begin
            case (state)
                V_WAIT: begin
                    r_cnt   <= 6'd0;
                    c_cnt   <= 6'd0;
                    raddr_r <= 11'd0;
                end
                V_RUN: begin
                    if (last_r) begin
                        r_cnt   <= 6'd0;
                        c_cnt   <= c_cnt + 6'd1;
                        raddr_r <= {5'd0, c_cnt} + 11'd1;
                    end else begin
                        r_cnt   <= r_cnt + 6'd1;
                        raddr_r <= raddr_r + 11'd37;
                    end
                end
                default: ;
            endcase
        end
    end

    assign v_raddr = raddr_r;

    // ======================================================
    // Pass 2 pipeline
    //   R : hram tra du lieu (tre 1 chu ky so voi dia chi) -> day vao thanh ghi dich doc
    //   2 : tong cap  3 : nhan hang so  4 : cong  5 : lam tron -> ghi smoothed buffer
    // ======================================================
    reg              vr_v1;
    reg [5:0]        vr_r1;
    reg [5:0]        vr_c1;

    reg [HW-1:0]     vwin0, vwin1, vwin2, vwin3, vwin4, vwin5, vwin6;
    reg              vv2;
    reg [5:0]        vv2_r;
    reg [5:0]        vv2_c;

    reg              vv3, vv4, vv5;
    reg [10:0]       va3, va4, va5;
    reg [HW:0]       vp0, vp1, vp2;
    reg [HW-1:0]     vp3;
    reg [VW-1:0]     vq0, vq1, vq2, vq3;
    reg [VW-1:0]     vsum;

    reg              sm_we_r;
    reg [10:0]       sm_waddr_r;
    reg [7:0]        sm_wdata_r;

    wire [10:0] vrow37 = ({5'd0, vv2_r} << 5) + ({5'd0, vv2_r} << 2) + {5'd0, vv2_r};
    wire [VW:0] vround = {1'b0, vsum} + ROUND;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vr_v1 <= 1'b0; vr_r1 <= 6'd0; vr_c1 <= 6'd0;
            vwin0 <= {HW{1'b0}}; vwin1 <= {HW{1'b0}}; vwin2 <= {HW{1'b0}}; vwin3 <= {HW{1'b0}};
            vwin4 <= {HW{1'b0}}; vwin5 <= {HW{1'b0}}; vwin6 <= {HW{1'b0}};
            vv2 <= 1'b0; vv2_r <= 6'd0; vv2_c <= 6'd0;
            vv3 <= 1'b0; vv4 <= 1'b0; vv5 <= 1'b0;
            va3 <= 11'd0; va4 <= 11'd0; va5 <= 11'd0;
            vp0 <= {(HW+1){1'b0}}; vp1 <= {(HW+1){1'b0}}; vp2 <= {(HW+1){1'b0}}; vp3 <= {HW{1'b0}};
            vq0 <= {VW{1'b0}}; vq1 <= {VW{1'b0}}; vq2 <= {VW{1'b0}}; vq3 <= {VW{1'b0}};
            vsum <= {VW{1'b0}};
            sm_we_r <= 1'b0; sm_waddr_r <= 11'd0; sm_wdata_r <= 8'd0;
        end else begin
            // stage R: gan the cho du lieu hram va day vao cua so doc
            vr_v1 <= (state == V_RUN);
            vr_r1 <= r_cnt;
            vr_c1 <= c_cnt;
            if (vr_v1) begin
                vwin0 <= hr_q;
                vwin1 <= vwin0;
                vwin2 <= vwin1;
                vwin3 <= vwin2;
                vwin4 <= vwin3;
                vwin5 <= vwin4;
                vwin6 <= vwin5;
            end
            vv2   <= vr_v1 && (vr_r1 >= 6'd6);               // da du 7 hang cung cot
            vv2_r <= vr_r1 - 6'd6;                           // hang smoothed 0..36
            vv2_c <= vr_c1;

            // stage 2: tong cap doi xung
            vv3 <= vv2;
            va3 <= vrow37 + {5'd0, vv2_c};
            vp0 <= {1'b0, vwin0} + {1'b0, vwin6};
            vp1 <= {1'b0, vwin1} + {1'b0, vwin5};
            vp2 <= {1'b0, vwin2} + {1'b0, vwin4};
            vp3 <= vwin3;

            // stage 3: nhan hang so
            vv4 <= vv3;
            va4 <= va3;
            vq0 <= K0 * vp0;
            vq1 <= K1 * vp1;
            vq2 <= K2 * vp2;
            vq3 <= K3 * vp3;

            // stage 4: cong
            vv5  <= vv4;
            va5  <= va4;
            vsum <= vq0 + vq1 + vq2 + vq3;

            // stage 5: lam tron 1 lan, ghi smoothed buffer
            sm_we_r    <= vv5;
            sm_waddr_r <= va5;
            sm_wdata_r <= vround[2*P+7:2*P];
        end
    end

    assign vpipe_busy = vr_v1 | vv2 | vv3 | vv4 | vv5 | sm_we_r;

    // ======================================================
    // FSM phan 3: combinational output logic
    // ======================================================
    assign sm_we    = sm_we_r;
    assign sm_waddr = sm_waddr_r;
    assign sm_wdata = sm_wdata_r;
    assign busy     = (state != V_IDLE);
    assign done     = (state == V_DONE);

endmodule


// ============================================================================
// RAM 2 cong don gian (1 cong ghi, 1 cong doc dong bo): suy ra M4K tren Cyclone II
// ============================================================================
module rbrief_sdp_ram #(
    parameter DW    = 16,
    parameter AW    = 11,
    parameter DEPTH = 2048
)(
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);

    reg [DW-1:0] mem [0:DEPTH-1] /* synthesis ramstyle = "M4K" */;

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end

endmodule
