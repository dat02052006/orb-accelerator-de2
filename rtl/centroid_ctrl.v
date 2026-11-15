module centroid_ctrl #(
    parameter ADDR_W = 10                       // buffer keypoint toi da 2^ADDR_W phan tu
)(
    input  wire                clk,
    input  wire                rst_n,

    // ---- dieu khien ca frame ----
    input  wire                ori_start,       // xung: bat dau xu ly cac keypoint trong buffer
    input  wire [ADDR_W:0]     kp_count,        // so keypoint trong buffer (on dinh suot qua trinh)
    output wire                ori_busy,
    output wire                ori_done,        // xung 1 chu ky khi xu ly het keypoint
    output wire [ADDR_W:0]     out_count,       // so keypoint da ghi ra buffer dau ra

    // ---- buffer keypoint dau vao (keypoint_selector), doc dong bo, tre 1 chu ky ----
    output wire                kp_rd_en,
    output wire [ADDR_W-1:0]   kp_rd_addr,
    input  wire [26:0]         kp_rd_data,      // {score[7:0], y[8:0], x[9:0]}

    // ---- xin / cap quyen SRAM voi FAST (arbiter giu grant cho den khi ha req) ----
    output wire                sram_req,
    input  wire                sram_gnt,

    // ---- centroid_patch_reader ----
    output wire                rd_start,
    output wire [9:0]          rd_kx,
    output wire [8:0]          rd_ky,
    input  wire                rd_busy,
    input  wire                rd_done,
    input  wire                rd_error,

    // ---- centroid_moment_accumulator ----
    output wire                acc_clear,
    input  wire                mom_valid,

    // ---- centroid_angle ----
    output wire                ang_start,
    input  wire                ang_done,
    input  wire [4:0]          ang_bin,

    // ---- rbrief_gauss_window (tu chay sau rd_done, ctrl chi cho gauss_done) ----
    input  wire                gauss_done,

    // ---- rbrief_descriptor ----
    output wire                rb_start,
    output wire [4:0]          rb_angle,
    input  wire                rb_done,
    input  wire [255:0]        rb_desc,

    // ---- buffer dau ra ----
    output wire                out_we,
    output wire [ADDR_W-1:0]   out_addr,
    output wire [9:0]          out_x,
    output wire [8:0]          out_y,
    output wire [7:0]          out_score,
    output wire [4:0]          out_angle,
    output wire [255:0]        out_desc
);

    // vung hop le (vien 21 pixel: ban kinh pattern sau khi xoay 18 + 3 cho bo loc 7x7; anh 640x480)
    localparam [9:0] KX_MIN = 10'd21;
    localparam [9:0] KX_MAX = 10'd618;
    localparam [8:0] KY_MIN = 9'd21;
    localparam [8:0] KY_MAX = 9'd458;

    // ---------------- state encoding ----------------
    localparam [4:0] S_IDLE    = 5'd0,
                     S_CHECK   = 5'd1,
                     S_FETCH   = 5'd2,
                     S_LATCH   = 5'd3,
                     S_RANGE   = 5'd4,
                     S_REQ     = 5'd5,
                     S_START   = 5'd6,
                     S_RDWAIT  = 5'd7,
                     S_MOMWAIT = 5'd8,
                     S_ANGLE   = 5'd9,
                     S_ANGWAIT = 5'd10,
                     S_GWAIT   = 5'd11,
                     S_DESC    = 5'd12,
                     S_DWAIT   = 5'd13,
                     S_WRITE   = 5'd14,
                     S_NEXT    = 5'd15,
                     S_FINISH  = 5'd16;

    reg [4:0] state, next_state;

    // ---------------- datapath registers ----------------
    reg [ADDR_W:0] idx;         // keypoint dang xu ly
    reg [ADDR_W:0] total;       // so keypoint can xu ly
    reg [ADDR_W:0] out_cnt;     // so keypoint da ghi
    reg [9:0]      kx_r;
    reg [8:0]      ky_r;
    reg [7:0]      sc_r;
    reg [4:0]      ang_r;       // bin goc da chot
    reg            g_seen;      // gauss_done da den (co the den truoc khi ctrl toi S_GWAIT)

    wire kx_ok    = (kx_r >= KX_MIN) && (kx_r <= KX_MAX);
    wire ky_ok    = (ky_r >= KY_MIN) && (ky_r <= KY_MAX);
    wire kp_valid = kx_ok && ky_ok;

    // ======================================================
    // FSM phan 1: next-state transition logic
    // ======================================================
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (ori_start) next_state = S_CHECK;
            end
            S_CHECK: begin
                if (idx == total) next_state = S_FINISH;
                else              next_state = S_FETCH;
            end
            S_FETCH: next_state = S_LATCH;
            S_LATCH: next_state = S_RANGE;
            S_RANGE: begin
                if (kp_valid) next_state = S_REQ;
                else          next_state = S_NEXT;          // ngoai vung hop le: bo qua
            end
            S_REQ: begin
                if (sram_gnt && !rd_busy) next_state = S_START;
            end
            S_START: next_state = S_RDWAIT;
            S_RDWAIT: begin
                if (rd_done)       next_state = S_MOMWAIT;
                else if (rd_error) next_state = S_NEXT;
            end
            S_MOMWAIT: begin
                if (mom_valid) next_state = S_ANGLE;
            end
            S_ANGLE:   next_state = S_ANGWAIT;
            S_ANGWAIT: begin
                if (ang_done) next_state = S_GWAIT;
            end
            S_GWAIT: begin
                if (g_seen || gauss_done) next_state = S_DESC;
            end
            S_DESC:  next_state = S_DWAIT;
            S_DWAIT: begin
                if (rb_done) next_state = S_WRITE;
            end
            S_WRITE:  next_state = S_NEXT;
            S_NEXT:   next_state = S_CHECK;
            S_FINISH: next_state = S_IDLE;
            default:  next_state = S_IDLE;
        endcase
    end

    // ======================================================
    // FSM phan 2: state register update
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    // ======================================================
    // Datapath: chi so keypoint, chot du lieu, dem ket qua
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx     <= {(ADDR_W+1){1'b0}};
            total   <= {(ADDR_W+1){1'b0}};
            out_cnt <= {(ADDR_W+1){1'b0}};
            kx_r    <= 10'd0;
            ky_r    <= 9'd0;
            sc_r    <= 8'd0;
            ang_r   <= 5'd0;
            g_seen  <= 1'b0;
        end else begin
            // gauss_done co the den bat cu luc nao sau rd_done; nho lai den khi dung
            if (state == S_START)  g_seen <= 1'b0;
            else if (gauss_done)   g_seen <= 1'b1;

            case (state)
                S_IDLE: begin
                    if (ori_start) begin
                        idx     <= {(ADDR_W+1){1'b0}};
                        out_cnt <= {(ADDR_W+1){1'b0}};
                        total   <= kp_count;
                    end
                end
                S_LATCH: begin
                    kx_r <= kp_rd_data[9:0];
                    ky_r <= kp_rd_data[18:10];
                    sc_r <= kp_rd_data[26:19];
                end
                S_ANGWAIT: begin
                    if (ang_done) ang_r <= ang_bin;
                end
                S_WRITE: begin
                    out_cnt <= out_cnt + {{ADDR_W{1'b0}}, 1'b1};
                end
                S_NEXT: begin
                    idx <= idx + {{ADDR_W{1'b0}}, 1'b1};
                end
                default: ;
            endcase
        end
    end

    // ======================================================
    // FSM phan 3: combinational output logic
    // ======================================================
    assign kp_rd_en   = (state == S_FETCH);
    assign kp_rd_addr = idx[ADDR_W-1:0];

    // giu req tu luc xin den khi patch reader doc xong SRAM
    assign sram_req   = (state == S_REQ) || (state == S_START) || (state == S_RDWAIT);

    assign rd_start   = (state == S_START);
    assign rd_kx      = kx_r;
    assign rd_ky      = ky_r;
    assign acc_clear  = (state == S_START);

    assign ang_start  = (state == S_ANGLE);

    assign rb_start   = (state == S_DESC);
    assign rb_angle   = ang_r;

    assign out_we     = (state == S_WRITE);
    assign out_addr   = out_cnt[ADDR_W-1:0];
    assign out_x      = kx_r;
    assign out_y      = ky_r;
    assign out_score  = sc_r;
    assign out_angle  = ang_r;
    assign out_desc   = rb_desc;

    assign out_count  = out_cnt;
    assign ori_busy   = (state != S_IDLE);
    assign ori_done   = (state == S_FINISH);

endmodule
