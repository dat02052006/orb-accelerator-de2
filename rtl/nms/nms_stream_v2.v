//==============================================================================
// nms_stream_v2.v
// NMS 3x3 streaming, 1 candidate / chu ky, loai tru theo thu tu raster
// (diem den truoc thang khi bang diem).
//
// Cau truc du lieu (thu tu cu -> moi):
//     W0 W1 W2 W3 (cua so 4 slot) <- H (thanh ghi dau hang) <- Q (ngo ra RAM)
//     <- RAM FIFO (M4K)  <- L (candidate vua den)
//   * L, H, W0..W3 deu "nhin thay" duoc boi candidate moi (so sanh song song).
//   * Het han (expire) va pop W0 duoc dieu khien theo VI TRI PIXEL (pix_x,pix_y),
//     khong theo su kien candidate -> cua so khong bi tre.
//   * Entry duoc dua vao H (khong qua RAM) khi hang doi phia sau dang rong,
//     nen khong co khoang "vo hinh" cho entry cuoi hang.
//   * Xa het du lieu khi frame_done (xung 1 chu ky) bang FSM 2 trang thai.
//
// Hop dong giao tiep:
//   - pix_valid=1 cho MOI pixel theo thu tu raster; pix_x tang 1 trong hang.
//   - in_valid chi duoc phep =1 khi pix_valid=1 (cung chu ky, cung pix_x/pix_y).
//   - frame_done: xung sau pixel cuoi; cho flush_done truoc khi bat dau frame moi.
//==============================================================================
module nms_stream_v2 #(
    parameter FIFO_DEPTH = 640,
    parameter ADDR_WIDTH = 10
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        pix_valid,
    input  wire [9:0]  pix_x,
    input  wire [8:0]  pix_y,

    input  wire        in_valid,
    input  wire [7:0]  in_score,

    input  wire        frame_done,

    output reg         kp_valid,
    output reg  [9:0]  kp_x,
    output reg  [8:0]  kp_y,
    output reg  [7:0]  kp_score,

    output wire        flush_done,
    output reg         overflow
);

    //==========================================================================
    // Khai bao thanh ghi
    //==========================================================================
    // L : candidate vua den
    reg        l_vld;
    reg [8:0]  l_y;
    reg [9:0]  l_x;
    reg [7:0]  l_sc;
    reg        l_alv;

    // H : dau hang doi
    reg        h_vld;
    reg [8:0]  h_y;
    reg [9:0]  h_x;
    reg [7:0]  h_sc;
    reg        h_alv;

    // W0..W3 : cua so (W0 cu nhat)
    reg        w0_vld, w1_vld, w2_vld, w3_vld;
    reg [8:0]  w0_y,   w1_y,   w2_y,   w3_y;
    reg [9:0]  w0_x,   w1_x,   w2_x,   w3_x;
    reg [7:0]  w0_sc,  w1_sc,  w2_sc,  w3_sc;
    reg        w0_alv, w1_alv, w2_alv, w3_alv;

    // RAM FIFO + Q
    (* ramstyle = "M4K, no_rw_check" *) reg [27:0] fifo_mem [0:FIFO_DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   rcnt;          // so entry trong RAM chua doc ra Q
    reg [27:0]           q_data;
    reg                  q_vld;

    //==========================================================================
    // FSM xa (3 khoi): next-state / state register / output logic
    //==========================================================================
    localparam S_RUN   = 1'b0;
    localparam S_DRAIN = 1'b1;

    reg  state;
    reg  next_state;
    reg  drain;

    wire empty_all = !l_vld && !h_vld && !q_vld && (rcnt == 0) && !w0_vld;

    // Khoi 1: next-state
    always @(*) begin
        next_state = state;
        case (state)
            S_RUN:   if (frame_done) next_state = S_DRAIN;
            S_DRAIN: if (empty_all)  next_state = S_RUN;
            default:                 next_state = S_RUN;
        endcase
    end

    // Khoi 2: state register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_RUN;
        else        state <= next_state;
    end

    // Khoi 3: output logic (to hop)
    always @(*) begin
        drain = (state == S_DRAIN);
    end
    assign flush_done = drain && empty_all;

    //==========================================================================
    // So sanh candidate moi voi 6 entry quá khu (L, H, W0..W3)
    //==========================================================================
    wire hit_l  = in_valid && l_vld && (
                    ((l_y == pix_y) && (l_x + 10'd1 == pix_x)) ||
                    ((l_y + 9'd1 == pix_y) && (l_x + 10'd1 >= pix_x) && (l_x <= pix_x + 10'd1)));
    wire hit_h  = in_valid && h_vld  && (h_y  + 9'd1 == pix_y) && (h_x  + 10'd1 >= pix_x) && (h_x  <= pix_x + 10'd1);
    wire hit_w0 = in_valid && w0_vld && (w0_y + 9'd1 == pix_y) && (w0_x + 10'd1 >= pix_x) && (w0_x <= pix_x + 10'd1);
    wire hit_w1 = in_valid && w1_vld && (w1_y + 9'd1 == pix_y) && (w1_x + 10'd1 >= pix_x) && (w1_x <= pix_x + 10'd1);
    wire hit_w2 = in_valid && w2_vld && (w2_y + 9'd1 == pix_y) && (w2_x + 10'd1 >= pix_x) && (w2_x <= pix_x + 10'd1);
    wire hit_w3 = in_valid && w3_vld && (w3_y + 9'd1 == pix_y) && (w3_x + 10'd1 >= pix_x) && (w3_x <= pix_x + 10'd1);

    // Candidate moi bi loai neu <= diem bat ky hang xom quá khu
    wire cur_beaten = (hit_l  && (in_score <= l_sc))  ||
                      (hit_h  && (in_score <= h_sc))  ||
                      (hit_w0 && (in_score <= w0_sc)) ||
                      (hit_w1 && (in_score <= w1_sc)) ||
                      (hit_w2 && (in_score <= w2_sc)) ||
                      (hit_w3 && (in_score <= w3_sc));

    // Hang xom quá khu bi loai neu candidate moi lon hon han
    wire kill_l  = hit_l  && (in_score > l_sc);
    wire kill_h  = hit_h  && (in_score > h_sc);
    wire kill_w0 = hit_w0 && (in_score > w0_sc);
    wire kill_w1 = hit_w1 && (in_score > w1_sc);
    wire kill_w2 = hit_w2 && (in_score > w2_sc);
    wire kill_w3 = hit_w3 && (in_score > w3_sc);

    wire h_alv_u  = h_alv  && !kill_h;
    wire w0_alv_u = w0_alv && !kill_w0;
    wire w1_alv_u = w1_alv && !kill_w1;
    wire w2_alv_u = w2_alv && !kill_w2;
    wire w3_alv_u = w3_alv && !kill_w3;

    //==========================================================================
    // Het han theo vi tri pixel, pop cua so, di chuyen H / Q / RAM
    //==========================================================================
    wire w0_expired = pix_valid && ((pix_y > w0_y + 9'd1) ||
                                    ((pix_y == w0_y + 9'd1) && (pix_x > w0_x + 10'd1)));
    wire pop        = w0_vld && (w0_expired || drain);

    wire win_acc    = !w3_vld || pop;           // cua so co the nhan 1 entry moi
    wire h_move     = h_vld && win_acc;         // H -> cua so
    wire h_can_take = !h_vld || h_move;         // H co the nhan entry moi
    wire q_to_h     = q_vld && h_can_take;      // Q -> H

    wire push_ev    = (in_valid && l_vld) || (drain && l_vld);   // L -> hang doi
    wire q_empty_behind = (rcnt == 0) && !q_vld;
    wire push_to_h  = push_ev && q_empty_behind && h_can_take;
    wire push_to_ram= push_ev && !push_to_h;

    wire ram_full   = (rcnt >= FIFO_DEPTH);
    wire ram_we     = push_to_ram && !ram_full;
    wire rd_issue   = (rcnt != 0) && (!q_vld || q_to_h);

    wire [27:0] entry_l = {l_y, l_x, l_sc, (l_alv && !kill_l)};

    wire [8:0] q_y   = q_data[27:19];
    wire [9:0] q_x   = q_data[18:9];
    wire [7:0] q_sc  = q_data[8:1];
    wire       q_alv = q_data[0];

    // Vi tri chen H vao cua so (cua so luon duoc nen, nhiet ke)
    wire ins  = h_move;
    wire ins0 = ins && (pop ? !w1_vld : !w0_vld);
    wire ins1 = ins && (pop ? (w1_vld && !w2_vld) : (w0_vld && !w1_vld));
    wire ins2 = ins && (pop ? (w2_vld && !w3_vld) : (w1_vld && !w2_vld));
    wire ins3 = ins && (pop ? w3_vld : (w2_vld && !w3_vld));

    //==========================================================================
    // RAM FIFO (ghi/doc dong bo, khong reset)
    //==========================================================================
    always @(posedge clk) begin
        if (ram_we)
            fifo_mem[wr_ptr] <= entry_l;
        if (rd_issue)
            q_data <= fifo_mem[rd_ptr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr   <= 0;
            rd_ptr   <= 0;
            rcnt     <= 0;
            q_vld    <= 1'b0;
            overflow <= 1'b0;
        end else begin
            if (ram_we)
                wr_ptr <= (wr_ptr == FIFO_DEPTH - 1) ? 0 : (wr_ptr + 1'b1);
            if (rd_issue)
                rd_ptr <= (rd_ptr == FIFO_DEPTH - 1) ? 0 : (rd_ptr + 1'b1);

            case ({ram_we, rd_issue})
                2'b10:   rcnt <= rcnt + 1'b1;
                2'b01:   rcnt <= rcnt - 1'b1;
                default: rcnt <= rcnt;
            endcase

            if (rd_issue)      q_vld <= 1'b1;
            else if (q_to_h)   q_vld <= 1'b0;

            if (push_to_ram && ram_full)
                overflow <= 1'b1;
        end
    end

    //==========================================================================
    // L
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l_vld <= 1'b0; l_y <= 9'd0; l_x <= 10'd0; l_sc <= 8'd0; l_alv <= 1'b0;
        end else if (in_valid) begin
            l_vld <= 1'b1; l_y <= pix_y; l_x <= pix_x; l_sc <= in_score; l_alv <= !cur_beaten;
        end else if (drain && l_vld) begin
            l_vld <= 1'b0;
        end
    end

    //==========================================================================
    // H
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_vld <= 1'b0; h_y <= 9'd0; h_x <= 10'd0; h_sc <= 8'd0; h_alv <= 1'b0;
        end else if (push_to_h) begin
            h_vld <= 1'b1;
            {h_y, h_x, h_sc, h_alv} <= entry_l;
        end else if (q_to_h) begin
            h_vld <= 1'b1;
            h_y <= q_y; h_x <= q_x; h_sc <= q_sc; h_alv <= q_alv;
        end else if (h_move) begin
            h_vld <= 1'b0;
        end else begin
            h_alv <= h_alv_u;
        end
    end

    //==========================================================================
    // Cua so W0..W3
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w0_vld <= 1'b0; w0_y <= 9'd0; w0_x <= 10'd0; w0_sc <= 8'd0; w0_alv <= 1'b0;
            w1_vld <= 1'b0; w1_y <= 9'd0; w1_x <= 10'd0; w1_sc <= 8'd0; w1_alv <= 1'b0;
            w2_vld <= 1'b0; w2_y <= 9'd0; w2_x <= 10'd0; w2_sc <= 8'd0; w2_alv <= 1'b0;
            w3_vld <= 1'b0; w3_y <= 9'd0; w3_x <= 10'd0; w3_sc <= 8'd0; w3_alv <= 1'b0;
        end else begin
            // ---- valid (nhiet ke) ----
            w0_vld <= ins0 | (pop ? w1_vld : w0_vld);
            w1_vld <= ins1 | (pop ? w2_vld : w1_vld);
            w2_vld <= ins2 | (pop ? w3_vld : w2_vld);
            w3_vld <= ins3 | (pop ? 1'b0   : w3_vld);

            // ---- slot 0 ----
            if (ins0) begin
                w0_y <= h_y; w0_x <= h_x; w0_sc <= h_sc; w0_alv <= h_alv_u;
            end else if (pop) begin
                w0_y <= w1_y; w0_x <= w1_x; w0_sc <= w1_sc; w0_alv <= w1_alv_u;
            end else begin
                w0_alv <= w0_alv_u;
            end

            // ---- slot 1 ----
            if (ins1) begin
                w1_y <= h_y; w1_x <= h_x; w1_sc <= h_sc; w1_alv <= h_alv_u;
            end else if (pop) begin
                w1_y <= w2_y; w1_x <= w2_x; w1_sc <= w2_sc; w1_alv <= w2_alv_u;
            end else begin
                w1_alv <= w1_alv_u;
            end

            // ---- slot 2 ----
            if (ins2) begin
                w2_y <= h_y; w2_x <= h_x; w2_sc <= h_sc; w2_alv <= h_alv_u;
            end else if (pop) begin
                w2_y <= w3_y; w2_x <= w3_x; w2_sc <= w3_sc; w2_alv <= w3_alv_u;
            end else begin
                w2_alv <= w2_alv_u;
            end

            // ---- slot 3 ----
            if (ins3) begin
                w3_y <= h_y; w3_x <= h_x; w3_sc <= h_sc; w3_alv <= h_alv_u;
            end else begin
                w3_alv <= w3_alv_u;
            end
        end
    end

    //==========================================================================
    // Ngo ra keypoint: entry song sot khi bi pop
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_valid <= 1'b0; kp_x <= 10'd0; kp_y <= 9'd0; kp_score <= 8'd0;
        end else begin
            kp_valid <= pop && w0_alv_u;
            kp_x     <= w0_x;
            kp_y     <= w0_y;
            kp_score <= w0_sc;
        end
    end

endmodule
