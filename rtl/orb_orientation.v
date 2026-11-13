//------------------------------------------------------------------------------
// Top module
//------------------------------------------------------------------------------
(* multstyle = "logic" *)
module orb_orientation (
    input  wire               clk,
    input  wire               rst,          // synchronous, active high

    // grayscale pixel stream, raster order, pix_x/pix_y belong to pix_data
    input  wire               pix_valid,
    input  wire [9:0]         pix_x,
    input  wire [8:0]         pix_y,
    input  wire [7:0]         pix_data,

    // keypoints from NMS (raster order)
    input  wire               kp_valid,
    input  wire [9:0]         kp_x,
    input  wire [8:0]         kp_y,
    input  wire [7:0]         kp_score,

    // result (1 clock pulse)
    output reg                ori_valid,
    output reg  [9:0]         ori_x,
    output reg  [8:0]         ori_y,
    output reg  [7:0]         ori_score,
    output reg  [4:0]         ori_sector,   // 0..29, 12 deg each
    output reg  signed [21:0] ori_m10,      // debug
    output reg  signed [21:0] ori_m01,      // debug
    output reg  [15:0]        drop_cnt      // FIFO full + late keypoints
);

    localparam [9:0] SAFE_X = 10'd600;      // on row y+16 start only if sx < SAFE_X

    //--------------------------------------------------------------------------
    // Stream position = last pixel already written into the ring
    //--------------------------------------------------------------------------
    reg [9:0] sx;
    reg [8:0] sy;
    reg       frame_done;

    wire sof = pix_valid & (pix_x == 10'd0)   & (pix_y == 9'd0);
    wire eof = pix_valid & (pix_x == 10'd639) & (pix_y == 9'd479);

    always @(posedge clk) begin
        if (rst) begin
            sx         <= 10'd0;
            sy         <= 9'd0;
            frame_done <= 1'b0;
        end else if (pix_valid) begin
            sx <= pix_x;
            sy <= pix_y;
            if (sof)
                frame_done <= 1'b0;
            else if (eof)
                frame_done <= 1'b1;
        end
    end

    //--------------------------------------------------------------------------
    // Keypoint FIFO (512 x 27) and head register
    //--------------------------------------------------------------------------
    (* ramstyle = "M4K" *) reg [26:0] fifo_mem [0:511];
    reg [8:0]  wr_ptr, rd_ptr;
    reg [9:0]  fifo_cnt;
    reg [26:0] fifo_q;
    reg        fetching;
    reg        hv;
    reg [9:0]  h_x;
    reg [8:0]  h_y;
    reg [7:0]  h_score;

    wire kp_ok     = (kp_x >= 10'd15) & (kp_x <= 10'd624) &
                     (kp_y >= 9'd15)  & (kp_y <= 9'd464);
    wire fifo_full = fifo_cnt[9];
    wire fifo_we   = kp_valid & kp_ok & ~fifo_full;
    wire drop_full = kp_valid & kp_ok & fifo_full;

    always @(posedge clk) begin
        if (fifo_we)
            fifo_mem[wr_ptr] <= {kp_x, kp_y, kp_score};
        fifo_q <= fifo_mem[rd_ptr];
    end

    //--------------------------------------------------------------------------
    // Due / late decision for the head keypoint
    //--------------------------------------------------------------------------
    wire [9:0] h_dx    = h_x + 10'd15;
    wire [9:0] h_dy    = {1'b0, h_y} + 10'd15;
    wire [9:0] h_dy1   = h_dy + 10'd1;
    wire [9:0] sy10    = {1'b0, sy};
    wire       at_row  = (sy10 == h_dy);
    wire       at_row1 = (sy10 == h_dy1);
    wire       past    = (sy10 >  h_dy1);
    wire       x_safe  = frame_done | (sx < SAFE_X);
    wire       late    = past | (at_row1 & ~x_safe);
    wire       due     = (at_row & (sx >= h_dx)) | (at_row1 & x_safe);
    wire       drop_late = hv & late;

    //--------------------------------------------------------------------------
    // Sweep control: 31 columns, u = -15 .. +15
    //--------------------------------------------------------------------------
    reg        busy;
    reg [4:0]  j;
    reg [9:0]  w_x;
    reg [8:0]  w_y;
    reg [7:0]  w_score;
    reg [9:0]  col_base;
    reg [4:0]  w_ky;
    reg [9:0]  rd_addr;
    reg [9:0]  r_x;
    reg [8:0]  r_y;
    reg [7:0]  r_score;

    // cycle-1 control (aligned with rd_addr)
    reg        f1, l1;
    reg [4:0]  u1, k1;

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr   <= 9'd0;
            rd_ptr   <= 9'd0;
            fifo_cnt <= 10'd0;
            fetching <= 1'b0;
            hv       <= 1'b0;
            busy     <= 1'b0;
            j        <= 5'd0;
            f1       <= 1'b0;
            l1       <= 1'b0;
            drop_cnt <= 16'd0;
        end else begin
            f1 <= 1'b0;
            l1 <= 1'b0;
            drop_cnt <= drop_cnt + {15'd0, drop_full} + {15'd0, drop_late};

            if (sof) begin
                // new frame: forget everything that is left over
                wr_ptr   <= 9'd0;
                rd_ptr   <= 9'd0;
                fifo_cnt <= 10'd0;
                fetching <= 1'b0;
                hv       <= 1'b0;
                busy     <= 1'b0;
            end else begin
                // FIFO write side
                if (fifo_we)
                    wr_ptr <= wr_ptr + 9'd1;
                fifo_cnt <= fifo_cnt + {9'd0, fifo_we} - {9'd0, fetching};

                // FIFO read side -> head register (2 clocks per keypoint)
                if (fetching) begin
                    h_x      <= fifo_q[26:17];
                    h_y      <= fifo_q[16:8];
                    h_score  <= fifo_q[7:0];
                    hv       <= 1'b1;
                    rd_ptr   <= rd_ptr + 9'd1;
                    fetching <= 1'b0;
                end else if (~hv & (fifo_cnt != 10'd0)) begin
                    fetching <= 1'b1;
                end

                // head keypoint: drop if late, start sweep if due
                if (hv) begin
                    if (late) begin
                        hv <= 1'b0;
                    end else if (due & ~busy) begin
                        hv       <= 1'b0;
                        busy     <= 1'b1;
                        j        <= 5'd0;
                        w_x      <= h_x;
                        w_y      <= h_y;
                        w_score  <= h_score;
                        col_base <= h_x - 10'd15;
                        w_ky     <= h_y[4:0];
                    end
                end

                // column sweep
                if (busy) begin
                    rd_addr <= col_base + {5'd0, j};
                    f1      <= (j == 5'd0);
                    l1      <= (j == 5'd30);
                    u1      <= j - 5'd15;          // two's complement, -15..+15
                    k1      <= w_ky;
                    if (j == 5'd30) begin
                        busy    <= 1'b0;
                        r_x     <= w_x;
                        r_y     <= w_y;
                        r_score <= w_score;
                    end else begin
                        j <= j + 5'd1;
                    end
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // umax table equivalent (combinational, replaces former function umax_f)
    //--------------------------------------------------------------------------
    wire [3:0] u1_abs = u1[4] ? (~u1[3:0] + 4'd1) : u1[3:0];

    // umax lookup for each possible |d| = 0..15
    // used later by each of the 32 slots
    reg [3:0] umax0, umax1, umax2, umax3, umax4, umax5, umax6, umax7;
    reg [3:0] umax8, umax9, umax10, umax11, umax12, umax13, umax14, umax15;

    always @(*) begin
        // |d| = 0
        umax0  = 4'd15;
        // |d| = 1
        umax1  = 4'd15;
        // |d| = 2
        umax2  = 4'd15;
        // |d| = 3
        umax3  = 4'd15;
        // |d| = 4
        umax4  = 4'd14;
        // |d| = 5
        umax5  = 4'd14;
        // |d| = 6
        umax6  = 4'd14;
        // |d| = 7
        umax7  = 4'd13;
        // |d| = 8
        umax8  = 4'd13;
        // |d| = 9
        umax9  = 4'd12;
        // |d| = 10
        umax10 = 4'd11;
        // |d| = 11
        umax11 = 4'd10;
        // |d| = 12
        umax12 = 4'd9;
        // |d| = 13
        umax13 = 4'd8;
        // |d| = 14
        umax14 = 4'd6;
        // |d| = 15
        umax15 = 4'd3;
    end

    //--------------------------------------------------------------------------
    // Ring RAMs + mask / weight / adder trees (fully unrolled, no generate)
    //--------------------------------------------------------------------------
    wire [255:0] q_bus;          // cycle 2: 32 pixels of the addressed column
    wire [31:0]  mask_w;         // cycle 1
    wire [255:0] mp_w;           // cycle 2
    wire [415:0] p_w;            // cycle 3: 32 x 13 bit signed d*pixel
    wire [79:0]  cs_w;           // cycle 3: 8 x 10 bit
    wire [119:0] ws_w;           // cycle 4: 8 x 15 bit signed

    reg  [31:0]  mask2;
    reg  [255:0] mp3;
    reg  [415:0] p4;
    reg  [79:0]  cs4;
    reg  [119:0] ws5;
    reg  [23:0]  cs5;

    reg  [4:0]   u2, u3, u4, u5, u6;
    reg  [4:0]   k2, k3;
    reg          f2, f3, f4, f5, f6, f7;
    reg          l2, l3, l4, l5, l6, l7;
    reg          res_v;

    //----- Slot 0 -----
    orb_ori_rowram u_ram0 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd0)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[7:0])
    );
    wire [4:0] d1_0  = 5'd0 - k1;
    wire [4:0] ad1_0 = d1_0[4] ? (~d1_0 + 5'd1) : d1_0;
    wire [3:0] um_0  = (ad1_0[3:0] == 4'd0)  ? umax0  :
                       (ad1_0[3:0] == 4'd1)  ? umax1  :
                       (ad1_0[3:0] == 4'd2)  ? umax2  :
                       (ad1_0[3:0] == 4'd3)  ? umax3  :
                       (ad1_0[3:0] == 4'd4)  ? umax4  :
                       (ad1_0[3:0] == 4'd5)  ? umax5  :
                       (ad1_0[3:0] == 4'd6)  ? umax6  :
                       (ad1_0[3:0] == 4'd7)  ? umax7  :
                       (ad1_0[3:0] == 4'd8)  ? umax8  :
                       (ad1_0[3:0] == 4'd9)  ? umax9  :
                       (ad1_0[3:0] == 4'd10) ? umax10 :
                       (ad1_0[3:0] == 4'd11) ? umax11 :
                       (ad1_0[3:0] == 4'd12) ? umax12 :
                       (ad1_0[3:0] == 4'd13) ? umax13 :
                       (ad1_0[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[0] = ~ad1_0[4] & (u1_abs <= um_0);
    assign mp_w[7:0] = q_bus[7:0] & {8{mask2[0]}};
    wire signed [4:0]  d3_0  = 5'd0 - k3;
    wire signed [8:0]  mp_0  = {1'b0, mp3[7:0]};
    wire signed [13:0] pr_0  = d3_0 * mp_0;
    assign p_w[12:0] = pr_0[12:0];

    //----- Slot 1 -----
    orb_ori_rowram u_ram1 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd1)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[15:8])
    );
    wire [4:0] d1_1  = 5'd1 - k1;
    wire [4:0] ad1_1 = d1_1[4] ? (~d1_1 + 5'd1) : d1_1;
    wire [3:0] um_1  = (ad1_1[3:0] == 4'd0)  ? umax0  :
                       (ad1_1[3:0] == 4'd1)  ? umax1  :
                       (ad1_1[3:0] == 4'd2)  ? umax2  :
                       (ad1_1[3:0] == 4'd3)  ? umax3  :
                       (ad1_1[3:0] == 4'd4)  ? umax4  :
                       (ad1_1[3:0] == 4'd5)  ? umax5  :
                       (ad1_1[3:0] == 4'd6)  ? umax6  :
                       (ad1_1[3:0] == 4'd7)  ? umax7  :
                       (ad1_1[3:0] == 4'd8)  ? umax8  :
                       (ad1_1[3:0] == 4'd9)  ? umax9  :
                       (ad1_1[3:0] == 4'd10) ? umax10 :
                       (ad1_1[3:0] == 4'd11) ? umax11 :
                       (ad1_1[3:0] == 4'd12) ? umax12 :
                       (ad1_1[3:0] == 4'd13) ? umax13 :
                       (ad1_1[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[1] = ~ad1_1[4] & (u1_abs <= um_1);
    assign mp_w[15:8] = q_bus[15:8] & {8{mask2[1]}};
    wire signed [4:0]  d3_1  = 5'd1 - k3;
    wire signed [8:0]  mp_1  = {1'b0, mp3[15:8]};
    wire signed [13:0] pr_1  = d3_1 * mp_1;
    assign p_w[25:13] = pr_1[12:0];

    //----- Slot 2 -----
    orb_ori_rowram u_ram2 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd2)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[23:16])
    );
    wire [4:0] d1_2  = 5'd2 - k1;
    wire [4:0] ad1_2 = d1_2[4] ? (~d1_2 + 5'd1) : d1_2;
    wire [3:0] um_2  = (ad1_2[3:0] == 4'd0)  ? umax0  :
                       (ad1_2[3:0] == 4'd1)  ? umax1  :
                       (ad1_2[3:0] == 4'd2)  ? umax2  :
                       (ad1_2[3:0] == 4'd3)  ? umax3  :
                       (ad1_2[3:0] == 4'd4)  ? umax4  :
                       (ad1_2[3:0] == 4'd5)  ? umax5  :
                       (ad1_2[3:0] == 4'd6)  ? umax6  :
                       (ad1_2[3:0] == 4'd7)  ? umax7  :
                       (ad1_2[3:0] == 4'd8)  ? umax8  :
                       (ad1_2[3:0] == 4'd9)  ? umax9  :
                       (ad1_2[3:0] == 4'd10) ? umax10 :
                       (ad1_2[3:0] == 4'd11) ? umax11 :
                       (ad1_2[3:0] == 4'd12) ? umax12 :
                       (ad1_2[3:0] == 4'd13) ? umax13 :
                       (ad1_2[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[2] = ~ad1_2[4] & (u1_abs <= um_2);
    assign mp_w[23:16] = q_bus[23:16] & {8{mask2[2]}};
    wire signed [4:0]  d3_2  = 5'd2 - k3;
    wire signed [8:0]  mp_2  = {1'b0, mp3[23:16]};
    wire signed [13:0] pr_2  = d3_2 * mp_2;
    assign p_w[38:26] = pr_2[12:0];

    //----- Slot 3 -----
    orb_ori_rowram u_ram3 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd3)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[31:24])
    );
    wire [4:0] d1_3  = 5'd3 - k1;
    wire [4:0] ad1_3 = d1_3[4] ? (~d1_3 + 5'd1) : d1_3;
    wire [3:0] um_3  = (ad1_3[3:0] == 4'd0)  ? umax0  :
                       (ad1_3[3:0] == 4'd1)  ? umax1  :
                       (ad1_3[3:0] == 4'd2)  ? umax2  :
                       (ad1_3[3:0] == 4'd3)  ? umax3  :
                       (ad1_3[3:0] == 4'd4)  ? umax4  :
                       (ad1_3[3:0] == 4'd5)  ? umax5  :
                       (ad1_3[3:0] == 4'd6)  ? umax6  :
                       (ad1_3[3:0] == 4'd7)  ? umax7  :
                       (ad1_3[3:0] == 4'd8)  ? umax8  :
                       (ad1_3[3:0] == 4'd9)  ? umax9  :
                       (ad1_3[3:0] == 4'd10) ? umax10 :
                       (ad1_3[3:0] == 4'd11) ? umax11 :
                       (ad1_3[3:0] == 4'd12) ? umax12 :
                       (ad1_3[3:0] == 4'd13) ? umax13 :
                       (ad1_3[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[3] = ~ad1_3[4] & (u1_abs <= um_3);
    assign mp_w[31:24] = q_bus[31:24] & {8{mask2[3]}};
    wire signed [4:0]  d3_3  = 5'd3 - k3;
    wire signed [8:0]  mp_3  = {1'b0, mp3[31:24]};
    wire signed [13:0] pr_3  = d3_3 * mp_3;
    assign p_w[51:39] = pr_3[12:0];

    //----- Slot 4 -----
    orb_ori_rowram u_ram4 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd4)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[39:32])
    );
    wire [4:0] d1_4  = 5'd4 - k1;
    wire [4:0] ad1_4 = d1_4[4] ? (~d1_4 + 5'd1) : d1_4;
    wire [3:0] um_4  = (ad1_4[3:0] == 4'd0)  ? umax0  :
                       (ad1_4[3:0] == 4'd1)  ? umax1  :
                       (ad1_4[3:0] == 4'd2)  ? umax2  :
                       (ad1_4[3:0] == 4'd3)  ? umax3  :
                       (ad1_4[3:0] == 4'd4)  ? umax4  :
                       (ad1_4[3:0] == 4'd5)  ? umax5  :
                       (ad1_4[3:0] == 4'd6)  ? umax6  :
                       (ad1_4[3:0] == 4'd7)  ? umax7  :
                       (ad1_4[3:0] == 4'd8)  ? umax8  :
                       (ad1_4[3:0] == 4'd9)  ? umax9  :
                       (ad1_4[3:0] == 4'd10) ? umax10 :
                       (ad1_4[3:0] == 4'd11) ? umax11 :
                       (ad1_4[3:0] == 4'd12) ? umax12 :
                       (ad1_4[3:0] == 4'd13) ? umax13 :
                       (ad1_4[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[4] = ~ad1_4[4] & (u1_abs <= um_4);
    assign mp_w[39:32] = q_bus[39:32] & {8{mask2[4]}};
    wire signed [4:0]  d3_4  = 5'd4 - k3;
    wire signed [8:0]  mp_4  = {1'b0, mp3[39:32]};
    wire signed [13:0] pr_4  = d3_4 * mp_4;
    assign p_w[64:52] = pr_4[12:0];

    //----- Slot 5 -----
    orb_ori_rowram u_ram5 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd5)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[47:40])
    );
    wire [4:0] d1_5  = 5'd5 - k1;
    wire [4:0] ad1_5 = d1_5[4] ? (~d1_5 + 5'd1) : d1_5;
    wire [3:0] um_5  = (ad1_5[3:0] == 4'd0)  ? umax0  :
                       (ad1_5[3:0] == 4'd1)  ? umax1  :
                       (ad1_5[3:0] == 4'd2)  ? umax2  :
                       (ad1_5[3:0] == 4'd3)  ? umax3  :
                       (ad1_5[3:0] == 4'd4)  ? umax4  :
                       (ad1_5[3:0] == 4'd5)  ? umax5  :
                       (ad1_5[3:0] == 4'd6)  ? umax6  :
                       (ad1_5[3:0] == 4'd7)  ? umax7  :
                       (ad1_5[3:0] == 4'd8)  ? umax8  :
                       (ad1_5[3:0] == 4'd9)  ? umax9  :
                       (ad1_5[3:0] == 4'd10) ? umax10 :
                       (ad1_5[3:0] == 4'd11) ? umax11 :
                       (ad1_5[3:0] == 4'd12) ? umax12 :
                       (ad1_5[3:0] == 4'd13) ? umax13 :
                       (ad1_5[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[5] = ~ad1_5[4] & (u1_abs <= um_5);
    assign mp_w[47:40] = q_bus[47:40] & {8{mask2[5]}};
    wire signed [4:0]  d3_5  = 5'd5 - k3;
    wire signed [8:0]  mp_5  = {1'b0, mp3[47:40]};
    wire signed [13:0] pr_5  = d3_5 * mp_5;
    assign p_w[77:65] = pr_5[12:0];

    //----- Slot 6 -----
    orb_ori_rowram u_ram6 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd6)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[55:48])
    );
    wire [4:0] d1_6  = 5'd6 - k1;
    wire [4:0] ad1_6 = d1_6[4] ? (~d1_6 + 5'd1) : d1_6;
    wire [3:0] um_6  = (ad1_6[3:0] == 4'd0)  ? umax0  :
                       (ad1_6[3:0] == 4'd1)  ? umax1  :
                       (ad1_6[3:0] == 4'd2)  ? umax2  :
                       (ad1_6[3:0] == 4'd3)  ? umax3  :
                       (ad1_6[3:0] == 4'd4)  ? umax4  :
                       (ad1_6[3:0] == 4'd5)  ? umax5  :
                       (ad1_6[3:0] == 4'd6)  ? umax6  :
                       (ad1_6[3:0] == 4'd7)  ? umax7  :
                       (ad1_6[3:0] == 4'd8)  ? umax8  :
                       (ad1_6[3:0] == 4'd9)  ? umax9  :
                       (ad1_6[3:0] == 4'd10) ? umax10 :
                       (ad1_6[3:0] == 4'd11) ? umax11 :
                       (ad1_6[3:0] == 4'd12) ? umax12 :
                       (ad1_6[3:0] == 4'd13) ? umax13 :
                       (ad1_6[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[6] = ~ad1_6[4] & (u1_abs <= um_6);
    assign mp_w[55:48] = q_bus[55:48] & {8{mask2[6]}};
    wire signed [4:0]  d3_6  = 5'd6 - k3;
    wire signed [8:0]  mp_6  = {1'b0, mp3[55:48]};
    wire signed [13:0] pr_6  = d3_6 * mp_6;
    assign p_w[90:78] = pr_6[12:0];

    //----- Slot 7 -----
    orb_ori_rowram u_ram7 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd7)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[63:56])
    );
    wire [4:0] d1_7  = 5'd7 - k1;
    wire [4:0] ad1_7 = d1_7[4] ? (~d1_7 + 5'd1) : d1_7;
    wire [3:0] um_7  = (ad1_7[3:0] == 4'd0)  ? umax0  :
                       (ad1_7[3:0] == 4'd1)  ? umax1  :
                       (ad1_7[3:0] == 4'd2)  ? umax2  :
                       (ad1_7[3:0] == 4'd3)  ? umax3  :
                       (ad1_7[3:0] == 4'd4)  ? umax4  :
                       (ad1_7[3:0] == 4'd5)  ? umax5  :
                       (ad1_7[3:0] == 4'd6)  ? umax6  :
                       (ad1_7[3:0] == 4'd7)  ? umax7  :
                       (ad1_7[3:0] == 4'd8)  ? umax8  :
                       (ad1_7[3:0] == 4'd9)  ? umax9  :
                       (ad1_7[3:0] == 4'd10) ? umax10 :
                       (ad1_7[3:0] == 4'd11) ? umax11 :
                       (ad1_7[3:0] == 4'd12) ? umax12 :
                       (ad1_7[3:0] == 4'd13) ? umax13 :
                       (ad1_7[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[7] = ~ad1_7[4] & (u1_abs <= um_7);
    assign mp_w[63:56] = q_bus[63:56] & {8{mask2[7]}};
    wire signed [4:0]  d3_7  = 5'd7 - k3;
    wire signed [8:0]  mp_7  = {1'b0, mp3[63:56]};
    wire signed [13:0] pr_7  = d3_7 * mp_7;
    assign p_w[103:91] = pr_7[12:0];

    //----- Slot 8 -----
    orb_ori_rowram u_ram8 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd8)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[71:64])
    );
    wire [4:0] d1_8  = 5'd8 - k1;
    wire [4:0] ad1_8 = d1_8[4] ? (~d1_8 + 5'd1) : d1_8;
    wire [3:0] um_8  = (ad1_8[3:0] == 4'd0)  ? umax0  :
                       (ad1_8[3:0] == 4'd1)  ? umax1  :
                       (ad1_8[3:0] == 4'd2)  ? umax2  :
                       (ad1_8[3:0] == 4'd3)  ? umax3  :
                       (ad1_8[3:0] == 4'd4)  ? umax4  :
                       (ad1_8[3:0] == 4'd5)  ? umax5  :
                       (ad1_8[3:0] == 4'd6)  ? umax6  :
                       (ad1_8[3:0] == 4'd7)  ? umax7  :
                       (ad1_8[3:0] == 4'd8)  ? umax8  :
                       (ad1_8[3:0] == 4'd9)  ? umax9  :
                       (ad1_8[3:0] == 4'd10) ? umax10 :
                       (ad1_8[3:0] == 4'd11) ? umax11 :
                       (ad1_8[3:0] == 4'd12) ? umax12 :
                       (ad1_8[3:0] == 4'd13) ? umax13 :
                       (ad1_8[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[8] = ~ad1_8[4] & (u1_abs <= um_8);
    assign mp_w[71:64] = q_bus[71:64] & {8{mask2[8]}};
    wire signed [4:0]  d3_8  = 5'd8 - k3;
    wire signed [8:0]  mp_8  = {1'b0, mp3[71:64]};
    wire signed [13:0] pr_8  = d3_8 * mp_8;
    assign p_w[116:104] = pr_8[12:0];

    //----- Slot 9 -----
    orb_ori_rowram u_ram9 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd9)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[79:72])
    );
    wire [4:0] d1_9  = 5'd9 - k1;
    wire [4:0] ad1_9 = d1_9[4] ? (~d1_9 + 5'd1) : d1_9;
    wire [3:0] um_9  = (ad1_9[3:0] == 4'd0)  ? umax0  :
                       (ad1_9[3:0] == 4'd1)  ? umax1  :
                       (ad1_9[3:0] == 4'd2)  ? umax2  :
                       (ad1_9[3:0] == 4'd3)  ? umax3  :
                       (ad1_9[3:0] == 4'd4)  ? umax4  :
                       (ad1_9[3:0] == 4'd5)  ? umax5  :
                       (ad1_9[3:0] == 4'd6)  ? umax6  :
                       (ad1_9[3:0] == 4'd7)  ? umax7  :
                       (ad1_9[3:0] == 4'd8)  ? umax8  :
                       (ad1_9[3:0] == 4'd9)  ? umax9  :
                       (ad1_9[3:0] == 4'd10) ? umax10 :
                       (ad1_9[3:0] == 4'd11) ? umax11 :
                       (ad1_9[3:0] == 4'd12) ? umax12 :
                       (ad1_9[3:0] == 4'd13) ? umax13 :
                       (ad1_9[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[9] = ~ad1_9[4] & (u1_abs <= um_9);
    assign mp_w[79:72] = q_bus[79:72] & {8{mask2[9]}};
    wire signed [4:0]  d3_9  = 5'd9 - k3;
    wire signed [8:0]  mp_9  = {1'b0, mp3[79:72]};
    wire signed [13:0] pr_9  = d3_9 * mp_9;
    assign p_w[129:117] = pr_9[12:0];

    //----- Slot 10 -----
    orb_ori_rowram u_ram10 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd10)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[87:80])
    );
    wire [4:0] d1_10  = 5'd10 - k1;
    wire [4:0] ad1_10 = d1_10[4] ? (~d1_10 + 5'd1) : d1_10;
    wire [3:0] um_10  = (ad1_10[3:0] == 4'd0)  ? umax0  :
                        (ad1_10[3:0] == 4'd1)  ? umax1  :
                        (ad1_10[3:0] == 4'd2)  ? umax2  :
                        (ad1_10[3:0] == 4'd3)  ? umax3  :
                        (ad1_10[3:0] == 4'd4)  ? umax4  :
                        (ad1_10[3:0] == 4'd5)  ? umax5  :
                        (ad1_10[3:0] == 4'd6)  ? umax6  :
                        (ad1_10[3:0] == 4'd7)  ? umax7  :
                        (ad1_10[3:0] == 4'd8)  ? umax8  :
                        (ad1_10[3:0] == 4'd9)  ? umax9  :
                        (ad1_10[3:0] == 4'd10) ? umax10 :
                        (ad1_10[3:0] == 4'd11) ? umax11 :
                        (ad1_10[3:0] == 4'd12) ? umax12 :
                        (ad1_10[3:0] == 4'd13) ? umax13 :
                        (ad1_10[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[10] = ~ad1_10[4] & (u1_abs <= um_10);
    assign mp_w[87:80] = q_bus[87:80] & {8{mask2[10]}};
    wire signed [4:0]  d3_10  = 5'd10 - k3;
    wire signed [8:0]  mp_10  = {1'b0, mp3[87:80]};
    wire signed [13:0] pr_10  = d3_10 * mp_10;
    assign p_w[142:130] = pr_10[12:0];

    //----- Slot 11 -----
    orb_ori_rowram u_ram11 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd11)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[95:88])
    );
    wire [4:0] d1_11  = 5'd11 - k1;
    wire [4:0] ad1_11 = d1_11[4] ? (~d1_11 + 5'd1) : d1_11;
    wire [3:0] um_11  = (ad1_11[3:0] == 4'd0)  ? umax0  :
                        (ad1_11[3:0] == 4'd1)  ? umax1  :
                        (ad1_11[3:0] == 4'd2)  ? umax2  :
                        (ad1_11[3:0] == 4'd3)  ? umax3  :
                        (ad1_11[3:0] == 4'd4)  ? umax4  :
                        (ad1_11[3:0] == 4'd5)  ? umax5  :
                        (ad1_11[3:0] == 4'd6)  ? umax6  :
                        (ad1_11[3:0] == 4'd7)  ? umax7  :
                        (ad1_11[3:0] == 4'd8)  ? umax8  :
                        (ad1_11[3:0] == 4'd9)  ? umax9  :
                        (ad1_11[3:0] == 4'd10) ? umax10 :
                        (ad1_11[3:0] == 4'd11) ? umax11 :
                        (ad1_11[3:0] == 4'd12) ? umax12 :
                        (ad1_11[3:0] == 4'd13) ? umax13 :
                        (ad1_11[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[11] = ~ad1_11[4] & (u1_abs <= um_11);
    assign mp_w[95:88] = q_bus[95:88] & {8{mask2[11]}};
    wire signed [4:0]  d3_11  = 5'd11 - k3;
    wire signed [8:0]  mp_11  = {1'b0, mp3[95:88]};
    wire signed [13:0] pr_11  = d3_11 * mp_11;
    assign p_w[155:143] = pr_11[12:0];

    //----- Slot 12 -----
    orb_ori_rowram u_ram12 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd12)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[103:96])
    );
    wire [4:0] d1_12  = 5'd12 - k1;
    wire [4:0] ad1_12 = d1_12[4] ? (~d1_12 + 5'd1) : d1_12;
    wire [3:0] um_12  = (ad1_12[3:0] == 4'd0)  ? umax0  :
                        (ad1_12[3:0] == 4'd1)  ? umax1  :
                        (ad1_12[3:0] == 4'd2)  ? umax2  :
                        (ad1_12[3:0] == 4'd3)  ? umax3  :
                        (ad1_12[3:0] == 4'd4)  ? umax4  :
                        (ad1_12[3:0] == 4'd5)  ? umax5  :
                        (ad1_12[3:0] == 4'd6)  ? umax6  :
                        (ad1_12[3:0] == 4'd7)  ? umax7  :
                        (ad1_12[3:0] == 4'd8)  ? umax8  :
                        (ad1_12[3:0] == 4'd9)  ? umax9  :
                        (ad1_12[3:0] == 4'd10) ? umax10 :
                        (ad1_12[3:0] == 4'd11) ? umax11 :
                        (ad1_12[3:0] == 4'd12) ? umax12 :
                        (ad1_12[3:0] == 4'd13) ? umax13 :
                        (ad1_12[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[12] = ~ad1_12[4] & (u1_abs <= um_12);
    assign mp_w[103:96] = q_bus[103:96] & {8{mask2[12]}};
    wire signed [4:0]  d3_12  = 5'd12 - k3;
    wire signed [8:0]  mp_12  = {1'b0, mp3[103:96]};
    wire signed [13:0] pr_12  = d3_12 * mp_12;
    assign p_w[168:156] = pr_12[12:0];

    //----- Slot 13 -----
    orb_ori_rowram u_ram13 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd13)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[111:104])
    );
    wire [4:0] d1_13  = 5'd13 - k1;
    wire [4:0] ad1_13 = d1_13[4] ? (~d1_13 + 5'd1) : d1_13;
    wire [3:0] um_13  = (ad1_13[3:0] == 4'd0)  ? umax0  :
                        (ad1_13[3:0] == 4'd1)  ? umax1  :
                        (ad1_13[3:0] == 4'd2)  ? umax2  :
                        (ad1_13[3:0] == 4'd3)  ? umax3  :
                        (ad1_13[3:0] == 4'd4)  ? umax4  :
                        (ad1_13[3:0] == 4'd5)  ? umax5  :
                        (ad1_13[3:0] == 4'd6)  ? umax6  :
                        (ad1_13[3:0] == 4'd7)  ? umax7  :
                        (ad1_13[3:0] == 4'd8)  ? umax8  :
                        (ad1_13[3:0] == 4'd9)  ? umax9  :
                        (ad1_13[3:0] == 4'd10) ? umax10 :
                        (ad1_13[3:0] == 4'd11) ? umax11 :
                        (ad1_13[3:0] == 4'd12) ? umax12 :
                        (ad1_13[3:0] == 4'd13) ? umax13 :
                        (ad1_13[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[13] = ~ad1_13[4] & (u1_abs <= um_13);
    assign mp_w[111:104] = q_bus[111:104] & {8{mask2[13]}};
    wire signed [4:0]  d3_13  = 5'd13 - k3;
    wire signed [8:0]  mp_13  = {1'b0, mp3[111:104]};
    wire signed [13:0] pr_13  = d3_13 * mp_13;
    assign p_w[181:169] = pr_13[12:0];

    //----- Slot 14 -----
    orb_ori_rowram u_ram14 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd14)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[119:112])
    );
    wire [4:0] d1_14  = 5'd14 - k1;
    wire [4:0] ad1_14 = d1_14[4] ? (~d1_14 + 5'd1) : d1_14;
    wire [3:0] um_14  = (ad1_14[3:0] == 4'd0)  ? umax0  :
                        (ad1_14[3:0] == 4'd1)  ? umax1  :
                        (ad1_14[3:0] == 4'd2)  ? umax2  :
                        (ad1_14[3:0] == 4'd3)  ? umax3  :
                        (ad1_14[3:0] == 4'd4)  ? umax4  :
                        (ad1_14[3:0] == 4'd5)  ? umax5  :
                        (ad1_14[3:0] == 4'd6)  ? umax6  :
                        (ad1_14[3:0] == 4'd7)  ? umax7  :
                        (ad1_14[3:0] == 4'd8)  ? umax8  :
                        (ad1_14[3:0] == 4'd9)  ? umax9  :
                        (ad1_14[3:0] == 4'd10) ? umax10 :
                        (ad1_14[3:0] == 4'd11) ? umax11 :
                        (ad1_14[3:0] == 4'd12) ? umax12 :
                        (ad1_14[3:0] == 4'd13) ? umax13 :
                        (ad1_14[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[14] = ~ad1_14[4] & (u1_abs <= um_14);
    assign mp_w[119:112] = q_bus[119:112] & {8{mask2[14]}};
    wire signed [4:0]  d3_14  = 5'd14 - k3;
    wire signed [8:0]  mp_14  = {1'b0, mp3[119:112]};
    wire signed [13:0] pr_14  = d3_14 * mp_14;
    assign p_w[194:182] = pr_14[12:0];

    //----- Slot 15 -----
    orb_ori_rowram u_ram15 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd15)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[127:120])
    );
    wire [4:0] d1_15  = 5'd15 - k1;
    wire [4:0] ad1_15 = d1_15[4] ? (~d1_15 + 5'd1) : d1_15;
    wire [3:0] um_15  = (ad1_15[3:0] == 4'd0)  ? umax0  :
                        (ad1_15[3:0] == 4'd1)  ? umax1  :
                        (ad1_15[3:0] == 4'd2)  ? umax2  :
                        (ad1_15[3:0] == 4'd3)  ? umax3  :
                        (ad1_15[3:0] == 4'd4)  ? umax4  :
                        (ad1_15[3:0] == 4'd5)  ? umax5  :
                        (ad1_15[3:0] == 4'd6)  ? umax6  :
                        (ad1_15[3:0] == 4'd7)  ? umax7  :
                        (ad1_15[3:0] == 4'd8)  ? umax8  :
                        (ad1_15[3:0] == 4'd9)  ? umax9  :
                        (ad1_15[3:0] == 4'd10) ? umax10 :
                        (ad1_15[3:0] == 4'd11) ? umax11 :
                        (ad1_15[3:0] == 4'd12) ? umax12 :
                        (ad1_15[3:0] == 4'd13) ? umax13 :
                        (ad1_15[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[15] = ~ad1_15[4] & (u1_abs <= um_15);
    assign mp_w[127:120] = q_bus[127:120] & {8{mask2[15]}};
    wire signed [4:0]  d3_15  = 5'd15 - k3;
    wire signed [8:0]  mp_15  = {1'b0, mp3[127:120]};
    wire signed [13:0] pr_15  = d3_15 * mp_15;
    assign p_w[207:195] = pr_15[12:0];

    //----- Slot 16 -----
    orb_ori_rowram u_ram16 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd16)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[135:128])
    );
    wire [4:0] d1_16  = 5'd16 - k1;
    wire [4:0] ad1_16 = d1_16[4] ? (~d1_16 + 5'd1) : d1_16;
    wire [3:0] um_16  = (ad1_16[3:0] == 4'd0)  ? umax0  :
                        (ad1_16[3:0] == 4'd1)  ? umax1  :
                        (ad1_16[3:0] == 4'd2)  ? umax2  :
                        (ad1_16[3:0] == 4'd3)  ? umax3  :
                        (ad1_16[3:0] == 4'd4)  ? umax4  :
                        (ad1_16[3:0] == 4'd5)  ? umax5  :
                        (ad1_16[3:0] == 4'd6)  ? umax6  :
                        (ad1_16[3:0] == 4'd7)  ? umax7  :
                        (ad1_16[3:0] == 4'd8)  ? umax8  :
                        (ad1_16[3:0] == 4'd9)  ? umax9  :
                        (ad1_16[3:0] == 4'd10) ? umax10 :
                        (ad1_16[3:0] == 4'd11) ? umax11 :
                        (ad1_16[3:0] == 4'd12) ? umax12 :
                        (ad1_16[3:0] == 4'd13) ? umax13 :
                        (ad1_16[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[16] = ~ad1_16[4] & (u1_abs <= um_16);
    assign mp_w[135:128] = q_bus[135:128] & {8{mask2[16]}};
    wire signed [4:0]  d3_16  = 5'd16 - k3;
    wire signed [8:0]  mp_16  = {1'b0, mp3[135:128]};
    wire signed [13:0] pr_16  = d3_16 * mp_16;
    assign p_w[220:208] = pr_16[12:0];

    //----- Slot 17 -----
    orb_ori_rowram u_ram17 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd17)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[143:136])
    );
    wire [4:0] d1_17  = 5'd17 - k1;
    wire [4:0] ad1_17 = d1_17[4] ? (~d1_17 + 5'd1) : d1_17;
    wire [3:0] um_17  = (ad1_17[3:0] == 4'd0)  ? umax0  :
                        (ad1_17[3:0] == 4'd1)  ? umax1  :
                        (ad1_17[3:0] == 4'd2)  ? umax2  :
                        (ad1_17[3:0] == 4'd3)  ? umax3  :
                        (ad1_17[3:0] == 4'd4)  ? umax4  :
                        (ad1_17[3:0] == 4'd5)  ? umax5  :
                        (ad1_17[3:0] == 4'd6)  ? umax6  :
                        (ad1_17[3:0] == 4'd7)  ? umax7  :
                        (ad1_17[3:0] == 4'd8)  ? umax8  :
                        (ad1_17[3:0] == 4'd9)  ? umax9  :
                        (ad1_17[3:0] == 4'd10) ? umax10 :
                        (ad1_17[3:0] == 4'd11) ? umax11 :
                        (ad1_17[3:0] == 4'd12) ? umax12 :
                        (ad1_17[3:0] == 4'd13) ? umax13 :
                        (ad1_17[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[17] = ~ad1_17[4] & (u1_abs <= um_17);
    assign mp_w[143:136] = q_bus[143:136] & {8{mask2[17]}};
    wire signed [4:0]  d3_17  = 5'd17 - k3;
    wire signed [8:0]  mp_17  = {1'b0, mp3[143:136]};
    wire signed [13:0] pr_17  = d3_17 * mp_17;
    assign p_w[233:221] = pr_17[12:0];

    //----- Slot 18 -----
    orb_ori_rowram u_ram18 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd18)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[151:144])
    );
    wire [4:0] d1_18  = 5'd18 - k1;
    wire [4:0] ad1_18 = d1_18[4] ? (~d1_18 + 5'd1) : d1_18;
    wire [3:0] um_18  = (ad1_18[3:0] == 4'd0)  ? umax0  :
                        (ad1_18[3:0] == 4'd1)  ? umax1  :
                        (ad1_18[3:0] == 4'd2)  ? umax2  :
                        (ad1_18[3:0] == 4'd3)  ? umax3  :
                        (ad1_18[3:0] == 4'd4)  ? umax4  :
                        (ad1_18[3:0] == 4'd5)  ? umax5  :
                        (ad1_18[3:0] == 4'd6)  ? umax6  :
                        (ad1_18[3:0] == 4'd7)  ? umax7  :
                        (ad1_18[3:0] == 4'd8)  ? umax8  :
                        (ad1_18[3:0] == 4'd9)  ? umax9  :
                        (ad1_18[3:0] == 4'd10) ? umax10 :
                        (ad1_18[3:0] == 4'd11) ? umax11 :
                        (ad1_18[3:0] == 4'd12) ? umax12 :
                        (ad1_18[3:0] == 4'd13) ? umax13 :
                        (ad1_18[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[18] = ~ad1_18[4] & (u1_abs <= um_18);
    assign mp_w[151:144] = q_bus[151:144] & {8{mask2[18]}};
    wire signed [4:0]  d3_18  = 5'd18 - k3;
    wire signed [8:0]  mp_18  = {1'b0, mp3[151:144]};
    wire signed [13:0] pr_18  = d3_18 * mp_18;
    assign p_w[246:234] = pr_18[12:0];

    //----- Slot 19 -----
    orb_ori_rowram u_ram19 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd19)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[159:152])
    );
    wire [4:0] d1_19  = 5'd19 - k1;
    wire [4:0] ad1_19 = d1_19[4] ? (~d1_19 + 5'd1) : d1_19;
    wire [3:0] um_19  = (ad1_19[3:0] == 4'd0)  ? umax0  :
                        (ad1_19[3:0] == 4'd1)  ? umax1  :
                        (ad1_19[3:0] == 4'd2)  ? umax2  :
                        (ad1_19[3:0] == 4'd3)  ? umax3  :
                        (ad1_19[3:0] == 4'd4)  ? umax4  :
                        (ad1_19[3:0] == 4'd5)  ? umax5  :
                        (ad1_19[3:0] == 4'd6)  ? umax6  :
                        (ad1_19[3:0] == 4'd7)  ? umax7  :
                        (ad1_19[3:0] == 4'd8)  ? umax8  :
                        (ad1_19[3:0] == 4'd9)  ? umax9  :
                        (ad1_19[3:0] == 4'd10) ? umax10 :
                        (ad1_19[3:0] == 4'd11) ? umax11 :
                        (ad1_19[3:0] == 4'd12) ? umax12 :
                        (ad1_19[3:0] == 4'd13) ? umax13 :
                        (ad1_19[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[19] = ~ad1_19[4] & (u1_abs <= um_19);
    assign mp_w[159:152] = q_bus[159:152] & {8{mask2[19]}};
    wire signed [4:0]  d3_19  = 5'd19 - k3;
    wire signed [8:0]  mp_19  = {1'b0, mp3[159:152]};
    wire signed [13:0] pr_19  = d3_19 * mp_19;
    assign p_w[259:247] = pr_19[12:0];

    //----- Slot 20 -----
    orb_ori_rowram u_ram20 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd20)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[167:160])
    );
    wire [4:0] d1_20  = 5'd20 - k1;
    wire [4:0] ad1_20 = d1_20[4] ? (~d1_20 + 5'd1) : d1_20;
    wire [3:0] um_20  = (ad1_20[3:0] == 4'd0)  ? umax0  :
                        (ad1_20[3:0] == 4'd1)  ? umax1  :
                        (ad1_20[3:0] == 4'd2)  ? umax2  :
                        (ad1_20[3:0] == 4'd3)  ? umax3  :
                        (ad1_20[3:0] == 4'd4)  ? umax4  :
                        (ad1_20[3:0] == 4'd5)  ? umax5  :
                        (ad1_20[3:0] == 4'd6)  ? umax6  :
                        (ad1_20[3:0] == 4'd7)  ? umax7  :
                        (ad1_20[3:0] == 4'd8)  ? umax8  :
                        (ad1_20[3:0] == 4'd9)  ? umax9  :
                        (ad1_20[3:0] == 4'd10) ? umax10 :
                        (ad1_20[3:0] == 4'd11) ? umax11 :
                        (ad1_20[3:0] == 4'd12) ? umax12 :
                        (ad1_20[3:0] == 4'd13) ? umax13 :
                        (ad1_20[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[20] = ~ad1_20[4] & (u1_abs <= um_20);
    assign mp_w[167:160] = q_bus[167:160] & {8{mask2[20]}};
    wire signed [4:0]  d3_20  = 5'd20 - k3;
    wire signed [8:0]  mp_20  = {1'b0, mp3[167:160]};
    wire signed [13:0] pr_20  = d3_20 * mp_20;
    assign p_w[272:260] = pr_20[12:0];

    //----- Slot 21 -----
    orb_ori_rowram u_ram21 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd21)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[175:168])
    );
    wire [4:0] d1_21  = 5'd21 - k1;
    wire [4:0] ad1_21 = d1_21[4] ? (~d1_21 + 5'd1) : d1_21;
    wire [3:0] um_21  = (ad1_21[3:0] == 4'd0)  ? umax0  :
                        (ad1_21[3:0] == 4'd1)  ? umax1  :
                        (ad1_21[3:0] == 4'd2)  ? umax2  :
                        (ad1_21[3:0] == 4'd3)  ? umax3  :
                        (ad1_21[3:0] == 4'd4)  ? umax4  :
                        (ad1_21[3:0] == 4'd5)  ? umax5  :
                        (ad1_21[3:0] == 4'd6)  ? umax6  :
                        (ad1_21[3:0] == 4'd7)  ? umax7  :
                        (ad1_21[3:0] == 4'd8)  ? umax8  :
                        (ad1_21[3:0] == 4'd9)  ? umax9  :
                        (ad1_21[3:0] == 4'd10) ? umax10 :
                        (ad1_21[3:0] == 4'd11) ? umax11 :
                        (ad1_21[3:0] == 4'd12) ? umax12 :
                        (ad1_21[3:0] == 4'd13) ? umax13 :
                        (ad1_21[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[21] = ~ad1_21[4] & (u1_abs <= um_21);
    assign mp_w[175:168] = q_bus[175:168] & {8{mask2[21]}};
    wire signed [4:0]  d3_21  = 5'd21 - k3;
    wire signed [8:0]  mp_21  = {1'b0, mp3[175:168]};
    wire signed [13:0] pr_21  = d3_21 * mp_21;
    assign p_w[285:273] = pr_21[12:0];

    //----- Slot 22 -----
    orb_ori_rowram u_ram22 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd22)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[183:176])
    );
    wire [4:0] d1_22  = 5'd22 - k1;
    wire [4:0] ad1_22 = d1_22[4] ? (~d1_22 + 5'd1) : d1_22;
    wire [3:0] um_22  = (ad1_22[3:0] == 4'd0)  ? umax0  :
                        (ad1_22[3:0] == 4'd1)  ? umax1  :
                        (ad1_22[3:0] == 4'd2)  ? umax2  :
                        (ad1_22[3:0] == 4'd3)  ? umax3  :
                        (ad1_22[3:0] == 4'd4)  ? umax4  :
                        (ad1_22[3:0] == 4'd5)  ? umax5  :
                        (ad1_22[3:0] == 4'd6)  ? umax6  :
                        (ad1_22[3:0] == 4'd7)  ? umax7  :
                        (ad1_22[3:0] == 4'd8)  ? umax8  :
                        (ad1_22[3:0] == 4'd9)  ? umax9  :
                        (ad1_22[3:0] == 4'd10) ? umax10 :
                        (ad1_22[3:0] == 4'd11) ? umax11 :
                        (ad1_22[3:0] == 4'd12) ? umax12 :
                        (ad1_22[3:0] == 4'd13) ? umax13 :
                        (ad1_22[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[22] = ~ad1_22[4] & (u1_abs <= um_22);
    assign mp_w[183:176] = q_bus[183:176] & {8{mask2[22]}};
    wire signed [4:0]  d3_22  = 5'd22 - k3;
    wire signed [8:0]  mp_22  = {1'b0, mp3[183:176]};
    wire signed [13:0] pr_22  = d3_22 * mp_22;
    assign p_w[298:286] = pr_22[12:0];

    //----- Slot 23 -----
    orb_ori_rowram u_ram23 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd23)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[191:184])
    );
    wire [4:0] d1_23  = 5'd23 - k1;
    wire [4:0] ad1_23 = d1_23[4] ? (~d1_23 + 5'd1) : d1_23;
    wire [3:0] um_23  = (ad1_23[3:0] == 4'd0)  ? umax0  :
                        (ad1_23[3:0] == 4'd1)  ? umax1  :
                        (ad1_23[3:0] == 4'd2)  ? umax2  :
                        (ad1_23[3:0] == 4'd3)  ? umax3  :
                        (ad1_23[3:0] == 4'd4)  ? umax4  :
                        (ad1_23[3:0] == 4'd5)  ? umax5  :
                        (ad1_23[3:0] == 4'd6)  ? umax6  :
                        (ad1_23[3:0] == 4'd7)  ? umax7  :
                        (ad1_23[3:0] == 4'd8)  ? umax8  :
                        (ad1_23[3:0] == 4'd9)  ? umax9  :
                        (ad1_23[3:0] == 4'd10) ? umax10 :
                        (ad1_23[3:0] == 4'd11) ? umax11 :
                        (ad1_23[3:0] == 4'd12) ? umax12 :
                        (ad1_23[3:0] == 4'd13) ? umax13 :
                        (ad1_23[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[23] = ~ad1_23[4] & (u1_abs <= um_23);
    assign mp_w[191:184] = q_bus[191:184] & {8{mask2[23]}};
    wire signed [4:0]  d3_23  = 5'd23 - k3;
    wire signed [8:0]  mp_23  = {1'b0, mp3[191:184]};
    wire signed [13:0] pr_23  = d3_23 * mp_23;
    assign p_w[311:299] = pr_23[12:0];

    //----- Slot 24 -----
    orb_ori_rowram u_ram24 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd24)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[199:192])
    );
    wire [4:0] d1_24  = 5'd24 - k1;
    wire [4:0] ad1_24 = d1_24[4] ? (~d1_24 + 5'd1) : d1_24;
    wire [3:0] um_24  = (ad1_24[3:0] == 4'd0)  ? umax0  :
                        (ad1_24[3:0] == 4'd1)  ? umax1  :
                        (ad1_24[3:0] == 4'd2)  ? umax2  :
                        (ad1_24[3:0] == 4'd3)  ? umax3  :
                        (ad1_24[3:0] == 4'd4)  ? umax4  :
                        (ad1_24[3:0] == 4'd5)  ? umax5  :
                        (ad1_24[3:0] == 4'd6)  ? umax6  :
                        (ad1_24[3:0] == 4'd7)  ? umax7  :
                        (ad1_24[3:0] == 4'd8)  ? umax8  :
                        (ad1_24[3:0] == 4'd9)  ? umax9  :
                        (ad1_24[3:0] == 4'd10) ? umax10 :
                        (ad1_24[3:0] == 4'd11) ? umax11 :
                        (ad1_24[3:0] == 4'd12) ? umax12 :
                        (ad1_24[3:0] == 4'd13) ? umax13 :
                        (ad1_24[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[24] = ~ad1_24[4] & (u1_abs <= um_24);
    assign mp_w[199:192] = q_bus[199:192] & {8{mask2[24]}};
    wire signed [4:0]  d3_24  = 5'd24 - k3;
    wire signed [8:0]  mp_24  = {1'b0, mp3[199:192]};
    wire signed [13:0] pr_24  = d3_24 * mp_24;
    assign p_w[324:312] = pr_24[12:0];

    //----- Slot 25 -----
    orb_ori_rowram u_ram25 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd25)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[207:200])
    );
    wire [4:0] d1_25  = 5'd25 - k1;
    wire [4:0] ad1_25 = d1_25[4] ? (~d1_25 + 5'd1) : d1_25;
    wire [3:0] um_25  = (ad1_25[3:0] == 4'd0)  ? umax0  :
                        (ad1_25[3:0] == 4'd1)  ? umax1  :
                        (ad1_25[3:0] == 4'd2)  ? umax2  :
                        (ad1_25[3:0] == 4'd3)  ? umax3  :
                        (ad1_25[3:0] == 4'd4)  ? umax4  :
                        (ad1_25[3:0] == 4'd5)  ? umax5  :
                        (ad1_25[3:0] == 4'd6)  ? umax6  :
                        (ad1_25[3:0] == 4'd7)  ? umax7  :
                        (ad1_25[3:0] == 4'd8)  ? umax8  :
                        (ad1_25[3:0] == 4'd9)  ? umax9  :
                        (ad1_25[3:0] == 4'd10) ? umax10 :
                        (ad1_25[3:0] == 4'd11) ? umax11 :
                        (ad1_25[3:0] == 4'd12) ? umax12 :
                        (ad1_25[3:0] == 4'd13) ? umax13 :
                        (ad1_25[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[25] = ~ad1_25[4] & (u1_abs <= um_25);
    assign mp_w[207:200] = q_bus[207:200] & {8{mask2[25]}};
    wire signed [4:0]  d3_25  = 5'd25 - k3;
    wire signed [8:0]  mp_25  = {1'b0, mp3[207:200]};
    wire signed [13:0] pr_25  = d3_25 * mp_25;
    assign p_w[337:325] = pr_25[12:0];

    //----- Slot 26 -----
    orb_ori_rowram u_ram26 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd26)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[215:208])
    );
    wire [4:0] d1_26  = 5'd26 - k1;
    wire [4:0] ad1_26 = d1_26[4] ? (~d1_26 + 5'd1) : d1_26;
    wire [3:0] um_26  = (ad1_26[3:0] == 4'd0)  ? umax0  :
                        (ad1_26[3:0] == 4'd1)  ? umax1  :
                        (ad1_26[3:0] == 4'd2)  ? umax2  :
                        (ad1_26[3:0] == 4'd3)  ? umax3  :
                        (ad1_26[3:0] == 4'd4)  ? umax4  :
                        (ad1_26[3:0] == 4'd5)  ? umax5  :
                        (ad1_26[3:0] == 4'd6)  ? umax6  :
                        (ad1_26[3:0] == 4'd7)  ? umax7  :
                        (ad1_26[3:0] == 4'd8)  ? umax8  :
                        (ad1_26[3:0] == 4'd9)  ? umax9  :
                        (ad1_26[3:0] == 4'd10) ? umax10 :
                        (ad1_26[3:0] == 4'd11) ? umax11 :
                        (ad1_26[3:0] == 4'd12) ? umax12 :
                        (ad1_26[3:0] == 4'd13) ? umax13 :
                        (ad1_26[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[26] = ~ad1_26[4] & (u1_abs <= um_26);
    assign mp_w[215:208] = q_bus[215:208] & {8{mask2[26]}};
    wire signed [4:0]  d3_26  = 5'd26 - k3;
    wire signed [8:0]  mp_26  = {1'b0, mp3[215:208]};
    wire signed [13:0] pr_26  = d3_26 * mp_26;
    assign p_w[350:338] = pr_26[12:0];

    //----- Slot 27 -----
    orb_ori_rowram u_ram27 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd27)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[223:216])
    );
    wire [4:0] d1_27  = 5'd27 - k1;
    wire [4:0] ad1_27 = d1_27[4] ? (~d1_27 + 5'd1) : d1_27;
    wire [3:0] um_27  = (ad1_27[3:0] == 4'd0)  ? umax0  :
                        (ad1_27[3:0] == 4'd1)  ? umax1  :
                        (ad1_27[3:0] == 4'd2)  ? umax2  :
                        (ad1_27[3:0] == 4'd3)  ? umax3  :
                        (ad1_27[3:0] == 4'd4)  ? umax4  :
                        (ad1_27[3:0] == 4'd5)  ? umax5  :
                        (ad1_27[3:0] == 4'd6)  ? umax6  :
                        (ad1_27[3:0] == 4'd7)  ? umax7  :
                        (ad1_27[3:0] == 4'd8)  ? umax8  :
                        (ad1_27[3:0] == 4'd9)  ? umax9  :
                        (ad1_27[3:0] == 4'd10) ? umax10 :
                        (ad1_27[3:0] == 4'd11) ? umax11 :
                        (ad1_27[3:0] == 4'd12) ? umax12 :
                        (ad1_27[3:0] == 4'd13) ? umax13 :
                        (ad1_27[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[27] = ~ad1_27[4] & (u1_abs <= um_27);
    assign mp_w[223:216] = q_bus[223:216] & {8{mask2[27]}};
    wire signed [4:0]  d3_27  = 5'd27 - k3;
    wire signed [8:0]  mp_27  = {1'b0, mp3[223:216]};
    wire signed [13:0] pr_27  = d3_27 * mp_27;
    assign p_w[363:351] = pr_27[12:0];

    //----- Slot 28 -----
    orb_ori_rowram u_ram28 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd28)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[231:224])
    );
    wire [4:0] d1_28  = 5'd28 - k1;
    wire [4:0] ad1_28 = d1_28[4] ? (~d1_28 + 5'd1) : d1_28;
    wire [3:0] um_28  = (ad1_28[3:0] == 4'd0)  ? umax0  :
                        (ad1_28[3:0] == 4'd1)  ? umax1  :
                        (ad1_28[3:0] == 4'd2)  ? umax2  :
                        (ad1_28[3:0] == 4'd3)  ? umax3  :
                        (ad1_28[3:0] == 4'd4)  ? umax4  :
                        (ad1_28[3:0] == 4'd5)  ? umax5  :
                        (ad1_28[3:0] == 4'd6)  ? umax6  :
                        (ad1_28[3:0] == 4'd7)  ? umax7  :
                        (ad1_28[3:0] == 4'd8)  ? umax8  :
                        (ad1_28[3:0] == 4'd9)  ? umax9  :
                        (ad1_28[3:0] == 4'd10) ? umax10 :
                        (ad1_28[3:0] == 4'd11) ? umax11 :
                        (ad1_28[3:0] == 4'd12) ? umax12 :
                        (ad1_28[3:0] == 4'd13) ? umax13 :
                        (ad1_28[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[28] = ~ad1_28[4] & (u1_abs <= um_28);
    assign mp_w[231:224] = q_bus[231:224] & {8{mask2[28]}};
    wire signed [4:0]  d3_28  = 5'd28 - k3;
    wire signed [8:0]  mp_28  = {1'b0, mp3[231:224]};
    wire signed [13:0] pr_28  = d3_28 * mp_28;
    assign p_w[376:364] = pr_28[12:0];

    //----- Slot 29 -----
    orb_ori_rowram u_ram29 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd29)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[239:232])
    );
    wire [4:0] d1_29  = 5'd29 - k1;
    wire [4:0] ad1_29 = d1_29[4] ? (~d1_29 + 5'd1) : d1_29;
    wire [3:0] um_29  = (ad1_29[3:0] == 4'd0)  ? umax0  :
                        (ad1_29[3:0] == 4'd1)  ? umax1  :
                        (ad1_29[3:0] == 4'd2)  ? umax2  :
                        (ad1_29[3:0] == 4'd3)  ? umax3  :
                        (ad1_29[3:0] == 4'd4)  ? umax4  :
                        (ad1_29[3:0] == 4'd5)  ? umax5  :
                        (ad1_29[3:0] == 4'd6)  ? umax6  :
                        (ad1_29[3:0] == 4'd7)  ? umax7  :
                        (ad1_29[3:0] == 4'd8)  ? umax8  :
                        (ad1_29[3:0] == 4'd9)  ? umax9  :
                        (ad1_29[3:0] == 4'd10) ? umax10 :
                        (ad1_29[3:0] == 4'd11) ? umax11 :
                        (ad1_29[3:0] == 4'd12) ? umax12 :
                        (ad1_29[3:0] == 4'd13) ? umax13 :
                        (ad1_29[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[29] = ~ad1_29[4] & (u1_abs <= um_29);
    assign mp_w[239:232] = q_bus[239:232] & {8{mask2[29]}};
    wire signed [4:0]  d3_29  = 5'd29 - k3;
    wire signed [8:0]  mp_29  = {1'b0, mp3[239:232]};
    wire signed [13:0] pr_29  = d3_29 * mp_29;
    assign p_w[389:377] = pr_29[12:0];

    //----- Slot 30 -----
    orb_ori_rowram u_ram30 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd30)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[247:240])
    );
    wire [4:0] d1_30  = 5'd30 - k1;
    wire [4:0] ad1_30 = d1_30[4] ? (~d1_30 + 5'd1) : d1_30;
    wire [3:0] um_30  = (ad1_30[3:0] == 4'd0)  ? umax0  :
                        (ad1_30[3:0] == 4'd1)  ? umax1  :
                        (ad1_30[3:0] == 4'd2)  ? umax2  :
                        (ad1_30[3:0] == 4'd3)  ? umax3  :
                        (ad1_30[3:0] == 4'd4)  ? umax4  :
                        (ad1_30[3:0] == 4'd5)  ? umax5  :
                        (ad1_30[3:0] == 4'd6)  ? umax6  :
                        (ad1_30[3:0] == 4'd7)  ? umax7  :
                        (ad1_30[3:0] == 4'd8)  ? umax8  :
                        (ad1_30[3:0] == 4'd9)  ? umax9  :
                        (ad1_30[3:0] == 4'd10) ? umax10 :
                        (ad1_30[3:0] == 4'd11) ? umax11 :
                        (ad1_30[3:0] == 4'd12) ? umax12 :
                        (ad1_30[3:0] == 4'd13) ? umax13 :
                        (ad1_30[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[30] = ~ad1_30[4] & (u1_abs <= um_30);
    assign mp_w[247:240] = q_bus[247:240] & {8{mask2[30]}};
    wire signed [4:0]  d3_30  = 5'd30 - k3;
    wire signed [8:0]  mp_30  = {1'b0, mp3[247:240]};
    wire signed [13:0] pr_30  = d3_30 * mp_30;
    assign p_w[402:390] = pr_30[12:0];

    //----- Slot 31 -----
    orb_ori_rowram u_ram31 (
        .clk   (clk),
        .we    (pix_valid & (pix_y[4:0] == 5'd31)),
        .waddr (pix_x),
        .wdata (pix_data),
        .raddr (rd_addr),
        .rdata (q_bus[255:248])
    );
    wire [4:0] d1_31  = 5'd31 - k1;
    wire [4:0] ad1_31 = d1_31[4] ? (~d1_31 + 5'd1) : d1_31;
    wire [3:0] um_31  = (ad1_31[3:0] == 4'd0)  ? umax0  :
                        (ad1_31[3:0] == 4'd1)  ? umax1  :
                        (ad1_31[3:0] == 4'd2)  ? umax2  :
                        (ad1_31[3:0] == 4'd3)  ? umax3  :
                        (ad1_31[3:0] == 4'd4)  ? umax4  :
                        (ad1_31[3:0] == 4'd5)  ? umax5  :
                        (ad1_31[3:0] == 4'd6)  ? umax6  :
                        (ad1_31[3:0] == 4'd7)  ? umax7  :
                        (ad1_31[3:0] == 4'd8)  ? umax8  :
                        (ad1_31[3:0] == 4'd9)  ? umax9  :
                        (ad1_31[3:0] == 4'd10) ? umax10 :
                        (ad1_31[3:0] == 4'd11) ? umax11 :
                        (ad1_31[3:0] == 4'd12) ? umax12 :
                        (ad1_31[3:0] == 4'd13) ? umax13 :
                        (ad1_31[3:0] == 4'd14) ? umax14 : umax15;
    assign mask_w[31] = ~ad1_31[4] & (u1_abs <= um_31);
    assign mp_w[255:248] = q_bus[255:248] & {8{mask2[31]}};
    wire signed [4:0]  d3_31  = 5'd31 - k3;
    wire signed [8:0]  mp_31  = {1'b0, mp3[255:248]};
    wire signed [13:0] pr_31  = d3_31 * mp_31;
    assign p_w[415:403] = pr_31[12:0];

    //--------------------------------------------------------------------------
    // Adder trees level 1 (groups of 4) - fully unrolled
    //--------------------------------------------------------------------------
    // group 0
    assign cs_w[9:0]   = mp3[7:0]   + mp3[15:8]  + mp3[23:16] + mp3[31:24];
    wire signed [12:0] a0_0 = p4[12:0];
    wire signed [12:0] a1_0 = p4[25:13];
    wire signed [12:0] a2_0 = p4[38:26];
    wire signed [12:0] a3_0 = p4[51:39];
    wire signed [14:0] sm_0 = a0_0 + a1_0 + a2_0 + a3_0;
    assign ws_w[14:0] = sm_0;

    // group 1
    assign cs_w[19:10] = mp3[39:32] + mp3[47:40] + mp3[55:48] + mp3[63:56];
    wire signed [12:0] a0_1 = p4[64:52];
    wire signed [12:0] a1_1 = p4[77:65];
    wire signed [12:0] a2_1 = p4[90:78];
    wire signed [12:0] a3_1 = p4[103:91];
    wire signed [14:0] sm_1 = a0_1 + a1_1 + a2_1 + a3_1;
    assign ws_w[29:15] = sm_1;

    // group 2
    assign cs_w[29:20] = mp3[71:64] + mp3[79:72] + mp3[87:80] + mp3[95:88];
    wire signed [12:0] a0_2 = p4[116:104];
    wire signed [12:0] a1_2 = p4[129:117];
    wire signed [12:0] a2_2 = p4[142:130];
    wire signed [12:0] a3_2 = p4[155:143];
    wire signed [14:0] sm_2 = a0_2 + a1_2 + a2_2 + a3_2;
    assign ws_w[44:30] = sm_2;

    // group 3
    assign cs_w[39:30] = mp3[103:96] + mp3[111:104] + mp3[119:112] + mp3[127:120];
    wire signed [12:0] a0_3 = p4[168:156];
    wire signed [12:0] a1_3 = p4[181:169];
    wire signed [12:0] a2_3 = p4[194:182];
    wire signed [12:0] a3_3 = p4[207:195];
    wire signed [14:0] sm_3 = a0_3 + a1_3 + a2_3 + a3_3;
    assign ws_w[59:45] = sm_3;

    // group 4
    assign cs_w[49:40] = mp3[135:128] + mp3[143:136] + mp3[151:144] + mp3[159:152];
    wire signed [12:0] a0_4 = p4[220:208];
    wire signed [12:0] a1_4 = p4[233:221];
    wire signed [12:0] a2_4 = p4[246:234];
    wire signed [12:0] a3_4 = p4[259:247];
    wire signed [14:0] sm_4 = a0_4 + a1_4 + a2_4 + a3_4;
    assign ws_w[74:60] = sm_4;

    // group 5
    assign cs_w[59:50] = mp3[167:160] + mp3[175:168] + mp3[183:176] + mp3[191:184];
    wire signed [12:0] a0_5 = p4[272:260];
    wire signed [12:0] a1_5 = p4[285:273];
    wire signed [12:0] a2_5 = p4[298:286];
    wire signed [12:0] a3_5 = p4[311:299];
    wire signed [14:0] sm_5 = a0_5 + a1_5 + a2_5 + a3_5;
    assign ws_w[89:75] = sm_5;

    // group 6
    assign cs_w[69:60] = mp3[199:192] + mp3[207:200] + mp3[215:208] + mp3[223:216];
    wire signed [12:0] a0_6 = p4[324:312];
    wire signed [12:0] a1_6 = p4[337:325];
    wire signed [12:0] a2_6 = p4[350:338];
    wire signed [12:0] a3_6 = p4[363:351];
    wire signed [14:0] sm_6 = a0_6 + a1_6 + a2_6 + a3_6;
    assign ws_w[104:90] = sm_6;

    // group 7
    assign cs_w[79:70] = mp3[231:224] + mp3[239:232] + mp3[247:240] + mp3[255:248];
    wire signed [12:0] a0_7 = p4[376:364];
    wire signed [12:0] a1_7 = p4[389:377];
    wire signed [12:0] a2_7 = p4[402:390];
    wire signed [12:0] a3_7 = p4[415:403];
    wire signed [14:0] sm_7 = a0_7 + a1_7 + a2_7 + a3_7;
    assign ws_w[119:105] = sm_7;

    //--------------------------------------------------------------------------
    // Level 2 / 3 adder trees
    //--------------------------------------------------------------------------
    wire [11:0] c2h0 = cs4[9:0]   + cs4[19:10] + cs4[29:20] + cs4[39:30];
    wire [11:0] c2h1 = cs4[49:40] + cs4[59:50] + cs4[69:60] + cs4[79:70];

    wire signed [14:0] w0 = ws5[14:0];
    wire signed [14:0] w1 = ws5[29:15];
    wire signed [14:0] w2 = ws5[44:30];
    wire signed [14:0] w3 = ws5[59:45];
    wire signed [14:0] w4 = ws5[74:60];
    wire signed [14:0] w5 = ws5[89:75];
    wire signed [14:0] w6 = ws5[104:90];
    wire signed [14:0] w7 = ws5[119:105];
    wire signed [16:0] wa = w0 + w1 + w2 + w3;
    wire signed [16:0] wb = w4 + w5 + w6 + w7;
    wire        [12:0] ct = cs5[11:0] + cs5[23:12];

    reg signed [16:0] wsa6, wsb6;
    reg        [12:0] ct6;
    reg signed [16:0] wsum7;
    reg signed [18:0] pm7;
    reg signed [21:0] m01_acc, m10_acc;

    wire signed [4:0]  u6s  = u6;
    wire signed [13:0] ct6s = {1'b0, ct6};

    // data pipeline (no reset needed)
    always @(posedge clk) begin
        // 1 -> 2
        mask2 <= mask_w;
        f2 <= f1;  u2 <= u1;  k2 <= k1;
        // 2 -> 3
        mp3 <= mp_w;
        f3 <= f2;  u3 <= u2;  k3 <= k2;
        // 3 -> 4
        p4  <= p_w;
        cs4 <= cs_w;
        f4 <= f3;  u4 <= u3;
        // 4 -> 5
        ws5 <= ws_w;
        cs5 <= {c2h1, c2h0};
        f5 <= f4;  u5 <= u4;
        // 5 -> 6
        wsa6 <= wa;
        wsb6 <= wb;
        ct6  <= ct;
        f6 <= f5;  u6 <= u5;
        // 6 -> 7
        wsum7 <= wsa6 + wsb6;
        pm7   <= u6s * ct6s;
        f7 <= f6;
        // 7 -> 8 : accumulate over the 31 columns
        m01_acc <= (f7 ? 22'sd0 : m01_acc) + wsum7;
        m10_acc <= (f7 ? 22'sd0 : m10_acc) + pm7;
    end

    // "last column" valid bit pipeline (reset)
    always @(posedge clk) begin
        if (rst) begin
            l2 <= 1'b0; l3 <= 1'b0; l4 <= 1'b0; l5 <= 1'b0;
            l6 <= 1'b0; l7 <= 1'b0; res_v <= 1'b0;
        end else begin
            l2 <= l1; l3 <= l2; l4 <= l3; l5 <= l4;
            l6 <= l5; l7 <= l6; res_v <= l7;
        end
    end

    //--------------------------------------------------------------------------
    // Moments -> sector, output registers
    //--------------------------------------------------------------------------
    wire       sec_done;
    wire [4:0] sec_val;

    orb_ori_sector u_sector (
        .clk    (clk),
        .rst    (rst),
        .start  (res_v),
        .m10    (m10_acc),
        .m01    (m01_acc),
        .done   (sec_done),
        .sector (sec_val)
    );

    reg [9:0]         s_x;
    reg [8:0]         s_y;
    reg [7:0]         s_score;
    reg signed [21:0] s_m10, s_m01;

    always @(posedge clk) begin
        if (res_v) begin
            s_x     <= r_x;
            s_y     <= r_y;
            s_score <= r_score;
            s_m10   <= m10_acc;
            s_m01   <= m01_acc;
        end

        if (rst)
            ori_valid <= 1'b0;
        else
            ori_valid <= sec_done;

        if (sec_done) begin
            ori_x      <= s_x;
            ori_y      <= s_y;
            ori_score  <= s_score;
            ori_sector <= sec_val;
            ori_m10    <= s_m10;
            ori_m01    <= s_m01;
        end
    end

endmodule
