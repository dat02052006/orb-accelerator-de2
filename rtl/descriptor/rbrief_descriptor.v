// ============================================================================
// rbrief_descriptor
//   Tao descriptor rBRIEF 256 bit tu smoothed buffer 37x37 va angle_bin.
//
//   - Smoothed buffer: 2 ban sao RAM cung noi dung (ghi dong thoi tu rbrief_gauss_window),
//     moi ban doc 1 pixel/chu ky -> doc dong thoi pixel p va q cua mot cap.
//   - ROM pattern da xoay san (rbrief_pattern.mif, sinh boi gen_rbrief_rom.py):
//       dia chi = {angle_bin[4:0], pair_idx[7:0]}, du lieu 22 bit = {addr_p[10:0], addr_q[10:0]}
//       addr = (18 + y) * 37 + (18 + x) la dia chi trong smoothed buffer
//   - bit cap k = I(p) < I(q); desc[k] = bit cap k (byte i, bit j LSB truoc = cap 8*i + j)
//
//   Moi cap 1 chu ky, tong khoang 260 chu ky tu start den done.
//   Yeu cau: rbrief_gauss_window.v (module rbrief_sdp_ram) cung nam trong project.
//   Smoothed buffer khong duoc ghi trong luc blok nay dang chay (ctrl tuan tu hoa).
// ============================================================================
module rbrief_descriptor (
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
    // ROM pattern da xoay san (doc dong bo, tre 1 chu ky)
    // ======================================================
    reg [21:0] rom [0:7679] /* synthesis ram_init_file = "rbrief_pattern.mif" */;
    reg [21:0] rom_q;

    wire [12:0] rom_addr = {bin_r, pair_cnt};

    always @(posedge clk) begin
        rom_q <= rom[rom_addr];
    end

    wire [10:0] addr_p = rom_q[21:11];
    wire [10:0] addr_q = rom_q[10:0];

    // ======================================================
    // Smoothed buffer: 2 ban sao, moi ban 1 cong doc
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
    // Pipeline: A (phat dia chi ROM) -> B (rom_q -> dia chi RAM) -> C (pix_p, pix_q hop le)
    // ======================================================
    reg rv1;
    reg rv2;

    wire cmp_bit = (pix_p < pix_q);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rv1    <= 1'b0;
            rv2    <= 1'b0;
            desc_r <= 256'd0;
        end else begin
            rv1 <= (state == R_RUN);
            rv2 <= rv1;
            if (rv2) desc_r <= {cmp_bit, desc_r[255:1]};     // cap dau tien ket thuc o desc[0]
        end
    end

    wire pipe_busy = rv1 | rv2;

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
