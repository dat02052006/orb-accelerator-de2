module centroid_patch_reader (
    input  wire        clk,
    input  wire        rst_n,

    // ---- dieu khien ----
    input  wire        start,           // xung bat dau (kx, ky phai hop le tai chu ky nay)
    input  wire [9:0]  kx,              // toa do x cua keypoint (15..624)
    input  wire [8:0]  ky,              // toa do y cua keypoint (15..464)
    output wire        busy,
    output wire        done,            // xung 1 chu ky khi nap xong patch
    output wire        error,           // xung 1 chu ky neu keypoint nam ngoai vung hop le:
                                        //   kx < 15, kx > 624, ky < 15 hoac ky > 464

    // ---- giao tiep sram_controller (giong image_reader) ----
    output wire [17:0] sram_addr,       // dia chi word
    output wire        sram_re,
    input  wire [15:0] sram_rd_data,
    input  wire        sram_valid,

    // ---- giao tiep patch buffer / bo tich luy ----
    output wire        pb_we,
    output wire [4:0]  pb_row,          // hang trong patch (0..30)
    output wire [3:0]  pb_word,         // word trong hang (0..15)
    output wire [8:0]  pb_addr,         // = {pb_row, pb_word} = row*16 + word (0..495)
    output wire [15:0] pb_wdata,        // [7:0] = pixel cot chan, [15:8] = pixel cot le (theo toa do anh)
    output wire        patch_x0_odd     // 1: pixel dau patch (cot x0 = kx-15) la byte le cua word dau
                                        //    pixel cot c cua patch nam o byte (c + patch_x0_odd): chan->[7:0], le->[15:8]
);

    localparam [3:0] LAST_WORD = 4'd15;            // moi hang doc 16 word = 32 pixel (can 31)
    localparam [4:0] LAST_ROW  = 5'd30;            // 31 hang
    localparam [9:0] KX_MIN    = 10'd15;
    localparam [9:0] KX_MAX    = 10'd624;          // 639 - 15
    localparam [8:0] KY_MIN    = 9'd15;
    localparam [8:0] KY_MAX    = 9'd464;           // 479 - 15

    // ---------------- state encoding ----------------
    localparam [2:0] S_IDLE  = 3'd0,
                     S_INIT  = 3'd1,
                     S_READ  = 3'd2,
                     S_STORE = 3'd3,
                     S_DONE  = 3'd4,
                     S_ERROR = 3'd5;

    reg [2:0] state, next_state;

    // ---------------- datapath registers ----------------
    reg [17:0] row_base;        // = (y0 + row_cnt)*320 + (x0 >> 1)
    reg        x0_odd_r;        // x0[0], chot luc start
    reg [3:0]  word_cnt;        // 0..15
    reg [4:0]  row_cnt;         // 0..30
    reg [15:0] data_r;          // word doc duoc tu SRAM

    wire last_word = (word_cnt == LAST_WORD);
    wire last_row  = (row_cnt  == LAST_ROW);

    // kiem tra tham so tai thoi diem start
    wire kx_ok     = (kx >= KX_MIN) && (kx <= KX_MAX);
    wire ky_ok     = (ky >= KY_MIN) && (ky <= KY_MAX);
    wire params_ok = kx_ok && ky_ok;

    // goc tren trai cua patch
    wire [9:0] x0 = kx - 10'd15;                    // 0..609
    wire [8:0] y0 = ky - 9'd15;                     // 0..449

    // dia chi word dau tien = y0*320 + (x0>>1), y0*320 = y0*256 + y0*64
    wire [17:0] start_base = ({9'd0, y0} << 8) + ({9'd0, y0} << 6) + {9'd0, x0[9:1]};

    // ======================================================
    // FSM phan 1: next-state transition logic
    // ======================================================
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:  begin
                if (start) begin
                    if (params_ok) next_state = S_INIT;
                    else           next_state = S_ERROR;
                end
            end
            S_INIT:  next_state = S_READ;
            S_READ:  begin
                if (sram_valid) next_state = S_STORE;
            end
            S_STORE: begin
                if (last_word && last_row) next_state = S_DONE;
                else                       next_state = S_READ;
            end
            S_DONE:  next_state = S_IDLE;
            S_ERROR: next_state = S_IDLE;
            default: next_state = S_IDLE;
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
    // Datapath: dem word/hang, tinh dia chi, chot du lieu
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_base <= 18'd0;
            x0_odd_r <= 1'b0;
            word_cnt <= 4'd0;
            row_cnt  <= 5'd0;
            data_r   <= 16'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        row_base <= start_base;
                        x0_odd_r <= x0[0];
                    end
                end
                S_INIT: begin
                    word_cnt <= 4'd0;
                    row_cnt  <= 5'd0;
                end
                S_READ: begin
                    if (sram_valid) data_r <= sram_rd_data;
                end
                S_STORE: begin
                    if (last_word) begin
                        word_cnt <= 4'd0;
                        row_cnt  <= row_cnt + 5'd1;
                        row_base <= row_base + 18'd320;
                    end else begin
                        word_cnt <= word_cnt + 4'd1;
                    end
                end
                default: ;
            endcase
        end
    end

    // ======================================================
    // FSM phan 3: combinational output logic
    // ======================================================
    assign sram_re       = (state == S_READ);
    assign sram_addr     = row_base + {14'd0, word_cnt};

    assign pb_we         = (state == S_STORE);
    assign pb_row        = row_cnt;
    assign pb_word       = word_cnt;
    assign pb_addr       = {row_cnt, word_cnt};
    assign pb_wdata      = data_r;
    assign patch_x0_odd  = x0_odd_r;

    assign busy          = (state != S_IDLE);
    assign done          = (state == S_DONE);
    assign error         = (state == S_ERROR);

endmodule
