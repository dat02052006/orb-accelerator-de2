//==============================================================================
// candidate_buffer.v
// Quản lý 3 bank M4K xoay vòng.
// - Score ghi vào 1 bank
// - NMS đọc song song cả 3 bank (3 port đọc độc lập)
//==============================================================================

module candidate_buffer (
    input  wire         clk,
    input  wire         rst_n,

    // ---- Interface với Score module ----
    input  wire         score_valid,
    input  wire [9:0]   score_x,
    input  wire [7:0]   score_value,
    input  wire         row_start,
    input  wire         row_done,

    // ---- Interface với NMS module ----
    // NMS báo đã xử lý xong một bank (center)
    input  wire         nms_done,
    input  wire [1:0]   nms_done_bank,

    // 3 read ports độc lập (NMS có thể đọc 3 bank cùng lúc)
    input  wire         rd_en_0,
    input  wire [9:0]   rd_addr_0,
    output wire [17:0]  rd_data_0,      // {score[7:0], x[9:0]}

    input  wire         rd_en_1,
    input  wire [9:0]   rd_addr_1,
    output wire [17:0]  rd_data_1,

    input  wire         rd_en_2,
    input  wire [9:0]   rd_addr_2,
    output wire [17:0]  rd_data_2,

    // Thông tin trạng thái từng bank
    output wire         bank_ready,
    output wire [9:0]   bank_count_0,
    output wire [9:0]   bank_count_1,
    output wire [9:0]   bank_count_2,
    output wire [9:0]   bank_row_0,
    output wire [9:0]   bank_row_1,
    output wire [9:0]   bank_row_2,
    output wire [2:0]   bank_state_0,   // 0=EMPTY, 1=WRITING, 2=FULL, 3=READING
    output wire [2:0]   bank_state_1,
    output wire [2:0]   bank_state_2,

    // Debug
    output wire [1:0]   dbg_wr_bank
);

    //------------------------------------------------------------------
    // Controller
    //------------------------------------------------------------------
    wire [1:0]  wr_bank_sel;
    wire        wr_enable;
    wire [9:0]  current_row_id;

    candidate_buffer_controller u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .row_start      (row_start),
        .row_done       (row_done),
        .score_valid    (score_valid),
        .nms_done       (nms_done),
        .nms_done_bank  (nms_done_bank),
        .wr_bank_sel    (wr_bank_sel),
        .wr_enable      (wr_enable),
        .bank_ready     (bank_ready),
        .bank_count_0   (bank_count_0),
        .bank_count_1   (bank_count_1),
        .bank_count_2   (bank_count_2),
        .bank_row_0     (bank_row_0),
        .bank_row_1     (bank_row_1),
        .bank_row_2     (bank_row_2),
        .bank_state_0   (bank_state_0),
        .bank_state_1   (bank_state_1),
        .bank_state_2   (bank_state_2),
        .current_row_id (current_row_id)
    );

    assign dbg_wr_bank = wr_bank_sel;

    //------------------------------------------------------------------
    // Write address counter cho từng bank
    //------------------------------------------------------------------
    reg [9:0] wr_addr_cnt [0:2];

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k < 3; k = k + 1)
                wr_addr_cnt[k] <= 10'd0;
        end else begin
            if (row_start && (wr_bank_sel < 3))
                wr_addr_cnt[wr_bank_sel] <= 10'd0;
            else if (wr_enable && score_valid && (wr_bank_sel < 3) &&
                     (wr_addr_cnt[wr_bank_sel] < 10'd610))
                wr_addr_cnt[wr_bank_sel] <= wr_addr_cnt[wr_bank_sel] + 10'd1;
        end
    end

    //------------------------------------------------------------------
    // 3 candidate banks – mỗi bank có 1 write + 1 read port độc lập
    //------------------------------------------------------------------
    wire [17:0] bank_wr_data = {score_value, score_x};

    wire        bank_wr_en   [0:2];
    wire [9:0]  bank_wr_addr [0:2];

    assign bank_wr_en[0]   = wr_enable && score_valid && (wr_bank_sel == 2'd0);
    assign bank_wr_en[1]   = wr_enable && score_valid && (wr_bank_sel == 2'd1);
    assign bank_wr_en[2]   = wr_enable && score_valid && (wr_bank_sel == 2'd2);

    assign bank_wr_addr[0] = wr_addr_cnt[0];
    assign bank_wr_addr[1] = wr_addr_cnt[1];
    assign bank_wr_addr[2] = wr_addr_cnt[2];

    // Bank 0
    candidate_bank u_bank0 (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (bank_wr_en[0]),
        .wr_addr (bank_wr_addr[0]),
        .wr_data (bank_wr_data),
        .rd_en   (rd_en_0),
        .rd_addr (rd_addr_0),
        .rd_data (rd_data_0)
    );

    // Bank 1
    candidate_bank u_bank1 (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (bank_wr_en[1]),
        .wr_addr (bank_wr_addr[1]),
        .wr_data (bank_wr_data),
        .rd_en   (rd_en_1),
        .rd_addr (rd_addr_1),
        .rd_data (rd_data_1)
    );

    // Bank 2
    candidate_bank u_bank2 (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (bank_wr_en[2]),
        .wr_addr (bank_wr_addr[2]),
        .wr_data (bank_wr_data),
        .rd_en   (rd_en_2),
        .rd_addr (rd_addr_2),
        .rd_data (rd_data_2)
    );

endmodule
