//==============================================================================
// candidate_buffer.v
// Quản lý 3 bank M4K xoay vòng.
// Score ghi 1 bank, NMS đọc song song cả 3 bank.
//==============================================================================

module candidate_buffer (
    input  wire         clk,
    input  wire         rst_n,

    // ---- Score interface ----
    input  wire         score_valid,
    input  wire [9:0]   score_x,
    input  wire [7:0]   score_value,
    input  wire         row_start,
    input  wire         row_done,

    // ---- NMS interface ----
    input  wire         nms_done,
    input  wire [1:0]   nms_done_bank,

    // 3 independent read ports
    input  wire         rd_en_0,
    input  wire [9:0]   rd_addr_0,
    output wire [17:0]  rd_data_0,

    input  wire         rd_en_1,
    input  wire [9:0]   rd_addr_1,
    output wire [17:0]  rd_data_1,

    input  wire         rd_en_2,
    input  wire [9:0]   rd_addr_2,
    output wire [17:0]  rd_data_2,

    // Bank status
    output wire         bank_ready,
    output wire [9:0]   bank_count_0,
    output wire [9:0]   bank_count_1,
    output wire [9:0]   bank_count_2,
    output wire [9:0]   bank_row_0,
    output wire [9:0]   bank_row_1,
    output wire [9:0]   bank_row_2,
    output wire [2:0]   bank_state_0,
    output wire [2:0]   bank_state_1,
    output wire [2:0]   bank_state_2,

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
    // Write address counters (3 riêng biệt, không dùng for)
    //------------------------------------------------------------------
    reg [9:0] wr_addr_cnt_0;
    reg [9:0] wr_addr_cnt_1;
    reg [9:0] wr_addr_cnt_2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_addr_cnt_0 <= 10'd0;
            wr_addr_cnt_1 <= 10'd0;
            wr_addr_cnt_2 <= 10'd0;
        end else begin
            if (row_start && (wr_bank_sel == 2'd0))
                wr_addr_cnt_0 <= 10'd0;
            else if (wr_enable && score_valid && (wr_bank_sel == 2'd0) &&
                     (wr_addr_cnt_0 < 10'd610))
                wr_addr_cnt_0 <= wr_addr_cnt_0 + 10'd1;

            if (row_start && (wr_bank_sel == 2'd1))
                wr_addr_cnt_1 <= 10'd0;
            else if (wr_enable && score_valid && (wr_bank_sel == 2'd1) &&
                     (wr_addr_cnt_1 < 10'd610))
                wr_addr_cnt_1 <= wr_addr_cnt_1 + 10'd1;

            if (row_start && (wr_bank_sel == 2'd2))
                wr_addr_cnt_2 <= 10'd0;
            else if (wr_enable && score_valid && (wr_bank_sel == 2'd2) &&
                     (wr_addr_cnt_2 < 10'd610))
                wr_addr_cnt_2 <= wr_addr_cnt_2 + 10'd1;
        end
    end

    //------------------------------------------------------------------
    // 3 banks
    //------------------------------------------------------------------
    wire [17:0] bank_wr_data = {score_value, score_x};

    wire wr_en_0 = wr_enable && score_valid && (wr_bank_sel == 2'd0);
    wire wr_en_1 = wr_enable && score_valid && (wr_bank_sel == 2'd1);
    wire wr_en_2 = wr_enable && score_valid && (wr_bank_sel == 2'd2);

    candidate_bank u_bank0 (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (wr_en_0),
        .wr_addr (wr_addr_cnt_0),
        .wr_data (bank_wr_data),
        .rd_en   (rd_en_0),
        .rd_addr (rd_addr_0),
        .rd_data (rd_data_0)
    );

    candidate_bank u_bank1 (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (wr_en_1),
        .wr_addr (wr_addr_cnt_1),
        .wr_data (bank_wr_data),
        .rd_en   (rd_en_1),
        .rd_addr (rd_addr_1),
        .rd_data (rd_data_1)
    );

    candidate_bank u_bank2 (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_en   (wr_en_2),
        .wr_addr (wr_addr_cnt_2),
        .wr_data (bank_wr_data),
        .rd_en   (rd_en_2),
        .rd_addr (rd_addr_2),
        .rd_data (rd_data_2)
    );

endmodule
