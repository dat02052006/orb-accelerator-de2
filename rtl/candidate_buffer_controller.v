//==============================================================================
// candidate_buffer_controller.v
// Quản lý trạng thái 3 bank xoay vòng, row_id, candidate_count.
// Hỗ trợ NMS đọc song song cả 3 bank.
//
// Cấu trúc FSM 3 khối chuẩn:
//   KHỐI 1: State register    (sequential)  -> cập nhật state, count, row_id
//   KHỐI 2: Next-state logic  (combinational) -> tính next_state
//   KHỐI 3: Output logic      (combinational) -> xuất tín hiệu ra ngoài
//==============================================================================

module candidate_buffer_controller (
    input  wire         clk,
    input  wire         rst_n,

    // ---- Từ Score module ----
    input  wire         row_start,
    input  wire         row_done,
    input  wire         score_valid,

    // ---- Từ NMS module ----
    input  wire         nms_done,
    input  wire [1:0]   nms_done_bank,

    // ---- Ra ngoài ----
    output wire [1:0]   wr_bank_sel,
    output wire         wr_enable,
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

    output wire [9:0]   current_row_id
);

    //------------------------------------------------------------------
    // Định nghĩa trạng thái
    //------------------------------------------------------------------
    localparam S_EMPTY   = 3'd0;
    localparam S_WRITING = 3'd1;
    localparam S_FULL    = 3'd2;
    localparam S_READING = 3'd3;

    localparam COUNT_MAX = 10'd610;

    //------------------------------------------------------------------
    // Thanh ghi trạng thái nội bộ (3 bank)
    //------------------------------------------------------------------
    reg [2:0] state_0, state_1, state_2;
    reg [9:0] count_0, count_1, count_2;
    reg [9:0] row_id_0, row_id_1, row_id_2;
    reg [9:0] row_id_cur;

    //------------------------------------------------------------------
    // Next-state (combinational)
    //------------------------------------------------------------------
    reg [2:0] next_state_0, next_state_1, next_state_2;
    reg [9:0] next_count_0, next_count_1, next_count_2;
    reg [9:0] next_row_id_0, next_row_id_1, next_row_id_2;
    reg [9:0] next_row_id_cur;

    //------------------------------------------------------------------
    // Tín hiệu tổ hợp
    //------------------------------------------------------------------
    wire [1:0] empty_bank;
    wire       empty_bank_valid;
    wire [1:0] active_wr_bank;
    wire       active_wr_en;

    //==========================================================================
    // KHỐI 1: STATE REGISTER (SEQUENTIAL)
    //   Chỉ cập nhật state + dữ liệu liên quan (count, row_id)
    //==========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_0    <= S_EMPTY;
            state_1    <= S_EMPTY;
            state_2    <= S_EMPTY;
            count_0    <= 10'd0;
            count_1    <= 10'd0;
            count_2    <= 10'd0;
            row_id_0   <= 10'd0;
            row_id_1   <= 10'd0;
            row_id_2   <= 10'd0;
            row_id_cur <= 10'd0;
        end else begin
            state_0    <= next_state_0;
            state_1    <= next_state_1;
            state_2    <= next_state_2;
            count_0    <= next_count_0;
            count_1    <= next_count_1;
            count_2    <= next_count_2;
            row_id_0   <= next_row_id_0;
            row_id_1   <= next_row_id_1;
            row_id_2   <= next_row_id_2;
            row_id_cur <= next_row_id_cur;
        end
    end

    //==========================================================================
    // KHỐI 2: NEXT-STATE LOGIC (COMBINATIONAL)
    //==========================================================================

    //------------------------------------------------------------------
    // 2.1 Tìm bank trống (thay function find_empty_bank)
    //------------------------------------------------------------------
    assign empty_bank_valid = (state_0 == S_EMPTY) ||
                              (state_1 == S_EMPTY) ||
                              (state_2 == S_EMPTY);

    assign empty_bank = (state_0 == S_EMPTY) ? 2'd0 :
                        (state_1 == S_EMPTY) ? 2'd1 :
                        (state_2 == S_EMPTY) ? 2'd2 :
                                               2'd3;

    //------------------------------------------------------------------
    // 2.2 Xác định bank đang ghi + wr_enable hiện tại
    //     (dùng để đếm score / nhận row_done)
    //------------------------------------------------------------------
    assign active_wr_bank = (state_0 == S_WRITING) ? 2'd0 :
                            (state_1 == S_WRITING) ? 2'd1 :
                            (state_2 == S_WRITING) ? 2'd2 :
                                                     2'd3;
    assign active_wr_en   = (active_wr_bank != 2'd3);

    //------------------------------------------------------------------
    // 2.3 Tính next-state
    //------------------------------------------------------------------
    always @(*) begin
        // Mặc định: giữ nguyên
        next_state_0    = state_0;
        next_state_1    = state_1;
        next_state_2    = state_2;
        next_count_0    = count_0;
        next_count_1    = count_1;
        next_count_2    = count_2;
        next_row_id_0   = row_id_0;
        next_row_id_1   = row_id_1;
        next_row_id_2   = row_id_2;
        next_row_id_cur = row_id_cur;

        //--------------------------------------------------------------
        // Ưu tiên 1: row_start -> chọn bank trống, bắt đầu ghi
        //--------------------------------------------------------------
        if (row_start && empty_bank_valid) begin
            case (empty_bank)
                2'd0: begin
                    next_state_0  = S_WRITING;
                    next_count_0  = 10'd0;
                    next_row_id_0 = row_id_cur;
                end
                2'd1: begin
                    next_state_1  = S_WRITING;
                    next_count_1  = 10'd0;
                    next_row_id_1 = row_id_cur;
                end
                2'd2: begin
                    next_state_2  = S_WRITING;
                    next_count_2  = 10'd0;
                    next_row_id_2 = row_id_cur;
                end
            endcase
        end

        //--------------------------------------------------------------
        // Ưu tiên 2: score_valid -> tăng count của bank đang ghi
        //--------------------------------------------------------------
        if (active_wr_en && score_valid) begin
            case (active_wr_bank)
                2'd0: if (count_0 < COUNT_MAX) next_count_0 = count_0 + 10'd1;
                2'd1: if (count_1 < COUNT_MAX) next_count_1 = count_1 + 10'd1;
                2'd2: if (count_2 < COUNT_MAX) next_count_2 = count_2 + 10'd1;
            endcase
        end

        //--------------------------------------------------------------
        // Ưu tiên 3: row_done -> bank đang ghi chuyển FULL
        //--------------------------------------------------------------
        if (row_done && active_wr_en) begin
            next_row_id_cur = row_id_cur + 10'd1;
            case (active_wr_bank)
                2'd0: next_state_0 = S_FULL;
                2'd1: next_state_1 = S_FULL;
                2'd2: next_state_2 = S_FULL;
            endcase
        end

        //--------------------------------------------------------------
        // Ưu tiên 4: NMS done -> giải phóng bank
        //--------------------------------------------------------------
        if (nms_done) begin
            case (nms_done_bank)
                2'd0: if (state_0 == S_FULL || state_0 == S_READING) begin
                          next_state_0 = S_EMPTY;
                          next_count_0 = 10'd0;
                      end
                2'd1: if (state_1 == S_FULL || state_1 == S_READING) begin
                          next_state_1 = S_EMPTY;
                          next_count_1 = 10'd0;
                      end
                2'd2: if (state_2 == S_FULL || state_2 == S_READING) begin
                          next_state_2 = S_EMPTY;
                          next_count_2 = 10'd0;
                      end
            endcase
        end
    end

    //==========================================================================
    // KHỐI 3: OUTPUT LOGIC (COMBINATIONAL)
    //   Tất cả output đều tổ hợp, không có thanh ghi trễ
    //==========================================================================

    // wr_bank_sel: bank đang được ghi (hoặc bank vừa chọn khi row_start)
    assign wr_bank_sel = (row_start && empty_bank_valid) ? empty_bank :
                                                           active_wr_bank;

    // wr_enable: đang trong giai đoạn ghi
    assign wr_enable   = (row_start && empty_bank_valid) ? 1'b1 :
                                                           active_wr_en;

    // bank_ready: có ít nhất 1 bank sẵn sàng cho NMS
    assign bank_ready  = (state_0 == S_FULL) || (state_0 == S_READING) ||
                         (state_1 == S_FULL) || (state_1 == S_READING) ||
                         (state_2 == S_FULL) || (state_2 == S_READING);

    // Thông tin từng bank (tổ hợp trực tiếp từ thanh ghi)
    assign bank_count_0 = count_0;
    assign bank_count_1 = count_1;
    assign bank_count_2 = count_2;
    assign bank_row_0   = row_id_0;
    assign bank_row_1   = row_id_1;
    assign bank_row_2   = row_id_2;
    assign bank_state_0 = state_0;
    assign bank_state_1 = state_1;
    assign bank_state_2 = state_2;
    assign current_row_id = row_id_cur;

endmodule