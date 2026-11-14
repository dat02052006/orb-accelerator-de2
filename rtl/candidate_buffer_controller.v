//==============================================================================
// candidate_buffer_controller.v
// Quản lý trạng thái 3 bank xoay vòng, row_id, candidate_count,
// tín hiệu hoàn thành ghi và quyền đọc/ghi.
//==============================================================================

module candidate_buffer_controller (
    input  wire         clk,
    input  wire         rst_n,

    // ---- Từ Score module ----
    input  wire         row_start,      // Bắt đầu hàng mới
    input  wire         row_done,       // Score đã ghi xong hàng hiện tại
    input  wire         score_valid,    // Candidate hợp lệ (để đếm)

    // ---- Từ NMS module ----
    input  wire         nms_req,        // NMS yêu cầu một bank sẵn sàng
    input  wire         nms_done,       // NMS đã xử lý xong bank đang đọc

    // ---- Ra ngoài (tới buffer & NMS) ----
    output reg  [1:0]   wr_bank_sel,    // Bank đang được Score ghi (0/1/2)
    output reg  [1:0]   rd_bank_sel,    // Bank đang được NMS đọc
    output reg          wr_enable,      // Cho phép Score ghi vào bank hiện tại
    output reg          bank_ready,     // Có ít nhất 1 bank sẵn sàng cho NMS
    output reg  [9:0]   candidate_count,// Số candidate của bank đang đọc
    output reg  [9:0]   current_row_id, // Hàng ảnh đang ghi

    // Debug / status (optional)
    output wire [2:0]   bank_state_0,   // 0=empty, 1=writing, 2=full, 3=reading
    output wire [2:0]   bank_state_1,
    output wire [2:0]   bank_state_2
);

    //------------------------------------------------------------------
    // Bank state encoding
    //------------------------------------------------------------------
    localparam S_EMPTY   = 3'd0;
    localparam S_WRITING = 3'd1;
    localparam S_FULL    = 3'd2;
    localparam S_READING = 3'd3;

    reg [2:0] state [0:2];
    reg [9:0] count [0:2];          // candidate count của mỗi bank
    reg [9:0] row_id [0:2];         // row_id gắn với mỗi bank

    assign bank_state_0 = state[0];
    assign bank_state_1 = state[1];
    assign bank_state_2 = state[2];

    //------------------------------------------------------------------
    // Tìm bank trống để ghi (ưu tiên theo thứ tự 0→1→2)
    //------------------------------------------------------------------
    function [1:0] find_empty_bank;
        input [2:0] s0, s1, s2;
        begin
            if (s0 == S_EMPTY)      find_empty_bank = 2'd0;
            else if (s1 == S_EMPTY) find_empty_bank = 2'd1;
            else if (s2 == S_EMPTY) find_empty_bank = 2'd2;
            else                    find_empty_bank = 2'd3; // full (error)
        end
    endfunction

    //------------------------------------------------------------------
    // Tìm bank FULL để đưa cho NMS
    //------------------------------------------------------------------
    function [1:0] find_full_bank;
        input [2:0] s0, s1, s2;
        begin
            if (s0 == S_FULL)      find_full_bank = 2'd0;
            else if (s1 == S_FULL) find_full_bank = 2'd1;
            else if (s2 == S_FULL) find_full_bank = 2'd2;
            else                   find_full_bank = 2'd3; // none
        end
    endfunction

    //------------------------------------------------------------------
    // Main FSM
    //------------------------------------------------------------------
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 3; i = i + 1) begin
                state[i] <= S_EMPTY;
                count[i] <= 10'd0;
                row_id[i] <= 10'd0;
            end
            wr_bank_sel     <= 2'd0;
            rd_bank_sel     <= 2'd0;
            wr_enable       <= 1'b0;
            bank_ready      <= 1'b0;
            candidate_count <= 10'd0;
            current_row_id  <= 10'd0;
        end else begin
            //----------------------------------------------------------
            // 1. Xử lý row_start → chọn bank trống và bắt đầu ghi
            //----------------------------------------------------------
            if (row_start) begin
                wr_bank_sel <= find_empty_bank(state[0], state[1], state[2]);
                if (find_empty_bank(state[0], state[1], state[2]) != 2'd3) begin
                    state[find_empty_bank(state[0], state[1], state[2])] <= S_WRITING;
                    count[find_empty_bank(state[0], state[1], state[2])] <= 10'd0;
                    row_id[find_empty_bank(state[0], state[1], state[2])] <= current_row_id;
                    wr_enable <= 1'b1;
                end else begin
                    // Không còn bank trống → tạm dừng ghi (back-pressure)
                    wr_enable <= 1'b0;
                end
            end

            //----------------------------------------------------------
            // 2. Đếm candidate khi score_valid trong lúc đang ghi
            //----------------------------------------------------------
            if (wr_enable && score_valid && (wr_bank_sel < 3)) begin
                if (count[wr_bank_sel] < 10'd610)
                    count[wr_bank_sel] <= count[wr_bank_sel] + 10'd1;
            end

            //----------------------------------------------------------
            // 3. row_done → chuyển bank sang FULL
            //----------------------------------------------------------
            if (row_done && wr_enable && (wr_bank_sel < 3)) begin
                state[wr_bank_sel] <= S_FULL;
                wr_enable <= 1'b0;
                current_row_id <= current_row_id + 10'd1;
            end

            //----------------------------------------------------------
            // 4. NMS request → cấp bank FULL cho NMS
            //----------------------------------------------------------
            if (nms_req && (find_full_bank(state[0], state[1], state[2]) != 2'd3)) begin
                rd_bank_sel     <= find_full_bank(state[0], state[1], state[2]);
                candidate_count <= count[find_full_bank(state[0], state[1], state[2])];
                state[find_full_bank(state[0], state[1], state[2])] <= S_READING;
            end

            //----------------------------------------------------------
            // 5. NMS done → giải phóng bank về EMPTY
            //----------------------------------------------------------
            if (nms_done && (rd_bank_sel < 3) && (state[rd_bank_sel] == S_READING)) begin
                state[rd_bank_sel] <= S_EMPTY;
                count[rd_bank_sel] <= 10'd0;
            end

            //----------------------------------------------------------
            // 6. bank_ready = có ít nhất 1 bank ở trạng thái FULL
            //----------------------------------------------------------
            bank_ready <= (state[0] == S_FULL) ||
                          (state[1] == S_FULL) ||
                          (state[2] == S_FULL);
        end
    end

endmodule
