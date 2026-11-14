//==============================================================================
// nms_controller.v
// FSM chính của NMS.
// Trạng thái: IDLE → SELECT → LOAD_CENTER → SEARCH → COMPARE → WRITE → NEXT → DONE
//==============================================================================

module nms_controller (
    input  wire         clk,
    input  wire         rst_n,

    // Từ Candidate Buffer
    input  wire         bank_ready,
    input  wire [9:0]   bank_count_0,
    input  wire [9:0]   bank_count_1,
    input  wire [9:0]   bank_count_2,
    input  wire [9:0]   bank_row_0,
    input  wire [9:0]   bank_row_1,
    input  wire [9:0]   bank_row_2,
    input  wire [2:0]   bank_state_0,
    input  wire [2:0]   bank_state_1,
    input  wire [2:0]   bank_state_2,

    // Điều khiển neighbor search
    output reg          nb_start,
    input  wire         nb_busy,
    input  wire         nb_done,
    output reg  [9:0]   center_x,
    output reg  [9:0]   center_y,
    output reg  [1:0]   center_bank,
    output reg  [7:0]   center_score,   // latched score cho comparator

    // Từ comparator
    input  wire         keep,

    // Đọc center từ bank
    output reg          center_rd_en,
    output reg  [9:0]   center_rd_addr,
    output reg  [1:0]   center_rd_bank,
    input  wire [17:0]  center_rd_data,

    // Ghi keypoint
    output reg          kp_valid,
    output reg  [27:0]  kp_data,            // {score[7:0], y[9:0], x[9:0]}
    input  wire         kp_ready,

    // Báo cáo
    output reg          nms_done,
    output reg  [1:0]   nms_done_bank,
    output reg          nms_busy
);

    //------------------------------------------------------------------
    // States
    //------------------------------------------------------------------
    localparam S_IDLE            = 4'd0;
    localparam S_SELECT_CENTER   = 4'd1;
    localparam S_LOAD_CENTER     = 4'd2;
    localparam S_WAIT_CENTER     = 4'd3;
    localparam S_SEARCH          = 4'd4;
    localparam S_WAIT_SEARCH     = 4'd5;
    localparam S_COMPARE         = 4'd6;
    localparam S_WRITE_RESULT    = 4'd7;
    localparam S_NEXT_CANDIDATE  = 4'd8;
    localparam S_ROW_DONE        = 4'd9;

    localparam BANK_FULL    = 3'd2;
    localparam BANK_READING = 3'd3;

    reg [3:0]  state;
    reg [9:0]  cand_idx;
    reg [1:0]  cur_center_bank;
    reg [9:0]  cur_center_row;
    reg [9:0]  cur_center_count;
    reg [9:0]  center_x_r;

    // Chọn bank FULL có row_id nhỏ nhất
    function [1:0] select_oldest_full_bank;
        input [2:0] s0, s1, s2;
        input [9:0] r0, r1, r2;
        reg [1:0] sel;
        reg [9:0] min_row;
        begin
            sel     = 2'd3;
            min_row = 10'h3FF;
            if ((s0 == BANK_FULL || s0 == BANK_READING) && (r0 < min_row)) begin
                sel = 2'd0; min_row = r0;
            end
            if ((s1 == BANK_FULL || s1 == BANK_READING) && (r1 < min_row)) begin
                sel = 2'd1; min_row = r1;
            end
            if ((s2 == BANK_FULL || s2 == BANK_READING) && (r2 < min_row)) begin
                sel = 2'd2;
            end
            select_oldest_full_bank = sel;
        end
    endfunction

    //------------------------------------------------------------------
    // FSM
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            nms_busy         <= 1'b0;
            nms_done         <= 1'b0;
            nms_done_bank    <= 2'd0;
            nb_start         <= 1'b0;
            kp_valid         <= 1'b0;
            kp_data          <= 28'd0;
            center_rd_en     <= 1'b0;
            center_rd_addr   <= 10'd0;
            center_rd_bank   <= 2'd0;
            cand_idx         <= 10'd0;
            center_x         <= 10'd0;
            center_y         <= 10'd0;
            center_bank      <= 2'd0;
            center_score     <= 8'd0;
            center_x_r       <= 10'd0;
            cur_center_bank  <= 2'd0;
            cur_center_row   <= 10'd0;
            cur_center_count <= 10'd0;
        end else begin
            nms_done     <= 1'b0;
            nb_start     <= 1'b0;
            kp_valid     <= 1'b0;
            center_rd_en <= 1'b0;

            case (state)
                //--------------------------------------------------
                S_IDLE: begin
                    nms_busy <= 1'b0;
                    if (bank_ready) begin
                        nms_busy <= 1'b1;
                        state    <= S_SELECT_CENTER;
                    end
                end

                //--------------------------------------------------
                S_SELECT_CENTER: begin
                    cur_center_bank <= select_oldest_full_bank(
                        bank_state_0, bank_state_1, bank_state_2,
                        bank_row_0,   bank_row_1,   bank_row_2);

                    if (select_oldest_full_bank(
                            bank_state_0, bank_state_1, bank_state_2,
                            bank_row_0, bank_row_1, bank_row_2) == 2'd3) begin
                        state <= S_IDLE;
                    end else begin
                        cand_idx <= 10'd0;
                        state    <= S_LOAD_CENTER;
                    end
                end

                //--------------------------------------------------
                S_LOAD_CENTER: begin
                    // Cập nhật thông tin bank (dùng giá trị đã chọn)
                    case (cur_center_bank)
                        2'd0: begin
                            cur_center_row   <= bank_row_0;
                            cur_center_count <= bank_count_0;
                        end
                        2'd1: begin
                            cur_center_row   <= bank_row_1;
                            cur_center_count <= bank_count_1;
                        end
                        default: begin
                            cur_center_row   <= bank_row_2;
                            cur_center_count <= bank_count_2;
                        end
                    endcase

                    // Kiểm tra hết candidate (dùng count đã latch từ cycle trước
                    // hoặc chấp nhận 1 cycle delay – an toàn vì count ổn định)
                    if (cand_idx >= (
                            (cur_center_bank == 2'd0) ? bank_count_0 :
                            (cur_center_bank == 2'd1) ? bank_count_1 : bank_count_2)) begin
                        state <= S_ROW_DONE;
                    end else begin
                        center_rd_bank <= cur_center_bank;
                        center_rd_addr <= cand_idx;
                        center_rd_en   <= 1'b1;
                        state          <= S_WAIT_CENTER;
                    end
                end

                //--------------------------------------------------
                S_WAIT_CENTER: begin
                    // Data center đã sẵn (registered từ RAM)
                    center_score <= center_rd_data[17:10];
                    center_x_r   <= center_rd_data[9:0];
                    center_x     <= center_rd_data[9:0];
                    center_y     <= cur_center_row;
                    center_bank  <= cur_center_bank;
                    state        <= S_SEARCH;
                end

                //--------------------------------------------------
                S_SEARCH: begin
                    nb_start <= 1'b1;
                    state    <= S_WAIT_SEARCH;
                end

                //--------------------------------------------------
                S_WAIT_SEARCH: begin
                    if (nb_done)
                        state <= S_COMPARE;
                end

                //--------------------------------------------------
                S_COMPARE: begin
                    if (keep)
                        state <= S_WRITE_RESULT;
                    else
                        state <= S_NEXT_CANDIDATE;
                end

                //--------------------------------------------------
                S_WRITE_RESULT: begin
                    if (kp_ready) begin
                        kp_valid <= 1'b1;
                        kp_data  <= {center_score, cur_center_row, center_x_r};
                        state    <= S_NEXT_CANDIDATE;
                    end
                end

                //--------------------------------------------------
                S_NEXT_CANDIDATE: begin
                    cand_idx <= cand_idx + 10'd1;
                    state    <= S_LOAD_CENTER;
                end

                //--------------------------------------------------
                S_ROW_DONE: begin
                    nms_done      <= 1'b1;
                    nms_done_bank <= cur_center_bank;
                    state         <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
