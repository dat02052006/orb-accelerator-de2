//==============================================================================
// nms_neighbor_search.v
// Tìm các candidate nằm trong cửa sổ 3×3 xung quanh center (x_c, y_c).
// - Candidates trong mỗi bank đã sắp xếp theo X tăng dần.
// - Duy trì 3 con trỏ đọc (chỉ tiến về phía trước) → hiệu quả cao.
// - Trả về tối đa 8 neighbor (score + valid).
// Module này là multi-cycle: start → busy → done.
//==============================================================================

module nms_neighbor_search (
    input  wire         clk,
    input  wire         rst_n,

    // Điều khiển
    input  wire         start,              // Bắt đầu tìm cho 1 center mới
    output reg          busy,
    output reg          done,               // Kết quả sẵn sàng (1 cycle)

    // Thông tin center
    input  wire [9:0]   center_x,
    input  wire [9:0]   center_y,
    input  wire [1:0]   center_bank,        // Bank chứa của center (để bỏ qua chính nó)

    // Thông tin 3 bank
    input  wire [9:0]   bank_count_0,
    input  wire [9:0]   bank_count_1,
    input  wire [9:0]   bank_count_2,
    input  wire [9:0]   bank_row_0,
    input  wire [9:0]   bank_row_1,
    input  wire [9:0]   bank_row_2,
    input  wire [2:0]   bank_state_0,       // chỉ dùng bank FULL/READING
    input  wire [2:0]   bank_state_1,
    input  wire [2:0]   bank_state_2,

    // Read ports tới candidate_buffer
    output reg          rd_en_0,
    output reg  [9:0]   rd_addr_0,
    input  wire [17:0]  rd_data_0,          // {score[7:0], x[9:0]}

    output reg          rd_en_1,
    output reg  [9:0]   rd_addr_1,
    input  wire [17:0]  rd_data_1,

    output reg          rd_en_2,
    output reg  [9:0]   rd_addr_2,
    input  wire [17:0]  rd_data_2,

    // Kết quả neighbor (tối đa 8)
    output reg  [7:0]   nb_score_0,
    output reg  [7:0]   nb_score_1,
    output reg  [7:0]   nb_score_2,
    output reg  [7:0]   nb_score_3,
    output reg  [7:0]   nb_score_4,
    output reg  [7:0]   nb_score_5,
    output reg  [7:0]   nb_score_6,
    output reg  [7:0]   nb_score_7,
    output reg  [7:0]   nb_valid            // bit mask
);

    //------------------------------------------------------------------
    // Local parameters
    //------------------------------------------------------------------
    localparam S_IDLE    = 3'd0;
    localparam S_SCAN    = 3'd1;
    localparam S_COLLECT = 3'd2;
    localparam S_DONE    = 3'd3;

    localparam BANK_FULL    = 3'd2;
    localparam BANK_READING = 3'd3;

    //------------------------------------------------------------------
    // Pointers (chỉ tăng, reset khi bắt đầu hàng mới – controller lo)
    //------------------------------------------------------------------
    reg [9:0] ptr [0:2];

    //------------------------------------------------------------------
    // Internal
    //------------------------------------------------------------------
    reg [2:0]  state;
    reg [3:0]  nb_cnt;              // số neighbor đã thu thập (0..8)
    reg [1:0]  scan_bank;           // bank đang scan
    reg [9:0]  x_min, x_max;

    wire [9:0] count [0:2];
    wire [9:0] row   [0:2];
    wire [2:0] st    [0:2];
    wire       valid_bank [0:2];

    assign count[0] = bank_count_0;
    assign count[1] = bank_count_1;
    assign count[2] = bank_count_2;
    assign row[0]   = bank_row_0;
    assign row[1]   = bank_row_1;
    assign row[2]   = bank_row_2;
    assign st[0]    = bank_state_0;
    assign st[1]    = bank_state_1;
    assign st[2]    = bank_state_2;

    assign valid_bank[0] = (st[0] == BANK_FULL) || (st[0] == BANK_READING);
    assign valid_bank[1] = (st[1] == BANK_FULL) || (st[1] == BANK_READING);
    assign valid_bank[2] = (st[2] == BANK_FULL) || (st[2] == BANK_READING);

    // Decode data đang đọc
    wire [7:0] cur_score = (scan_bank == 2'd0) ? rd_data_0[17:10] :
                           (scan_bank == 2'd1) ? rd_data_1[17:10] :
                                                 rd_data_2[17:10];
    wire [9:0] cur_x     = (scan_bank == 2'd0) ? rd_data_0[9:0] :
                           (scan_bank == 2'd1) ? rd_data_1[9:0] :
                                                 rd_data_2[9:0];

    // Y của bank đang scan có nằm trong [y_c-1, y_c+1] không?
    wire y_in_range = (row[scan_bank] >= center_y - 10'd1) &&
                      (row[scan_bank] <= center_y + 10'd1);

    // Có phải chính center không?
    wire is_center = (scan_bank == center_bank) && (cur_x == center_x);

    //------------------------------------------------------------------
    // FSM
    //------------------------------------------------------------------
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            busy    <= 1'b0;
            done    <= 1'b0;
            nb_cnt  <= 4'd0;
            nb_valid<= 8'd0;
            ptr[0]  <= 10'd0;
            ptr[1]  <= 10'd0;
            ptr[2]  <= 10'd0;
            rd_en_0 <= 1'b0;
            rd_en_1 <= 1'b0;
            rd_en_2 <= 1'b0;
            rd_addr_0 <= 10'd0;
            rd_addr_1 <= 10'd0;
            rd_addr_2 <= 10'd0;
            for (i = 0; i < 8; i = i + 1) begin
                // clear scores
            end
            nb_score_0 <= 8'd0; nb_score_1 <= 8'd0;
            nb_score_2 <= 8'd0; nb_score_3 <= 8'd0;
            nb_score_4 <= 8'd0; nb_score_5 <= 8'd0;
            nb_score_6 <= 8'd0; nb_score_7 <= 8'd0;
        end else begin
            done   <= 1'b0;
            rd_en_0 <= 1'b0;
            rd_en_1 <= 1'b0;
            rd_en_2 <= 1'b0;

            case (state)
                //--------------------------------------------------
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy     <= 1'b1;
                        nb_cnt   <= 4'd0;
                        nb_valid <= 8'd0;
                        x_min    <= (center_x == 10'd0) ? 10'd0 : center_x - 10'd1;
                        x_max    <= center_x + 10'd1;
                        scan_bank<= 2'd0;
                        state    <= S_SCAN;

                        // Clear previous results
                        nb_score_0 <= 8'd0; nb_score_1 <= 8'd0;
                        nb_score_2 <= 8'd0; nb_score_3 <= 8'd0;
                        nb_score_4 <= 8'd0; nb_score_5 <= 8'd0;
                        nb_score_6 <= 8'd0; nb_score_7 <= 8'd0;
                    end
                end

                //--------------------------------------------------
                // Duyệt từng bank, từ pointer hiện tại
                //--------------------------------------------------
                S_SCAN: begin
                    if (scan_bank > 2'd2) begin
                        state <= S_DONE;
                    end else if (!valid_bank[scan_bank] ||
                                 !( (row[scan_bank] >= center_y - 10'd1) &&
                                    (row[scan_bank] <= center_y + 10'd1) )) begin
                        // Bank không hợp lệ hoặc Y ngoài cửa sổ → sang bank tiếp
                        scan_bank <= scan_bank + 2'd1;
                    end else if (ptr[scan_bank] >= count[scan_bank]) begin
                        scan_bank <= scan_bank + 2'd1;
                    end else begin
                        // Issue read
                        case (scan_bank)
                            2'd0: begin rd_en_0 <= 1'b1; rd_addr_0 <= ptr[0]; end
                            2'd1: begin rd_en_1 <= 1'b1; rd_addr_1 <= ptr[1]; end
                            2'd2: begin rd_en_2 <= 1'b1; rd_addr_2 <= ptr[2]; end
                        endcase
                        state <= S_COLLECT;
                    end
                end

                //--------------------------------------------------
                // Nhận data (1 cycle latency), quyết định
                //--------------------------------------------------
                S_COLLECT: begin
                    // Data đã có sẵn ở rd_data_* (registered từ bank)
                    if (cur_x > x_max) begin
                        // Đã vượt cửa sổ X của bank này → sang bank khác
                        scan_bank <= scan_bank + 2'd1;
                        state     <= S_SCAN;
                    end else if (cur_x >= x_min) begin
                        // Nằm trong cửa sổ X
                        if (!is_center && (nb_cnt < 4'd8)) begin
                            // Thêm vào danh sách neighbor
                            case (nb_cnt)
                                4'd0: begin nb_score_0 <= cur_score; nb_valid[0] <= 1'b1; end
                                4'd1: begin nb_score_1 <= cur_score; nb_valid[1] <= 1'b1; end
                                4'd2: begin nb_score_2 <= cur_score; nb_valid[2] <= 1'b1; end
                                4'd3: begin nb_score_3 <= cur_score; nb_valid[3] <= 1'b1; end
                                4'd4: begin nb_score_4 <= cur_score; nb_valid[4] <= 1'b1; end
                                4'd5: begin nb_score_5 <= cur_score; nb_valid[5] <= 1'b1; end
                                4'd6: begin nb_score_6 <= cur_score; nb_valid[6] <= 1'b1; end
                                4'd7: begin nb_score_7 <= cur_score; nb_valid[7] <= 1'b1; end
                            endcase
                            nb_cnt <= nb_cnt + 4'd1;
                        end
                        // Tiến pointer và đọc tiếp (nếu còn)
                        ptr[scan_bank] <= ptr[scan_bank] + 10'd1;
                        if (ptr[scan_bank] + 10'd1 >= count[scan_bank]) begin
                            scan_bank <= scan_bank + 2'd1;
                            state     <= S_SCAN;
                        end else begin
                            // Đọc entry tiếp theo cùng bank
                            case (scan_bank)
                                2'd0: begin rd_en_0 <= 1'b1; rd_addr_0 <= ptr[0] + 10'd1; end
                                2'd1: begin rd_en_1 <= 1'b1; rd_addr_1 <= ptr[1] + 10'd1; end
                                2'd2: begin rd_en_2 <= 1'b1; rd_addr_2 <= ptr[2] + 10'd1; end
                            endcase
                            // ở lại S_COLLECT (data sẽ đến cycle sau)
                        end
                    end else begin
                        // cur_x < x_min → tiến pointer, bỏ qua
                        ptr[scan_bank] <= ptr[scan_bank] + 10'd1;
                        if (ptr[scan_bank] + 10'd1 >= count[scan_bank]) begin
                            scan_bank <= scan_bank + 2'd1;
                            state     <= S_SCAN;
                        end else begin
                            case (scan_bank)
                                2'd0: begin rd_en_0 <= 1'b1; rd_addr_0 <= ptr[0] + 10'd1; end
                                2'd1: begin rd_en_1 <= 1'b1; rd_addr_1 <= ptr[1] + 10'd1; end
                                2'd2: begin rd_en_2 <= 1'b1; rd_addr_2 <= ptr[2] + 10'd1; end
                            endcase
                        end
                    end
                end

                //--------------------------------------------------
                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    //------------------------------------------------------------------
    // Cho phép controller reset pointer khi sang hàng mới (tùy chọn)
    // Hiện tại pointer chỉ tăng → đúng vì centers xử lý theo X tăng.
    // Khi bắt đầu center row mới, controller nên gọi reset_ptr nếu cần.
    //------------------------------------------------------------------

endmodule
