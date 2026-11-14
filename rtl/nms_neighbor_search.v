//==============================================================================
// nms_neighbor_search.v
// Tìm candidate trong cửa sổ 3×3 quanh center (x_c, y_c).
// Candidates đã sort theo X tăng dần → dùng 3 pointer chỉ tiến.
// Multi-cycle: start → busy → done.
//==============================================================================

module nms_neighbor_search (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         start,
    output reg          busy,
    output reg          done,

    input  wire [9:0]   center_x,
    input  wire [9:0]   center_y,
    input  wire [1:0]   center_bank,

    input  wire [9:0]   bank_count_0,
    input  wire [9:0]   bank_count_1,
    input  wire [9:0]   bank_count_2,
    input  wire [9:0]   bank_row_0,
    input  wire [9:0]   bank_row_1,
    input  wire [9:0]   bank_row_2,
    input  wire [2:0]   bank_state_0,
    input  wire [2:0]   bank_state_1,
    input  wire [2:0]   bank_state_2,

    output reg          rd_en_0,
    output reg  [9:0]   rd_addr_0,
    input  wire [17:0]  rd_data_0,

    output reg          rd_en_1,
    output reg  [9:0]   rd_addr_1,
    input  wire [17:0]  rd_data_1,

    output reg          rd_en_2,
    output reg  [9:0]   rd_addr_2,
    input  wire [17:0]  rd_data_2,

    output reg  [7:0]   nb_score_0,
    output reg  [7:0]   nb_score_1,
    output reg  [7:0]   nb_score_2,
    output reg  [7:0]   nb_score_3,
    output reg  [7:0]   nb_score_4,
    output reg  [7:0]   nb_score_5,
    output reg  [7:0]   nb_score_6,
    output reg  [7:0]   nb_score_7,
    output reg  [7:0]   nb_valid
);

    localparam S_IDLE    = 3'd0;
    localparam S_SCAN    = 3'd1;
    localparam S_COLLECT = 3'd2;
    localparam S_DONE    = 3'd3;

    localparam BANK_FULL    = 3'd2;
    localparam BANK_READING = 3'd3;

    // 3 pointer riêng (không dùng mảng + for)
    reg [9:0] ptr_0, ptr_1, ptr_2;

    reg [2:0]  state;
    reg [3:0]  nb_cnt;
    reg [1:0]  scan_bank;
    reg [9:0]  x_min, x_max;

    // Helper wires
    wire [9:0] count_sel = (scan_bank == 2'd0) ? bank_count_0 :
                           (scan_bank == 2'd1) ? bank_count_1 : bank_count_2;

    wire [9:0] row_sel   = (scan_bank == 2'd0) ? bank_row_0 :
                           (scan_bank == 2'd1) ? bank_row_1 : bank_row_2;

    wire [2:0] state_sel = (scan_bank == 2'd0) ? bank_state_0 :
                           (scan_bank == 2'd1) ? bank_state_1 : bank_state_2;

    wire       bank_ok   = (state_sel == BANK_FULL) || (state_sel == BANK_READING);

    wire [9:0] ptr_sel   = (scan_bank == 2'd0) ? ptr_0 :
                           (scan_bank == 2'd1) ? ptr_1 : ptr_2;

    wire [7:0] cur_score = (scan_bank == 2'd0) ? rd_data_0[17:10] :
                           (scan_bank == 2'd1) ? rd_data_1[17:10] : rd_data_2[17:10];

    wire [9:0] cur_x     = (scan_bank == 2'd0) ? rd_data_0[9:0] :
                           (scan_bank == 2'd1) ? rd_data_1[9:0] : rd_data_2[9:0];

    wire y_in_range = (row_sel >= center_y - 10'd1) && (row_sel <= center_y + 10'd1);
    wire is_center  = (scan_bank == center_bank) && (cur_x == center_x);

    //------------------------------------------------------------------
    // FSM
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            busy      <= 1'b0;
            done      <= 1'b0;
            nb_cnt    <= 4'd0;
            nb_valid  <= 8'd0;
            ptr_0     <= 10'd0;
            ptr_1     <= 10'd0;
            ptr_2     <= 10'd0;
            rd_en_0   <= 1'b0;
            rd_en_1   <= 1'b0;
            rd_en_2   <= 1'b0;
            rd_addr_0 <= 10'd0;
            rd_addr_1 <= 10'd0;
            rd_addr_2 <= 10'd0;
            scan_bank <= 2'd0;
            x_min     <= 10'd0;
            x_max     <= 10'd0;
            nb_score_0 <= 8'd0;
            nb_score_1 <= 8'd0;
            nb_score_2 <= 8'd0;
            nb_score_3 <= 8'd0;
            nb_score_4 <= 8'd0;
            nb_score_5 <= 8'd0;
            nb_score_6 <= 8'd0;
            nb_score_7 <= 8'd0;
        end else begin
            done    <= 1'b0;
            rd_en_0 <= 1'b0;
            rd_en_1 <= 1'b0;
            rd_en_2 <= 1'b0;

            case (state)
                //--------------------------------------------------
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        nb_cnt    <= 4'd0;
                        nb_valid  <= 8'd0;
                        x_min     <= (center_x == 10'd0) ? 10'd0 : (center_x - 10'd1);
                        x_max     <= center_x + 10'd1;
                        scan_bank <= 2'd0;
                        nb_score_0 <= 8'd0;
                        nb_score_1 <= 8'd0;
                        nb_score_2 <= 8'd0;
                        nb_score_3 <= 8'd0;
                        nb_score_4 <= 8'd0;
                        nb_score_5 <= 8'd0;
                        nb_score_6 <= 8'd0;
                        nb_score_7 <= 8'd0;
                        state     <= S_SCAN;
                    end
                end

                //--------------------------------------------------
                S_SCAN: begin
                    if (scan_bank > 2'd2) begin
                        state <= S_DONE;
                    end else if (!bank_ok || !y_in_range) begin
                        scan_bank <= scan_bank + 2'd1;
                    end else if (ptr_sel >= count_sel) begin
                        scan_bank <= scan_bank + 2'd1;
                    end else begin
                        // Issue read
                        case (scan_bank)
                            2'd0: begin
                                rd_en_0   <= 1'b1;
                                rd_addr_0 <= ptr_0;
                            end
                            2'd1: begin
                                rd_en_1   <= 1'b1;
                                rd_addr_1 <= ptr_1;
                            end
                            2'd2: begin
                                rd_en_2   <= 1'b1;
                                rd_addr_2 <= ptr_2;
                            end
                        endcase
                        state <= S_COLLECT;
                    end
                end

                //--------------------------------------------------
                S_COLLECT: begin
                    // Data đã có (1-cycle latency từ RAM)
                    if (cur_x > x_max) begin
                        // Vượt cửa sổ X → sang bank khác
                        scan_bank <= scan_bank + 2'd1;
                        state     <= S_SCAN;
                    end else if (cur_x >= x_min) begin
                        // Trong cửa sổ
                        if (!is_center && (nb_cnt < 4'd8)) begin
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

                        // Tiến pointer
                        case (scan_bank)
                            2'd0: ptr_0 <= ptr_0 + 10'd1;
                            2'd1: ptr_1 <= ptr_1 + 10'd1;
                            2'd2: ptr_2 <= ptr_2 + 10'd1;
                        endcase

                        if (ptr_sel + 10'd1 >= count_sel) begin
                            scan_bank <= scan_bank + 2'd1;
                            state     <= S_SCAN;
                        end else begin
                            // Đọc tiếp cùng bank
                            case (scan_bank)
                                2'd0: begin
                                    rd_en_0   <= 1'b1;
                                    rd_addr_0 <= ptr_0 + 10'd1;
                                end
                                2'd1: begin
                                    rd_en_1   <= 1'b1;
                                    rd_addr_1 <= ptr_1 + 10'd1;
                                end
                                2'd2: begin
                                    rd_en_2   <= 1'b1;
                                    rd_addr_2 <= ptr_2 + 10'd1;
                                end
                            endcase
                            // giữ S_COLLECT
                        end
                    end else begin
                        // cur_x < x_min → bỏ qua, tiến pointer
                        case (scan_bank)
                            2'd0: ptr_0 <= ptr_0 + 10'd1;
                            2'd1: ptr_1 <= ptr_1 + 10'd1;
                            2'd2: ptr_2 <= ptr_2 + 10'd1;
                        endcase

                        if (ptr_sel + 10'd1 >= count_sel) begin
                            scan_bank <= scan_bank + 2'd1;
                            state     <= S_SCAN;
                        end else begin
                            case (scan_bank)
                                2'd0: begin
                                    rd_en_0   <= 1'b1;
                                    rd_addr_0 <= ptr_0 + 10'd1;
                                end
                                2'd1: begin
                                    rd_en_1   <= 1'b1;
                                    rd_addr_1 <= ptr_1 + 10'd1;
                                end
                                2'd2: begin
                                    rd_en_2   <= 1'b1;
                                    rd_addr_2 <= ptr_2 + 10'd1;
                                end
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

endmodule
