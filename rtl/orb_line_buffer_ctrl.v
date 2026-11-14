module orb_line_buffer_ctrl (
    input  wire         clk,
    input  wire         rst_n,

    // ---- dieu khien chung ----
    input  wire         start,          // bat dau xu ly ca anh
    output wire         busy,
    output wire         frame_done,     // xung khi het anh
    output wire         error,
    output wire         row_done,       // xung khi xong 1 hang tam (cua so da phat het)
    output reg  [8:0]   center_y,       // hang tam hien tai (3..476)

    // ---- sang image_reader ----
    output wire         ir_start,
    output wire [8:0]   ir_first_row,
    output wire [4:0]   ir_first_bank,
    output wire [4:0]   ir_num_rows,
    input  wire         ir_done,
    input  wire         ir_error,

    // ---- sang orb_line_buffer (cong doc) ----
    output wire         lb_rd_en,
    output wire [9:0]   lb_rd_col,
    input  wire         lb_rd_valid,
    input  wire [7:0]   lb_p0, lb_p1, lb_p2, lb_p3, lb_p4, lb_p5, lb_p6,   // theo bank vat ly

    // ---- sang orb_window_generator ----
    output wire         win_clear,
    output wire         win_in_valid,
    output reg  [7:0]   win_p0, win_p1, win_p2, win_p3, win_p4, win_p5, win_p6  // theo hang logic
);

    localparam [8:0] FIRST_CENTER = 9'd3;
    localparam [8:0] LAST_CENTER  = 9'd476;      // 480 - 4
    localparam [9:0] LAST_COL     = 10'd639;
    localparam [2:0] FLUSH_LAST   = 3'd3;

    // ---------------- state encoding ----------------
    localparam [3:0] S_IDLE        = 4'd0,
                     S_LOAD_REQ    = 4'd1,
                     S_LOAD_WAIT   = 4'd2,
                     S_SCAN_CLR    = 4'd3,
                     S_SCAN        = 4'd4,
                     S_FLUSH       = 4'd5,
                     S_ROW_END     = 4'd6,
                     S_REFILL_REQ  = 4'd7,
                     S_REFILL_WAIT = 4'd8,
                     S_DONE        = 4'd9,
                     S_ERROR       = 4'd10;

    reg [3:0] state, next_state;

    // ---------------- datapath registers ----------------
    reg [2:0] base;         // bank vat ly cua hang logic 0 (hang tren cung)
    reg [9:0] col_cnt;      // cot dang doc
    reg [2:0] flush_cnt;

    // ======================================================
    // FSM phan 1: next-state transition logic
    // ======================================================
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE:        if (start) next_state = S_LOAD_REQ;
            S_LOAD_REQ:    next_state = S_LOAD_WAIT;
            S_LOAD_WAIT: begin
                if (ir_error)     next_state = S_ERROR;
                else if (ir_done) next_state = S_SCAN_CLR;
            end
            S_SCAN_CLR:    next_state = S_SCAN;
            S_SCAN:        if (col_cnt == LAST_COL) next_state = S_FLUSH;
            S_FLUSH:       if (flush_cnt == FLUSH_LAST) next_state = S_ROW_END;
            S_ROW_END: begin
                if (center_y == LAST_CENTER) next_state = S_DONE;
                else                         next_state = S_REFILL_REQ;
            end
            S_REFILL_REQ:  next_state = S_REFILL_WAIT;
            S_REFILL_WAIT: begin
                if (ir_error)     next_state = S_ERROR;
                else if (ir_done) next_state = S_SCAN_CLR;
            end
            S_DONE:        next_state = S_IDLE;
            S_ERROR:       next_state = S_IDLE;
            default:       next_state = S_IDLE;
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
    // Datapath: hang tam, base xoay vong, bo dem cot
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            center_y  <= FIRST_CENTER;
            base      <= 3'd0;
            col_cnt   <= 10'd0;
            flush_cnt <= 3'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        center_y <= FIRST_CENTER;
                        base     <= 3'd0;
                    end
                end
                S_SCAN_CLR: begin
                    col_cnt <= 10'd0;
                end
                S_SCAN: begin
                    col_cnt   <= col_cnt + 10'd1;
                    flush_cnt <= 3'd0;
                end
                S_FLUSH: begin
                    flush_cnt <= flush_cnt + 3'd1;
                end
                S_REFILL_WAIT: begin
                    if (ir_done) begin
                        center_y <= center_y + 9'd1;
                        if (base == 3'd6) base <= 3'd0;
                        else              base <= base + 3'd1;
                    end
                end
                default: ;
            endcase
        end
    end

    // ======================================================
    // FSM phan 3: combinational output logic
    // ======================================================
    assign ir_start      = (state == S_LOAD_REQ) || (state == S_REFILL_REQ);
    assign ir_first_row  = (state == S_LOAD_REQ) ? 9'd0 : (center_y + 9'd4);
    assign ir_first_bank = (state == S_LOAD_REQ) ? 5'd0 : {2'b00, base};
    assign ir_num_rows   = (state == S_LOAD_REQ) ? 5'd7 : 5'd1;

    assign win_clear     = (state == S_SCAN_CLR);
    assign lb_rd_en      = (state == S_SCAN);
    assign lb_rd_col     = col_cnt;
    assign win_in_valid  = lb_rd_valid;

    assign row_done      = (state == S_ROW_END);
    assign frame_done    = (state == S_DONE);
    assign error         = (state == S_ERROR);
    assign busy          = (state != S_IDLE);

    // ======================================================
    // Mux xoay hang: hang logic i = bank vat ly (base + i) mod 7
    // ======================================================
    always @(*) begin
        case (base)
            3'd0: begin
                win_p0 = lb_p0; win_p1 = lb_p1; win_p2 = lb_p2; win_p3 = lb_p3;
                win_p4 = lb_p4; win_p5 = lb_p5; win_p6 = lb_p6;
            end
            3'd1: begin
                win_p0 = lb_p1; win_p1 = lb_p2; win_p2 = lb_p3; win_p3 = lb_p4;
                win_p4 = lb_p5; win_p5 = lb_p6; win_p6 = lb_p0;
            end
            3'd2: begin
                win_p0 = lb_p2; win_p1 = lb_p3; win_p2 = lb_p4; win_p3 = lb_p5;
                win_p4 = lb_p6; win_p5 = lb_p0; win_p6 = lb_p1;
            end
            3'd3: begin
                win_p0 = lb_p3; win_p1 = lb_p4; win_p2 = lb_p5; win_p3 = lb_p6;
                win_p4 = lb_p0; win_p5 = lb_p1; win_p6 = lb_p2;
            end
            3'd4: begin
                win_p0 = lb_p4; win_p1 = lb_p5; win_p2 = lb_p6; win_p3 = lb_p0;
                win_p4 = lb_p1; win_p5 = lb_p2; win_p6 = lb_p3;
            end
            3'd5: begin
                win_p0 = lb_p5; win_p1 = lb_p6; win_p2 = lb_p0; win_p3 = lb_p1;
                win_p4 = lb_p2; win_p5 = lb_p3; win_p6 = lb_p4;
            end
            3'd6: begin
                win_p0 = lb_p6; win_p1 = lb_p0; win_p2 = lb_p1; win_p3 = lb_p2;
                win_p4 = lb_p3; win_p5 = lb_p4; win_p6 = lb_p5;
            end
            default: begin
                win_p0 = lb_p0; win_p1 = lb_p1; win_p2 = lb_p2; win_p3 = lb_p3;
                win_p4 = lb_p4; win_p5 = lb_p5; win_p6 = lb_p6;
            end
        endcase
    end

endmodule