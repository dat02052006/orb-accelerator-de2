module image_reader #(
    parameter NUM_BANKS = 7             // so bank (hang vat ly) cua Line Buffer
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- dieu khien ----
    input  wire        start,           // xung bat dau
    input  wire [8:0]  first_row,       // hang anh dau tien can doc (0..479)
    input  wire [4:0]  num_rows,        // so hang can nap (1 khi refill, 7 khi nap ban dau)
    input  wire [4:0]  first_bank,      // bank Line Buffer nhan hang dau tien
    output wire        busy,
    output wire        done,            // xung 1 chu ky khi doc du
    output wire        error,           // xung 1 chu ky neu tham so khong hop le:
                                        //   num_rows = 0, first_row + num_rows > 480,
                                        //   hoac first_bank + num_rows > NUM_BANKS

    // ---- giao tiep sram_controller ----
    output wire [17:0] sram_addr,       // dia chi word
    output wire        sram_re,
    input  wire [15:0] sram_rd_data,
    input  wire        sram_valid,

    // ---- giao tiep Line Buffer ----
    output wire        lb_we,
    output wire [4:0]  lb_row,          // bank dich trong Line Buffer
    output wire [8:0]  lb_col_word,     // chi so word trong hang (0..319), pixel x = 2*col, 2*col+1
    output wire [15:0] lb_wdata         // [7:0] = pixel cot chan, [15:8] = pixel cot le
);

    localparam [8:0] LAST_COL_WORD = 9'd319;       // 640/2 - 1
    localparam [9:0] IMG_HEIGHT    = 10'd480;
    localparam [5:0] BANKS         = NUM_BANKS;

    // ---------------- state encoding ----------------
    localparam [2:0] S_IDLE  = 3'd0,
                     S_INIT  = 3'd1,
                     S_READ  = 3'd2,
                     S_STORE = 3'd3,
                     S_DONE  = 3'd4,
                     S_ERROR = 3'd5;

    reg [2:0] state, next_state;

    // ---------------- datapath registers ----------------
    reg [4:0]  num_rows_r;      // so hang can nap (chot luc start)
    reg [4:0]  first_bank_r;    // bank dau tien (chot luc start)
    reg [17:0] row_base;        // = (first_row + row_cnt) * 320
    reg [8:0]  col_cnt;         // 0..319
    reg [4:0]  row_cnt;         // 0..num_rows-1
    reg [15:0] data_r;          // word doc duoc tu SRAM

    wire last_col = (col_cnt == LAST_COL_WORD);
    wire last_row = (row_cnt == (num_rows_r - 5'd1));

    // kiem tra tham so tai thoi diem start
    wire rows_ok  = (num_rows != 5'd0);
    wire img_ok   = ({1'b0, first_row}  + {5'd0, num_rows}) <= IMG_HEIGHT;
    wire bank_ok  = ({1'b0, first_bank} + {1'b0, num_rows}) <= BANKS;
    wire params_ok = rows_ok && img_ok && bank_ok;

    // first_row * 320 = first_row*256 + first_row*64
    wire [17:0] first_row_base = ({9'd0, first_row} << 8) + ({9'd0, first_row} << 6);

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
                if (last_col && last_row) next_state = S_DONE;
                else                      next_state = S_READ;
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
    // Datapath: dem cot/hang, tinh dia chi, chot du lieu
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            num_rows_r   <= 5'd0;
            first_bank_r <= 5'd0;
            row_base     <= 18'd0;
            col_cnt      <= 9'd0;
            row_cnt      <= 5'd0;
            data_r       <= 16'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        num_rows_r   <= num_rows;
                        first_bank_r <= first_bank;
                    end
                    // row_base nap luon tu first_row de INIT chi can reset bo dem
                    row_base <= first_row_base;
                end
                S_INIT: begin
                    col_cnt <= 9'd0;
                    row_cnt <= 5'd0;
                end
                S_READ: begin
                    if (sram_valid) data_r <= sram_rd_data;
                end
                S_STORE: begin
                    if (last_col) begin
                        col_cnt  <= 9'd0;
                        row_cnt  <= row_cnt + 5'd1;
                        row_base <= row_base + 18'd320;
                    end else begin
                        col_cnt  <= col_cnt + 9'd1;
                    end
                end
                default: ;
            endcase
        end
    end

    // ======================================================
    // FSM phan 3: combinational output logic
    // ======================================================
    assign sram_re     = (state == S_READ);
    assign sram_addr   = row_base + {9'd0, col_cnt};

    assign lb_we       = (state == S_STORE);
    assign lb_row      = first_bank_r + row_cnt;   // bank dich = bank dau + hang dang nap
    assign lb_col_word = col_cnt;
    assign lb_wdata    = data_r;         // [7:0] pixel chan, [15:8] pixel le

    assign busy        = (state != S_IDLE);
    assign done        = (state == S_DONE);
    assign error       = (state == S_ERROR);

endmodule