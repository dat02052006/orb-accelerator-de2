//============================================================================
// image_dma.v
// DMA sao chep transfer_length byte tu src_addr sang dst_addr trong SRAM,
// di qua sram_controller (1 cong). Moi byte: DOC (re) -> chot -> GHI (we).
// CPU chi can cau hinh src_addr/dst_addr/transfer_length roi tao xung start.
//
// Giao tiep voi CPU / top:
//   start            : xung 1 chu ky (chi nhan khi khong busy)
//   src_addr         : dia chi byte nguon
//   dst_addr         : dia chi byte dich
//   transfer_length  : so byte can chuyen (1 .. MEM_SIZE)
//   busy             : 1 trong luc dang chuyen
//   done             : 1 khi chuyen xong, giu den khi co start moi
//   error            : 1 khi loi, giu den khi co start moi
//
// Loi (error) khi:
//   - transfer_length = 0
//   - src_addr + length hoac dst_addr + length vuot MEM_SIZE
//   - vung nguon va dich chong lan theo kieu copy tien se hong du lieu
//     (dst nam trong (src, src+length))
//   - SRAM khong phan hoi sau TIMEOUT chu ky
//
// Giao tiep voi sram_controller (giong image_loader):
//   sram_re    : giu 1 cho den khi sram_valid = 1 (rd_data hop le)
//   sram_we    : giu 1 cho den khi sram_ready = 1 (ghi xong)
//
// Luu y o top: image_loader va image_dma cung dung 1 cong SRAM, can mux
// theo busy cua DMA va khong chay dong thoi.
//============================================================================
module image_dma #(
    parameter ADDR_W   = 19,
    parameter LEN_W    = 20,
    parameter MEM_SIZE = 524288,     // 2^19 byte dia chi; chinh theo SRAM thuc te
    parameter TIMEOUT  = 1024        // so chu ky toi da cho 1 giao dich SRAM
)(
    input  wire              clk,
    input  wire              rst_n,

    // ---- phia CPU ----
    input  wire              start,
    input  wire [ADDR_W-1:0] src_addr,
    input  wire [ADDR_W-1:0] dst_addr,
    input  wire [LEN_W-1:0]  transfer_length,
    output reg               busy,
    output reg               done,
    output reg               error,

    // ---- phia sram_controller ----
    output wire [ADDR_W-1:0] sram_addr,
    output wire [7:0]        sram_wr_data,
    input  wire [7:0]        sram_rd_data,
    output reg               sram_re,
    output reg               sram_we,
    input  wire              sram_ready,
    input  wire              sram_valid
);

    //------------------------------------------------------------------
    // Trang thai
    //------------------------------------------------------------------
    localparam [2:0] S_IDLE  = 3'd0,
                     S_CHECK = 3'd1,    // kiem tra tham so
                     S_READ  = 3'd2,    // doc 1 byte tu src
                     S_WRITE = 3'd3,    // ghi 1 byte vao dst
                     S_NEXT  = 3'd4,    // tang dia chi, giam so byte con lai
                     S_DONE  = 3'd5,
                     S_ERROR = 3'd6;

    reg  [2:0]        state, next_state;

    // Datapath
    reg  [ADDR_W-1:0] src_cnt;
    reg  [ADDR_W-1:0] dst_cnt;
    reg  [LEN_W-1:0]  remain;
    reg  [7:0]        data_reg;
    reg  [15:0]       wait_cnt;

    // Tin hieu dieu khien tu output logic
    reg               load_cfg;
    reg               step;
    reg               load_data;
    reg               sel_dst;
    reg               cnt_en;
    reg               cnt_clr;

    //------------------------------------------------------------------
    // Cac dieu kien kiem tra
    //------------------------------------------------------------------
    wire [ADDR_W+1:0] src_end = src_cnt + remain;
    wire [ADDR_W+1:0] dst_end = dst_cnt + remain;

    wire len_zero    = (remain == 0);
    wire range_bad   = (src_end > MEM_SIZE) || (dst_end > MEM_SIZE);
    wire overlap_bad = (dst_cnt > src_cnt) && (dst_cnt < src_end);
    wire chk_fail    = len_zero | range_bad | overlap_bad;

    wire remain_last = (remain == 1);
    wire timeout     = (wait_cnt == TIMEOUT);

    assign sram_addr    = sel_dst ? dst_cnt : src_cnt;
    assign sram_wr_data = data_reg;

    //------------------------------------------------------------------
    // FSM phan 1: next-state transition logic
    //------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start) next_state = S_CHECK;
            end

            S_CHECK: begin
                if (chk_fail) next_state = S_ERROR;
                else          next_state = S_READ;
            end

            S_READ: begin
                if (sram_valid)    next_state = S_WRITE;
                else if (timeout)  next_state = S_ERROR;
            end

            S_WRITE: begin
                if (sram_ready)    next_state = S_NEXT;
                else if (timeout)  next_state = S_ERROR;
            end

            S_NEXT: begin
                if (remain_last) next_state = S_DONE;
                else             next_state = S_READ;
            end

            S_DONE: begin
                if (start) next_state = S_CHECK;
            end

            S_ERROR: begin
                if (start) next_state = S_CHECK;
            end

            default: next_state = S_IDLE;
        endcase
    end

    //------------------------------------------------------------------
    // FSM phan 2: state register update
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    //------------------------------------------------------------------
    // FSM phan 3: combinational output logic
    //------------------------------------------------------------------
    always @(*) begin
        busy      = 1'b0;
        done      = 1'b0;
        error     = 1'b0;
        sram_re   = 1'b0;
        sram_we   = 1'b0;
        load_cfg  = 1'b0;
        step      = 1'b0;
        load_data = 1'b0;
        sel_dst   = 1'b0;
        cnt_en    = 1'b0;
        cnt_clr   = 1'b0;

        case (state)
            S_IDLE: begin
                load_cfg = start;
                cnt_clr  = 1'b1;
            end

            S_CHECK: begin
                busy    = 1'b1;
                cnt_clr = 1'b1;
            end

            S_READ: begin
                busy      = 1'b1;
                sram_re   = 1'b1;
                load_data = sram_valid;
                cnt_en    = 1'b1;
                cnt_clr   = sram_valid;     // bat dau dem lai cho phase ghi
            end

            S_WRITE: begin
                busy    = 1'b1;
                sram_we = 1'b1;
                sel_dst = 1'b1;
                cnt_en  = 1'b1;
            end

            S_NEXT: begin
                busy    = 1'b1;
                step    = 1'b1;
                cnt_clr = 1'b1;
            end

            S_DONE: begin
                done     = 1'b1;
                load_cfg = start;
                cnt_clr  = 1'b1;
            end

            S_ERROR: begin
                error    = 1'b1;
                load_cfg = start;
                cnt_clr  = 1'b1;
            end

            default: begin
                cnt_clr = 1'b1;
            end
        endcase
    end

    //------------------------------------------------------------------
    // Datapath registers
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            src_cnt  <= {ADDR_W{1'b0}};
            dst_cnt  <= {ADDR_W{1'b0}};
            remain   <= {LEN_W{1'b0}};
            data_reg <= 8'd0;
            wait_cnt <= 16'd0;
        end
        else begin
            if (load_cfg) begin
                src_cnt <= src_addr;
                dst_cnt <= dst_addr;
                remain  <= transfer_length;
            end
            else if (step) begin
                src_cnt <= src_cnt + 1'b1;
                dst_cnt <= dst_cnt + 1'b1;
                remain  <= remain - 1'b1;
            end

            if (load_data) data_reg <= sram_rd_data;

            if (cnt_clr)     wait_cnt <= 16'd0;
            else if (cnt_en) wait_cnt <= wait_cnt + 1'b1;
        end
    end

endmodule
