// ============================================================================
// sram_controller.v
// SRAM controller cho IS61LV25616AL-10 (256K x 16) tren DE2 (Cyclone II)
// Giao dien logic: addr, wr_data, rd_data, we, re, ready, valid (+ done)
// FSM 3 phan: (1) cap nhat trang thai, (2) logic chuyen trang thai,
//             (3) to hop logic output (Moore)
//
// Cau hinh SRAM tren DE2: SRAM_ADDR[17:0], SRAM_DQ[15:0] (inout),
//   SRAM_CE_N, SRAM_OE_N, SRAM_WE_N, SRAM_UB_N, SRAM_LB_N
// Timing (-10): tAA = 10 ns, tPWE >= 8 ns, tSD >= 6 ns, tWC/tRC = 10 ns
// Clock mac dinh 50 MHz (20 ns) -> WR_PULSE_CYC = RD_ACC_CYC = 1
// ============================================================================
module sram_controller #(
    parameter ADDR_W        = 18,   // DE2: 18 bit dia chi (256K tu)
    parameter DATA_W        = 16,   // DE2: 16 bit du lieu
    parameter WR_PULSE_CYC  = 1,    // so chu ky WE_N = 0   (>= tPWE / Tclk)
    parameter RD_ACC_CYC    = 1     // so chu ky cho tAA    (>= tAA  / Tclk)
)(
    input                   clk,
    input                   rst_n,

    // Giao dien logic
    input  [ADDR_W-1:0]     addr,
    input  [DATA_W-1:0]     wr_data,
    input                   we,
    input                   re,
    output [DATA_W-1:0]     rd_data,
    output                  ready,      // 1: san sang nhan yeu cau moi
    output                  valid,      // 1 chu ky: rd_data hop le
    output                  done,       // 1 chu ky: giao dich (doc/ghi) hoan thanh

    // Chan SRAM vat ly
    output [ADDR_W-1:0]     sram_addr,
    inout  [DATA_W-1:0]     sram_dq,
    output                  sram_ce_n,
    output                  sram_oe_n,
    output                  sram_we_n,
    output                  sram_ub_n,
    output                  sram_lb_n
);

    // ------------------------------------------------------------------
    // Ma hoa trang thai: one-hot (output giai ma sach, han che glitch
    // tren WE_N/OE_N/CE_N ra chan vat ly)
    // ------------------------------------------------------------------
    localparam [6:0] S_IDLE      = 7'b0000001,
                     S_WR_SETUP  = 7'b0000010,  // dua addr/data, WE_N = 1
                     S_WR_PULSE  = 7'b0000100,  // WE_N = 0
                     S_WR_HOLD   = 7'b0001000,  // WE_N = 1, giu addr/data, bao done
                     S_RD_ACCESS = 7'b0010000,  // OE_N = 0, cho tAA
                     S_RD_LATCH  = 7'b0100000,  // chot SRAM_DQ vao rd_data
                     S_RD_DONE   = 7'b1000000;  // valid = 1, done = 1

    reg [6:0]        state, next_state;
    reg [3:0]        wait_cnt;
    reg [ADDR_W-1:0] addr_reg;
    reg [DATA_W-1:0] wr_data_reg;
    reg [DATA_W-1:0] rd_data_reg;

    // Tin hieu to hop tu state (output logic)
    reg              o_ready;
    reg              o_valid;
    reg              o_done;
    reg              o_ce_n;
    reg              o_oe_n;
    reg              o_we_n;
    reg              o_dq_oe;      // 1: FPGA lai bus DQ
    reg              o_rd_capture; // 1: chot du lieu doc
    reg              o_req_latch;  // 1: chot addr/wr_data tu yeu cau

    wire wr_cnt_end;
    wire rd_cnt_end;
    assign wr_cnt_end = (wait_cnt >= (WR_PULSE_CYC - 1));
    assign rd_cnt_end = (wait_cnt >= (RD_ACC_CYC   - 1));

    // ==================================================================
    // FSM PHAN 1: cap nhat trang thai (sequential)
    // ==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // ==================================================================
    // FSM PHAN 2: logic chuyen trang thai (combinational)
    // Neu we va re cung len, uu tien ghi.
    // ==================================================================
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (we)
                    next_state = S_WR_SETUP;
                else if (re)
                    next_state = S_RD_ACCESS;
                else
                    next_state = S_IDLE;
            end

            S_WR_SETUP: begin
                next_state = S_WR_PULSE;
            end

            S_WR_PULSE: begin
                if (wr_cnt_end)
                    next_state = S_WR_HOLD;
                else
                    next_state = S_WR_PULSE;
            end

            S_WR_HOLD: begin
                next_state = S_IDLE;
            end

            S_RD_ACCESS: begin
                if (rd_cnt_end)
                    next_state = S_RD_LATCH;
                else
                    next_state = S_RD_ACCESS;
            end

            S_RD_LATCH: begin
                next_state = S_RD_DONE;
            end

            S_RD_DONE: begin
                next_state = S_IDLE;
            end

            default: begin
                next_state = S_IDLE;
            end
        endcase
    end

    // ==================================================================
    // FSM PHAN 3: to hop logic output (Moore, chi phu thuoc state)
    // ==================================================================
    always @(*) begin
        // gia tri mac dinh
        o_ready      = 1'b0;
        o_valid      = 1'b0;
        o_done       = 1'b0;
        o_ce_n       = 1'b1;
        o_oe_n       = 1'b1;
        o_we_n       = 1'b1;
        o_dq_oe      = 1'b0;
        o_rd_capture = 1'b0;
        o_req_latch  = 1'b0;

        case (state)
            S_IDLE: begin
                o_ready     = 1'b1;
                o_req_latch = 1'b1;
            end

            S_WR_SETUP: begin
                o_ce_n  = 1'b0;
                o_dq_oe = 1'b1;
            end

            S_WR_PULSE: begin
                o_ce_n  = 1'b0;
                o_we_n  = 1'b0;
                o_dq_oe = 1'b1;
            end

            S_WR_HOLD: begin
                o_ce_n  = 1'b0;
                o_dq_oe = 1'b1;
                o_done  = 1'b1;
            end

            S_RD_ACCESS: begin
                o_ce_n = 1'b0;
                o_oe_n = 1'b0;
            end

            S_RD_LATCH: begin
                o_ce_n       = 1'b0;
                o_oe_n       = 1'b0;
                o_rd_capture = 1'b1;
            end

            S_RD_DONE: begin
                o_valid = 1'b1;
                o_done  = 1'b1;
            end

            default: begin
            end
        endcase
    end

    // ==================================================================
    // Bo dem chu ky cho (WR_PULSE / RD_ACCESS)
    // ==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wait_cnt <= 4'd0;
        else if (state == S_WR_PULSE) begin
            if (wr_cnt_end) wait_cnt <= 4'd0;
            else            wait_cnt <= wait_cnt + 4'd1;
        end
        else if (state == S_RD_ACCESS) begin
            if (rd_cnt_end) wait_cnt <= 4'd0;
            else            wait_cnt <= wait_cnt + 4'd1;
        end
        else
            wait_cnt <= 4'd0;
    end

    // ==================================================================
    // Thanh ghi chot yeu cau (addr, wr_data) - giu on dinh suot giao dich
    // ==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr_reg    <= {ADDR_W{1'b0}};
            wr_data_reg <= {DATA_W{1'b0}};
        end
        else if (o_req_latch && (we || re)) begin
            addr_reg    <= addr;
            wr_data_reg <= wr_data;
        end
    end

    // ==================================================================
    // Thanh ghi chot du lieu doc
    // ==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_data_reg <= {DATA_W{1'b0}};
        else if (o_rd_capture)
            rd_data_reg <= sram_dq;
    end

    // ==================================================================
    // Gan output
    // ==================================================================
    assign ready     = o_ready;
    assign valid     = o_valid;
    assign done      = o_done;
    assign rd_data   = rd_data_reg;

    assign sram_addr = addr_reg;
    assign sram_dq   = o_dq_oe ? wr_data_reg : {DATA_W{1'bz}};
    assign sram_ce_n = o_ce_n;
    assign sram_oe_n = o_oe_n;
    assign sram_we_n = o_we_n;
    assign sram_ub_n = 1'b0;    // truy cap ca 16 bit
    assign sram_lb_n = 1'b0;

endmodule
