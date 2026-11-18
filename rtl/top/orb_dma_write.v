// -----------------------------------------------------------------------------
// orb_dma_write.v
// Doc word tu result FIFO (show-ahead / FWFT) va ghi vao SDRAM qua Avalon-MM
// master, theo tung burst BURST_LEN word, dia chi tang tuan tu.
//
//  - Chi bat dau 1 burst khi FIFO da co du BURST_LEN word (khong bi stall giua
//    burst vi thieu du lieu).
//  - Khi src_done = 1 ma FIFO con le < BURST_LEN word: flush bang 1 burst, cac
//    word thieu duoc dem 0.
//  - Voi Qsys SDR SDRAM controller (khong ho tro burst that): dat BURST_LEN = 1.
//
// FSM tach 3 phan: next-state logic / state register / output logic.
// -----------------------------------------------------------------------------
module orb_dma_write #(
    parameter DATA_W    = 16,
    parameter ADDR_W    = 32,
    parameter BURST_LEN = 8,
    parameter FIFO_CW   = 10
)(
    input  wire                 clk,
    input  wire                 rst,              // sync, active-high

    // dieu khien
    input  wire                 start,            // xung 1 chu ky
    input  wire [ADDR_W-1:0]    base_addr,        // dia chi byte dau vung ket qua
    input  wire                 src_done,         // giu muc 1 khi da day het ket qua vao FIFO
    output wire                 busy,
    output wire                 done,             // xung 1 chu ky
    output wire [31:0]          words_written,    // so word da ghi (gom ca word dem 0)

    // result FIFO (show-ahead)
    input  wire [DATA_W-1:0]    fifo_rdata,
    input  wire                 fifo_empty,
    input  wire [FIFO_CW-1:0]   fifo_count,
    output wire                 fifo_rd,

    // Avalon-MM master
    output wire [ADDR_W-1:0]    avm_address,
    output wire                 avm_write,
    output wire [DATA_W-1:0]    avm_writedata,
    output wire [DATA_W/8-1:0]  avm_byteenable,
    output wire [7:0]           avm_burstcount,
    input  wire                 avm_waitrequest
);

    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_WAIT  = 2'd1;
    localparam [1:0] S_BURST = 2'd2;
    localparam [1:0] S_DONE  = 2'd3;

    localparam BURST_BYTES = BURST_LEN * (DATA_W / 8);
    localparam LAST_BEAT   = BURST_LEN - 1;

    reg  [1:0]          state;
    reg  [1:0]          next_state;

    reg  [ADDR_W-1:0]   addr_reg;
    reg  [7:0]          beat_cnt;
    reg  [31:0]         word_cnt;

    wire beat_accept      = (state == S_BURST) & ~avm_waitrequest;
    wire last_beat        = (beat_cnt == LAST_BEAT);
    wire full_burst_ready = (fifo_count >= BURST_LEN);
    wire flush_ready      = src_done & ~fifo_empty;
    wire all_drained      = src_done & fifo_empty;

    // -------------------------------------------------------------------------
    // Phan 1: next-state logic (to hop)
    // -------------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start)
                    next_state = S_WAIT;
            end
            S_WAIT: begin
                if (full_burst_ready | flush_ready)
                    next_state = S_BURST;
                else if (all_drained)
                    next_state = S_DONE;
            end
            S_BURST: begin
                if (beat_accept & last_beat)
                    next_state = S_WAIT;
            end
            S_DONE: begin
                next_state = S_IDLE;
            end
            default: begin
                next_state = S_IDLE;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // Phan 2: state register (tuan tu)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // -------------------------------------------------------------------------
    // Datapath: dia chi, dem beat, dem word (tuan tu)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            addr_reg <= {ADDR_W{1'b0}};
            beat_cnt <= 8'd0;
            word_cnt <= 32'd0;
        end else begin
            if ((state == S_IDLE) & start) begin
                addr_reg <= base_addr;
                beat_cnt <= 8'd0;
                word_cnt <= 32'd0;
            end
            if (beat_accept) begin
                word_cnt <= word_cnt + 32'd1;
                if (last_beat) begin
                    beat_cnt <= 8'd0;
                    addr_reg <= addr_reg + BURST_BYTES;
                end else begin
                    beat_cnt <= beat_cnt + 8'd1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Phan 3: output logic (to hop thuan tuy)
    // -------------------------------------------------------------------------
    assign busy           = (state != S_IDLE);
    assign done           = (state == S_DONE);
    assign words_written  = word_cnt;

    assign avm_write      = (state == S_BURST);
    assign avm_address    = addr_reg;
    assign avm_burstcount = BURST_LEN;
    assign avm_byteenable = {(DATA_W/8){1'b1}};
    assign avm_writedata  = fifo_empty ? {DATA_W{1'b0}} : fifo_rdata;

    assign fifo_rd        = beat_accept & ~fifo_empty;

endmodule
