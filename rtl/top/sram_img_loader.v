// -----------------------------------------------------------------------------
// sram_img_loader.v
// Nhan anh tu PC qua UART (8N1), ghep 2 byte thanh 1 word 16 bit va ghi tuan tu
// vao SRAM tu dia chi 0 den IMG_WORDS-1.
//
//  - Byte thu nhat cua moi cap = byte thap [7:0] (pixel chan),
//    byte thu hai = byte cao [15:8] (pixel le).
//  - load_en = 1 : che do nap anh (loader giu SRAM). load_en = 0 : tra SRAM
//    cho accelerator va reset bo dem dia chi.
//  - load_done = 1 khi da ghi du IMG_WORDS word; chi cho phep orb_start sau do.
//
// Giao dien ghi SRAM: wr_req duoc giu o muc 1 cho den khi wr_ack = 1 (xung 1
// chu ky khi sram_controller da ghi xong word do). Chinh lai cho khop voi port
// cua sram_controller hien co.
// -----------------------------------------------------------------------------
module sram_img_loader #(
    parameter CLK_HZ    = 50000000,
    parameter BAUD      = 115200,
    parameter ADDR_W    = 18,
    parameter IMG_WORDS = 153600          // 640 x 480 / 2
)(
    input  wire              clk,
    input  wire              rst,         // sync, active-high
    input  wire              rxd,         // UART_RXD tu PC
    input  wire              load_en,

    output wire              load_done,

    // yeu cau ghi vao sram_controller
    output wire              wr_req,
    output wire [ADDR_W-1:0] wr_addr,
    output wire [15:0]       wr_data,
    input  wire              wr_ack
);

    localparam [2:0] S_IDLE = 3'd0;
    localparam [2:0] S_LO   = 3'd1;       // cho byte thap
    localparam [2:0] S_HI   = 3'd2;       // cho byte cao
    localparam [2:0] S_WR   = 3'd3;       // ghi 1 word vao SRAM
    localparam [2:0] S_DONE = 3'd4;

    localparam [ADDR_W-1:0] LAST_ADDR = IMG_WORDS - 1;

    reg  [2:0]        state;
    reg  [2:0]        next_state;
    reg  [ADDR_W-1:0] addr_reg;
    reg  [15:0]       data_reg;

    wire [7:0] rx_data;
    wire       rx_valid;
    wire       last_word = (addr_reg == LAST_ADDR);

    uart_rx #(
        .CLK_HZ (CLK_HZ),
        .BAUD   (BAUD)
    ) u_uart_rx (
        .clk      (clk),
        .rst      (rst),
        .rxd      (rxd),
        .rx_data  (rx_data),
        .rx_valid (rx_valid)
    );

    // -------------------------------------------------------------------------
    // Phan 1: next-state logic
    // -------------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        if (~load_en) begin
            next_state = S_IDLE;
        end else begin
            case (state)
                S_IDLE: begin
                    next_state = S_LO;
                end
                S_LO: begin
                    if (rx_valid)
                        next_state = S_HI;
                end
                S_HI: begin
                    if (rx_valid)
                        next_state = S_WR;
                end
                S_WR: begin
                    if (wr_ack) begin
                        if (last_word)
                            next_state = S_DONE;
                        else
                            next_state = S_LO;
                    end
                end
                S_DONE: begin
                    next_state = S_DONE;     // giu cho den khi load_en ha xuong
                end
                default: begin
                    next_state = S_IDLE;
                end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Phan 2: state register
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // -------------------------------------------------------------------------
    // Datapath: chot byte va dem dia chi word
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            addr_reg <= {ADDR_W{1'b0}};
            data_reg <= 16'd0;
        end else begin
            if (state == S_IDLE)
                addr_reg <= {ADDR_W{1'b0}};
            if ((state == S_LO) & rx_valid)
                data_reg[7:0] <= rx_data;
            if ((state == S_HI) & rx_valid)
                data_reg[15:8] <= rx_data;
            if ((state == S_WR) & wr_ack & ~last_word)
                addr_reg <= addr_reg + {{(ADDR_W-1){1'b0}}, 1'b1};
        end
    end

    // -------------------------------------------------------------------------
    // Phan 3: output logic
    // -------------------------------------------------------------------------
    assign wr_req    = (state == S_WR);
    assign wr_addr   = addr_reg;
    assign wr_data   = data_reg;
    assign load_done = (state == S_DONE);

endmodule
