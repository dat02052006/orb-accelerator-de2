// -----------------------------------------------------------------------------
// avs_img_loader.v
// Avalon-MM slave nap anh vao SRAM, dung voi JTAG-to-Avalon Master (hoac Nios II).
// Thay the sram_img_loader.v (UART). Giao dien ghi SRAM phia sau giu nguyen:
// wr_req / wr_addr / wr_data / wr_ack  (wr_req giu cho den khi wr_ack = 1).
//
// Hai slave interface:
//  1) avs_mem_*  : cua so anh, ghi 16 bit/word, dia chi theo word (0..IMG_WORDS-1).
//                  Ghi bi chan bang waitrequest cho den khi SRAM ghi xong.
//                  Khi load_mode = 0 thi ghi bi bo qua (khong waitrequest).
//                  byteenable khong dung: phai ghi du 16 bit, dia chi byte chan.
//  2) avs_ctrl_* : thanh ghi dieu khien, zero-wait-state, read latency = 1.
//        addr 0 : [0] load_mode (R/W), [1] load_done (R/W)
//        addr 1 : word_cnt[15:0]  (R)   so word da ghi vao SRAM
//        addr 2 : word_cnt[17:16] (R)
//        addr 3 : 0
//     word_cnt duoc xoa khi load_mode chuyen 0 -> 1.
//
// FSM ghi tach 3 phan: next-state / state register / output.
// Reset: sync, active-high (trong Qsys dung reset role "reset").
// -----------------------------------------------------------------------------
module avs_img_loader #(
    parameter ADDR_W = 18
)(
    input  wire              clk,
    input  wire              rst,

    // Avalon-MM slave: cua so anh
    input  wire [ADDR_W-1:0] avs_mem_address,
    input  wire              avs_mem_write,
    input  wire [15:0]       avs_mem_writedata,
    input  wire [1:0]        avs_mem_byteenable,   // khong dung
    output wire              avs_mem_waitrequest,

    // Avalon-MM slave: thanh ghi dieu khien
    input  wire [1:0]        avs_ctrl_address,
    input  wire              avs_ctrl_write,
    input  wire [15:0]       avs_ctrl_writedata,
    input  wire              avs_ctrl_read,
    output reg  [15:0]       avs_ctrl_readdata,

    // sang sram_controller
    output wire              wr_req,
    output wire [ADDR_W-1:0] wr_addr,
    output wire [15:0]       wr_data,
    input  wire              wr_ack,

    // sang accelerator / phan quyen SRAM
    output wire              load_mode,
    output wire              load_done
);

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_REQ  = 2'd1;   // giu wr_req cho den khi wr_ack
    localparam [1:0] S_ACK  = 2'd2;   // ha waitrequest 1 chu ky de hoan tat transfer

    reg  [1:0]        state;
    reg  [1:0]        next_state;

    reg               mode_reg;
    reg               done_reg;
    reg  [ADDR_W-1:0] addr_reg;
    reg  [15:0]       data_reg;
    reg  [ADDR_W-1:0] word_cnt;

    wire mem_accept   = avs_mem_write & mode_reg;
    wire ctrl_wr_reg0 = avs_ctrl_write & (avs_ctrl_address == 2'd0);

    // -------------------------------------------------------------------------
    // Phan 1: next-state logic
    // -------------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (mem_accept)
                    next_state = S_REQ;
            end
            S_REQ: begin
                if (wr_ack)
                    next_state = S_ACK;
            end
            S_ACK: begin
                next_state = S_IDLE;
            end
            default: begin
                next_state = S_IDLE;
            end
        endcase
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
    // Thanh ghi dieu khien (load_mode, load_done)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            mode_reg <= 1'b0;
            done_reg <= 1'b0;
        end else begin
            if (ctrl_wr_reg0) begin
                mode_reg <= avs_ctrl_writedata[0];
                done_reg <= avs_ctrl_writedata[1];
            end
        end
    end

    // -------------------------------------------------------------------------
    // Datapath: chot dia chi/du lieu, dem word da ghi
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            addr_reg <= {ADDR_W{1'b0}};
            data_reg <= 16'd0;
            word_cnt <= {ADDR_W{1'b0}};
        end else begin
            if ((state == S_IDLE) & mem_accept) begin
                addr_reg <= avs_mem_address;
                data_reg <= avs_mem_writedata;
            end
            if ((state == S_REQ) & wr_ack)
                word_cnt <= word_cnt + {{(ADDR_W-1){1'b0}}, 1'b1};
            if (ctrl_wr_reg0 & avs_ctrl_writedata[0] & ~mode_reg)
                word_cnt <= {ADDR_W{1'b0}};
        end
    end

    // -------------------------------------------------------------------------
    // Doc thanh ghi dieu khien (read latency = 1)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            avs_ctrl_readdata <= 16'd0;
        end else if (avs_ctrl_read) begin
            case (avs_ctrl_address)
                2'd0:    avs_ctrl_readdata <= {14'd0, done_reg, mode_reg};
                2'd1:    avs_ctrl_readdata <= word_cnt[15:0];
                2'd2:    avs_ctrl_readdata <= {14'd0, word_cnt[17:16]};
                default: avs_ctrl_readdata <= 16'd0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Phan 3: output logic
    // -------------------------------------------------------------------------
    assign avs_mem_waitrequest = avs_mem_write & mode_reg & (state != S_ACK);

    assign wr_req    = (state == S_REQ);
    assign wr_addr   = addr_reg;
    assign wr_data   = data_reg;

    assign load_mode = mode_reg;
    assign load_done = done_reg;

endmodule
