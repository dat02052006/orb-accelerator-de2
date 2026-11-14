//============================================================================
// top_loader_system.v
// Top tích hợp: loader + dma + reader + line_buffer + ctrl + window_gen
//              + sram_controller + arbiter
//
// Lưu ý quan trọng:
//   - window_flat (392 bit) và 49 pixel w00..w66 KHÔNG xuất ra chân vật lý
//     (tiết kiệm I/O). Chúng được giữ bằng _unused để Quartus không
//     optimize mất orb_window_generator.
//   - DMA hiện đọc/ghi byte nhưng controller 16-bit. Chưa sửa adapter
//     read-modify-write → tạm khóa DMA write để tránh hỏng dữ liệu.
//     (Bật lại khi sửa image_dma sang 16-bit hoặc thêm RMW adapter.)
//============================================================================
module top_loader_system #(
    parameter IMG_W    = 640,
    parameter IMG_H    = 480,
    parameter ADDR_W   = 19,          // byte address
    parameter WORD_W   = 18,          // word address (SRAM 256K x 16)
    parameter LEN_W    = 20,
    parameter MEM_SIZE = 524288,
    parameter TIMEOUT  = 1024
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---------- Nios II: nạp ảnh ----------
    input  wire        ld_start,
    input  wire [9:0]  ld_img_w,
    input  wire [9:0]  ld_img_h,
    input  wire        ld_pix_valid,
    input  wire [7:0]  ld_pix_data,
    output wire        ld_pix_ready,
    output wire        ld_image_loaded,
    output wire        ld_size_error,

    // ---------- Nios II: DMA ----------
    input  wire        dma_start,
    input  wire [ADDR_W-1:0] dma_src,
    input  wire [ADDR_W-1:0] dma_dst,
    input  wire [LEN_W-1:0]  dma_len,
    output wire        dma_busy,
    output wire        dma_done,
    output wire        dma_error,

    // ---------- Nios II: điều khiển xử lý ORB ----------
    input  wire        orb_start,
    output wire        orb_busy,
    output wire        orb_frame_done,
    output wire        orb_error,
    output wire        orb_row_done,
    output wire [8:0]  orb_center_y,

    // ---------- Ra cửa sổ (chỉ tín hiệu quan sát) ----------
    output wire        win_valid,
    output wire [9:0]  win_center_x,
    output wire [7:0]  dbg_w33,       // pixel tâm cửa sổ (debug)
    input  wire        win_ready,     // chưa dùng, giữ cho tương lai

    // ---------- SRAM vật lý (DE2) ----------
    output wire [WORD_W-1:0] SRAM_ADDR,
    inout  wire [15:0]       SRAM_DQ,
    output wire              SRAM_CE_N,
    output wire              SRAM_OE_N,
    output wire              SRAM_WE_N,
    output wire              SRAM_UB_N,
    output wire              SRAM_LB_N
);

    //==================================================================
    // 1. Tín hiệu nội bộ
    //==================================================================
    // Loader
    wire [ADDR_W-1:0] ld_addr;
    wire [7:0]        ld_wdata;
    wire              ld_we, ld_ready;

    // Reader
    wire [WORD_W-1:0] rd_addr;
    wire              rd_re, rd_valid;
    wire [15:0]       rd_rdata;
    wire              rd_busy;
    wire              rd_lb_we;
    wire [4:0]        rd_lb_row;
    wire [8:0]        rd_lb_col_word;
    wire [15:0]       rd_lb_wdata;

    // DMA
    wire [ADDR_W-1:0] dma_addr;
    wire [7:0]        dma_wdata;
    wire [7:0]        dma_rdata;
    wire              dma_re, dma_we, dma_ready, dma_valid;

    // Arbiter → controller
    wire [WORD_W-1:0] ctrl_addr;
    wire [15:0]       ctrl_wdata;
    wire              ctrl_we, ctrl_re;
    wire [15:0]       ctrl_rdata;
    wire              ctrl_ready, ctrl_valid;

    // Ctrl ↔ reader
    wire        ir_start;
    wire [8:0]  ir_first_row;
    wire [4:0]  ir_first_bank;
    wire [4:0]  ir_num_rows;
    wire        ir_done, ir_error;

    // Ctrl ↔ line buffer (đọc)
    wire        lb_rd_en;
    wire [9:0]  lb_rd_col;
    wire        lb_rd_valid;
    wire [7:0]  lb_p0, lb_p1, lb_p2, lb_p3, lb_p4, lb_p5, lb_p6;

    // Ctrl → window generator
    wire        win_clear;
    wire        win_in_valid;
    wire [7:0]  win_p0, win_p1, win_p2, win_p3, win_p4, win_p5, win_p6;

    // Window generator nội bộ
    wire [391:0] window_flat_int;

    //==================================================================
    // 2. Adapter loader 8→16 bit (gom 2 pixel thành 1 word)
    //    Giữ ld_word_valid đến khi controller nhận (ld_ready).
    //==================================================================
    reg              ld_phase;
    reg [7:0]        ld_byte_lo;
    reg [15:0]       ld_word;
    reg              ld_word_valid;
    reg [WORD_W-1:0] ld_word_addr;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ld_phase      <= 1'b0;
            ld_byte_lo    <= 8'd0;
            ld_word       <= 16'd0;
            ld_word_valid <= 1'b0;
            ld_word_addr  <= {WORD_W{1'b0}};
        end else begin
            // Clear khi controller đã nhận word
            if (ld_word_valid && ld_ready)
                ld_word_valid <= 1'b0;

            if (ld_we && !ld_word_valid) begin
                if (!ld_phase) begin
                    // Nhận byte thấp
                    ld_byte_lo   <= ld_wdata;
                    ld_word_addr <= ld_addr[ADDR_W-1:1];
                    ld_phase     <= 1'b1;
                end else begin
                    // Nhận byte cao → đủ word
                    ld_word       <= {ld_wdata, ld_byte_lo};
                    ld_word_valid <= 1'b1;
                    ld_phase      <= 1'b0;
                end
            end
        end
    end

    //==================================================================
    // 3. Arbiter SRAM: reader > dma > loader
    //==================================================================
    localparam OWN_IDLE = 2'd0,
               OWN_LD   = 2'd1,
               OWN_RD   = 2'd2,
               OWN_DMA  = 2'd3;

    reg [1:0] owner;

    wire rd_req  = rd_re;
    wire dma_req = dma_busy;
    wire ld_req  = ld_word_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) owner <= OWN_IDLE;
        else begin
            case (owner)
                OWN_IDLE: begin
                    if (rd_req)       owner <= OWN_RD;
                    else if (dma_req) owner <= OWN_DMA;
                    else if (ld_req)  owner <= OWN_LD;
                end
                OWN_LD:  if (!ld_req)  owner <= OWN_IDLE;
                OWN_RD:  if (!rd_req)  owner <= OWN_IDLE;
                OWN_DMA: if (!dma_req) owner <= OWN_IDLE;
            endcase
        end
    end

    // Mux tới controller
    assign ctrl_addr  = (owner == OWN_LD)  ? ld_word_addr :
                        (owner == OWN_RD)  ? rd_addr :
                        (owner == OWN_DMA) ? dma_addr[ADDR_W-1:1] :
                                             {WORD_W{1'b0}};

    // TẠM KHÓA DMA WRITE: vì {dma_wdata,dma_wdata} sẽ phá byte cao.
    // Khi sửa image_dma sang 16-bit hoặc thêm RMW, đổi lại như comment.
    assign ctrl_wdata = (owner == OWN_LD)  ? ld_word :
                        // (owner == OWN_DMA) ? dma_word :   // khi có RMW
                                            16'd0;

    assign ctrl_we    = (owner == OWN_LD)  ? ld_word_valid :
                        // (owner == OWN_DMA) ? dma_we :     // khi có RMW
                                             1'b0;

    assign ctrl_re    = (owner == OWN_RD)  ? rd_re :
                        (owner == OWN_DMA) ? dma_re :
                                             1'b0;

    // Feedback
    assign ld_ready  = (owner == OWN_LD)  ? ctrl_ready : 1'b0;
    assign rd_valid  = (owner == OWN_RD)  ? ctrl_valid : 1'b0;
    assign rd_rdata  = ctrl_rdata;
    assign dma_ready = (owner == OWN_DMA) ? ctrl_ready : 1'b0;
    assign dma_valid = (owner == OWN_DMA) ? ctrl_valid : 1'b0;
    assign dma_rdata = ctrl_rdata[7:0];

    //==================================================================
    // 4. Instantiate các module
    //==================================================================
    image_loader #(
        .IMG_WIDTH (IMG_W),
        .IMG_HEIGHT(IMG_H),
        .ADDR_W    (ADDR_W)
    ) u_loader (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (ld_start),
        .img_width   (ld_img_w),
        .img_height  (ld_img_h),
        .pixel_valid (ld_pix_valid),
        .pixel_data  (ld_pix_data),
        .pixel_ready (ld_pix_ready),
        .sram_addr   (ld_addr),
        .sram_wr_data(ld_wdata),
        .sram_we     (ld_we),
        .sram_ready  (ld_ready),
        .image_loaded(ld_image_loaded),
        .size_error  (ld_size_error)
    );

    image_dma #(
        .ADDR_W   (ADDR_W),
        .LEN_W    (LEN_W),
        .MEM_SIZE (MEM_SIZE),
        .TIMEOUT  (TIMEOUT)
    ) u_dma (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (dma_start),
        .src_addr        (dma_src),
        .dst_addr        (dma_dst),
        .transfer_length (dma_len),
        .busy            (dma_busy),
        .done            (dma_done),
        .error           (dma_error),
        .sram_addr       (dma_addr),
        .sram_wr_data    (dma_wdata),
        .sram_rd_data    (dma_rdata),
        .sram_re         (dma_re),
        .sram_we         (dma_we),
        .sram_ready      (dma_ready),
        .sram_valid      (dma_valid)
    );

    image_reader #(
        .NUM_BANKS (7)
    ) u_reader (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (ir_start),
        .first_row    (ir_first_row),
        .num_rows     (ir_num_rows),
        .first_bank   (ir_first_bank),
        .busy         (rd_busy),
        .done         (ir_done),
        .error        (ir_error),
        .sram_addr    (rd_addr),
        .sram_re      (rd_re),
        .sram_rd_data (rd_rdata),
        .sram_valid   (rd_valid),
        .lb_we        (rd_lb_we),
        .lb_row       (rd_lb_row),
        .lb_col_word  (rd_lb_col_word),
        .lb_wdata     (rd_lb_wdata)
    );

    orb_line_buffer u_lb (
        .clk         (clk),
        // ghi từ reader
        .lb_we       (rd_lb_we),
        .lb_row      (rd_lb_row),
        .lb_col_word (rd_lb_col_word),
        .lb_wdata    (rd_lb_wdata),
        // đọc cho ctrl
        .rd_en       (lb_rd_en),
        .rd_col      (lb_rd_col),
        .rd_valid    (lb_rd_valid),
        .rd_p0       (lb_p0),
        .rd_p1       (lb_p1),
        .rd_p2       (lb_p2),
        .rd_p3       (lb_p3),
        .rd_p4       (lb_p4),
        .rd_p5       (lb_p5),
        .rd_p6       (lb_p6)
    );

    orb_line_buffer_ctrl u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (orb_start),
        .busy           (orb_busy),
        .frame_done     (orb_frame_done),
        .error          (orb_error),
        .row_done       (orb_row_done),
        .center_y       (orb_center_y),
        // sang reader
        .ir_start       (ir_start),
        .ir_first_row   (ir_first_row),
        .ir_first_bank  (ir_first_bank),
        .ir_num_rows    (ir_num_rows),
        .ir_done        (ir_done),
        .ir_error       (ir_error),
        // đọc line buffer
        .lb_rd_en       (lb_rd_en),
        .lb_rd_col      (lb_rd_col),
        .lb_rd_valid    (lb_rd_valid),
        .lb_p0          (lb_p0),
        .lb_p1          (lb_p1),
        .lb_p2          (lb_p2),
        .lb_p3          (lb_p3),
        .lb_p4          (lb_p4),
        .lb_p5          (lb_p5),
        .lb_p6          (lb_p6),
        // sang window generator
        .win_clear      (win_clear),
        .win_in_valid   (win_in_valid),
        .win_p0         (win_p0),
        .win_p1         (win_p1),
        .win_p2         (win_p2),
        .win_p3         (win_p3),
        .win_p4         (win_p4),
        .win_p5         (win_p5),
        .win_p6         (win_p6)
    );

    orb_window_generator u_win (
        .clk           (clk),
        .rst_n         (rst_n),
        .clear         (win_clear),
        .in_valid      (win_in_valid),
        .in_p0         (win_p0),
        .in_p1         (win_p1),
        .in_p2         (win_p2),
        .in_p3         (win_p3),
        .in_p4         (win_p4),
        .in_p5         (win_p5),
        .in_p6         (win_p6),
        .window_valid  (win_valid),
        .win_center_x  (win_center_x),
        .window_flat   (window_flat_int),
        // 49 output w00..w66: để trống, window_flat vẫn dùng chúng
        // (Quartus giữ nguyên nhờ window_flat_int được dùng trong _unused)
        .w00(), .w01(), .w02(), .w03(), .w04(), .w05(), .w06(),
        .w10(), .w11(), .w12(), .w13(), .w14(), .w15(), .w16(),
        .w20(), .w21(), .w22(), .w23(), .w24(), .w25(), .w26(),
        .w30(), .w31(), .w32(), .w33(dbg_w33),   // chỉ xuất tâm để debug
        .w34(), .w35(), .w36(),
        .w40(), .w41(), .w42(), .w43(), .w44(), .w45(), .w46(),
        .w50(), .w51(), .w52(), .w53(), .w54(), .w55(), .w56(),
        .w60(), .w61(), .w62(), .w63(), .w64(), .w65(), .w66()
    );

    sram_controller #(
        .ADDR_W       (WORD_W),
        .DATA_W       (16),
        .WR_PULSE_CYC (1),
        .RD_ACC_CYC   (1)
    ) u_sram (
        .clk        (clk),
        .rst_n      (rst_n),
        .addr       (ctrl_addr),
        .wr_data    (ctrl_wdata),
        .we         (ctrl_we),
        .re         (ctrl_re),
        .rd_data    (ctrl_rdata),
        .ready      (ctrl_ready),
        .valid      (ctrl_valid),
        .done       (),
        .sram_addr  (SRAM_ADDR),
        .sram_dq    (SRAM_DQ),
        .sram_ce_n  (SRAM_CE_N),
        .sram_oe_n  (SRAM_OE_N),
        .sram_we_n  (SRAM_WE_N),
        .sram_ub_n  (SRAM_UB_N),
        .sram_lb_n  (SRAM_LB_N)
    );

    //==================================================================
    // 5. Giữ tín hiệu khỏi bị optimize
    //    window_flat_int và dbg_w33 phải được "dùng" để Quartus
    //    không xóa 49 thanh ghi dịch của orb_window_generator.
    //    win_ready hiện chưa dùng — đưa vào _unused để tránh warning.
    //==================================================================
    wire _unused = &{1'b0,
                     window_flat_int,
                     rd_busy,
                     win_ready,
                     1'b0};

endmodule