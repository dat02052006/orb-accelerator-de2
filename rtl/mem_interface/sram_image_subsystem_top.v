// ============================================================================
// sram_image_subsystem_top.v
// Top-module ket noi:
//   Nios II -> image_loader -> loader_sram_pack --\
//                                                  +--> sram_controller <-> SRAM
//   Line Buffer <- image_reader ------------------/
// ============================================================================

module sram_image_subsystem_top #(
    parameter IMG_WIDTH    = 640,
    parameter IMG_HEIGHT   = 480,
    parameter NUM_BANKS    = 7,
    parameter WR_PULSE_CYC = 1,
    parameter RD_ACC_CYC   = 1
)(
    input  wire        clk,
    input  wire        rst_n,

    // ========================================================================
    // 1. Giao tiep phia Nios II (Avalon-ST Sink & Control cho image_loader)
    // ========================================================================
    input  wire        ld_start,
    input  wire [9:0]  ld_img_width,
    input  wire [9:0]  ld_img_height,
    input  wire        ld_pixel_valid,
    input  wire [7:0]  ld_pixel_data,
    output wire        ld_pixel_ready,
    output wire        ld_image_loaded,
    output wire        ld_size_error,

    // ========================================================================
    // 2. Giao tiep phia dieu khien doc (image_reader)
    // ========================================================================
    input  wire        rd_start,
    input  wire [8:0]  rd_first_row,
    input  wire [4:0]  rd_num_rows,
    input  wire [4:0]  rd_first_bank,
    output wire        rd_busy,
    output wire        rd_done,
    output wire        rd_error,

    // ========================================================================
    // 3. Giao tiep ngo ra toi Line Buffer
    // ========================================================================
    output wire        lb_we,
    output wire [4:0]  lb_row,
    output wire [8:0]  lb_col_word,
    output wire [15:0] lb_wdata,

    // ========================================================================
    // 4. Chan vat ly ket noi chip SRAM ngoai (IS61LV25616AL tren DE2)
    // ========================================================================
    output wire [17:0] SRAM_ADDR,
    inout  wire [15:0] SRAM_DQ,
    output wire        SRAM_CE_N,
    output wire        SRAM_OE_N,
    output wire        SRAM_WE_N,
    output wire        SRAM_UB_N,
    output wire        SRAM_LB_N
);

    // ------------------------------------------------------------------------
    // Day noi noi bo (Internal Wires)
    // ------------------------------------------------------------------------
    // image_loader <-> loader_sram_pack
    wire [18:0] ld_sram_addr_byte;
    wire [7:0]  ld_sram_wr_data;
    wire        ld_sram_we;
    wire        ld_sram_ready;

    // loader_sram_pack <-> sram_controller (Write path)
    wire [17:0] pack_sr_addr;
    wire [15:0] pack_sr_wdata;
    wire        pack_sr_we;

    // image_reader <-> sram_controller (Read path)
    wire [17:0] rd_sram_addr;
    wire        rd_sram_re;

    // Tin hieu chung tu sram_controller
    wire [17:0] ctrl_addr;
    wire [15:0] ctrl_rd_data;
    wire        ctrl_ready;
    wire        ctrl_valid;
    wire        ctrl_done; // Co the bo trong hoac dua ra debug neu can

    // ------------------------------------------------------------------------
    // Bo phan kenh dia chi vao sram_controller (Uu tien ghi giong sram_controller)
    // ------------------------------------------------------------------------
    assign ctrl_addr = pack_sr_we ? pack_sr_addr : rd_sram_addr;

    // ------------------------------------------------------------------------
    // Instance 1: image_loader (Nhan pixel 8-bit tu Nios II)
    // ------------------------------------------------------------------------
    image_loader #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT),
        .ADDR_W     (19)
    ) u_image_loader (
        .clk          (clk),
        .rst_n        (rst_n),
        // Nios II
        .start        (ld_start),
        .img_width    (ld_img_width),
        .img_height   (ld_img_height),
        .pixel_valid  (ld_pixel_valid),
        .pixel_data   (ld_pixel_data),
        .pixel_ready  (ld_pixel_ready),
        // loader_sram_pack
        .sram_addr    (ld_sram_addr_byte),
        .sram_wr_data (ld_sram_wr_data),
        .sram_we      (ld_sram_we),
        .sram_ready   (ld_sram_ready),
        // Status
        .image_loaded (ld_image_loaded),
        .size_error   (ld_size_error)
    );

    // ------------------------------------------------------------------------
    // Instance 2: loader_sram_pack (Ghep 2 byte 8-bit thanh 1 word 16-bit)
    // ------------------------------------------------------------------------
    loader_sram_pack u_loader_sram_pack (
        .clk      (clk),
        .rst_n    (rst_n),
        // Phia image_loader
        .ld_addr  (ld_sram_addr_byte),
        .ld_wdata (ld_sram_wr_data),
        .ld_we    (ld_sram_we),
        .ld_ready (ld_sram_ready),
        // Phia sram_controller
        .sr_addr  (pack_sr_addr),
        .sr_wdata (pack_sr_wdata),
        .sr_we    (pack_sr_we),
        .sr_ready (ctrl_ready)
    );

    // ------------------------------------------------------------------------
    // Instance 3: image_reader (Doc tung hang tu SRAM nap vao Line Buffer)
    // ------------------------------------------------------------------------
    image_reader #(
        .NUM_BANKS (NUM_BANKS)
    ) u_image_reader (
        .clk          (clk),
        .rst_n        (rst_n),
        // Dieu khien
        .start        (rd_start),
        .first_row    (rd_first_row),
        .num_rows     (rd_num_rows),
        .first_bank   (rd_first_bank),
        .busy         (rd_busy),
        .done         (rd_done),
        .error        (rd_error),
        // sram_controller
        .sram_addr    (rd_sram_addr),
        .sram_re      (rd_sram_re),
        .sram_rd_data (ctrl_rd_data),
        .sram_valid   (ctrl_valid),
        // Line Buffer
        .lb_we        (lb_we),
        .lb_row       (lb_row),
        .lb_col_word  (lb_col_word),
        .lb_wdata     (lb_wdata)
    );

    // ------------------------------------------------------------------------
    // Instance 4: sram_controller (Dieu khien giao tiep SRAM vat ly)
    // ------------------------------------------------------------------------
    sram_controller #(
        .ADDR_W       (18),
        .DATA_W       (16),
        .WR_PULSE_CYC (WR_PULSE_CYC),
        .RD_ACC_CYC   (RD_ACC_CYC)
    ) u_sram_controller (
        .clk       (clk),
        .rst_n     (rst_n),
        // Giao dien logic
        .addr      (ctrl_addr),
        .wr_data   (pack_sr_wdata),
        .we        (pack_sr_we),
        .re        (rd_sram_re),
        .rd_data   (ctrl_rd_data),
        .ready     (ctrl_ready),
        .valid     (ctrl_valid),
        .done      (ctrl_done),
        // Chan SRAM vat ly
        .sram_addr (SRAM_ADDR),
        .sram_dq   (SRAM_DQ),
        .sram_ce_n (SRAM_CE_N),
        .sram_oe_n (SRAM_OE_N),
        .sram_we_n (SRAM_WE_N),
        .sram_ub_n (SRAM_UB_N),
        .sram_lb_n (SRAM_LB_N)
    );

endmodule