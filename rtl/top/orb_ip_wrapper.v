// ============================================================================
// orb_ip_wrapper.v
// Qsys Top-Level Wrapper cho ORB Accelerator IP Core.
// Gom cac tin hieu dieu khien/trang thai thanh Avalon-MM Slave 32-bit (avs_csr),
// giu nguyen cac cong Avalon-MM nap anh, DMA ghi SDRAM va Conduit SRAM.
// ============================================================================
module orb_ip_wrapper #(
    parameter ADDR_W        = 11,       // Do sau buffer keypoint: 2^11 = 2048
    parameter DRAIN_CYC     = 16,       // So chu ky xa pipeline FAST/score
    parameter DMA_BURST_LEN = 1         // 1 cho Qsys SDR SDRAM controller
)(
    input  wire         clk,
    input  wire         rst_n,

    // ========================================================================
    // 1. Avalon-MM Slave: Thanh ghi CSR dieu khien & trang thai (32-bit)
    // ========================================================================
    input  wire [2:0]   avs_csr_address,        // 8 thanh ghi 32-bit (word offset 0..7)
    input  wire         avs_csr_write,
    input  wire [31:0]  avs_csr_writedata,
    input  wire [3:0]   avs_csr_byteenable,
    input  wire         avs_csr_read,
    output reg  [31:0]  avs_csr_readdata,
    output wire         irq,                    // Ngat gui ve Nios II

    // ========================================================================
    // 2. Avalon-MM Slave: Cua so nap anh 640x480 (16-bit)
    // ========================================================================
    input  wire [17:0]  avs_mem_address,
    input  wire         avs_mem_write,
    input  wire [15:0]  avs_mem_writedata,
    input  wire [1:0]   avs_mem_byteenable,
    output wire         avs_mem_waitrequest,

    // ========================================================================
    // 3. Avalon-MM Slave: Thanh ghi dieu khien nap anh (16-bit)
    // ========================================================================
    input  wire [1:0]   avs_ctrl_address,
    input  wire         avs_ctrl_write,
    input  wire [15:0]  avs_ctrl_writedata,
    input  wire         avs_ctrl_read,
    output wire [15:0]  avs_ctrl_readdata,

    // ========================================================================
    // 4. Avalon-MM Master: DMA ghi ket qua keypoint ra SDRAM (16-bit)
    // ========================================================================
    output wire [31:0]  avm_address,
    output wire         avm_write,
    output wire [15:0]  avm_writedata,
    output wire [1:0]   avm_byteenable,
    output wire [7:0]   avm_burstcount,
    input  wire         avm_waitrequest,

    // ========================================================================
    // 5. Conduit: Chan vat ly SRAM ngoai (Kit DE2)
    // ========================================================================
    output wire [17:0]  sram_addr,
    inout  wire [15:0]  sram_dq,
    output wire         sram_ce_n,
    output wire         sram_oe_n,
    output wire         sram_we_n,
    output wire         sram_ub_n,
    output wire         sram_lb_n
);

    localparam [31:0] IP_SIGNATURE = 32'h0B20_2026;

    // Thanh ghi cau hinh & trang thai chot (sticky)
    reg         orb_start_pulse;
    reg         irq_en;
    reg  [31:0] dma_base_addr_reg;
    reg  [7:0]  fast_thresh_reg;

    reg         done_sticky;
    reg         err_sticky;
    reg         kp_ovf_sticky;
    reg         pkt_ovf_sticky;

    // Tin hieu noi voi orb_top
    wire        image_loaded;
    wire        load_mode;
    wire        orb_busy;
    wire        orb_done;
    wire        orb_error;
    wire        kp_overflow;
    wire        packer_overflow;
    wire [11:0] out_count;
    wire [31:0] words_written;

    wire csr_wr_ctrl   = avs_csr_write && (avs_csr_address == 3'd0);
    wire clear_sticky  = csr_wr_ctrl && avs_csr_byteenable[0] && avs_csr_writedata[2];
    wire start_cmd     = csr_wr_ctrl && avs_csr_byteenable[0] && avs_csr_writedata[0];

    // ------------------------------------------------------------------------
    // Logic ghi thanh ghi CSR & tao xung start 1 chu ky
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            orb_start_pulse   <= 1'b0;
            irq_en            <= 1'b0;
            dma_base_addr_reg <= 32'h0000_0000;
            fast_thresh_reg   <= 8'd20;
        end else begin
            // Mac dinh ha xung start sau 1 chu ky
            orb_start_pulse <= start_cmd;

            if (avs_csr_write) begin
                case (avs_csr_address)
                    3'd0: begin // CTRL
                        if (avs_csr_byteenable[0])
                            irq_en <= avs_csr_writedata[1];
                    end
                    3'd2: begin // DMA_BASE_ADDR
                        if (avs_csr_byteenable[0]) dma_base_addr_reg[7:0]   <= avs_csr_writedata[7:0];
                        if (avs_csr_byteenable[1]) dma_base_addr_reg[15:8]  <= avs_csr_writedata[15:8];
                        if (avs_csr_byteenable[2]) dma_base_addr_reg[23:16] <= avs_csr_writedata[23:16];
                        if (avs_csr_byteenable[3]) dma_base_addr_reg[31:24] <= avs_csr_writedata[31:24];
                    end
                    3'd3: begin // THRESHOLD
                        if (avs_csr_byteenable[0])
                            fast_thresh_reg <= avs_csr_writedata[7:0];
                    end
                    default: ;
                endcase
            end
        end
    end

    // ------------------------------------------------------------------------
    // Chot cac co trang thai / loi (Sticky Flags) de Nios II doc khong bi hut xung
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done_sticky    <= 1'b0;
            err_sticky     <= 1'b0;
            kp_ovf_sticky  <= 1'b0;
            pkt_ovf_sticky <= 1'b0;
        end else if (start_cmd || clear_sticky) begin
            done_sticky    <= 1'b0;
            err_sticky     <= 1'b0;
            kp_ovf_sticky  <= 1'b0;
            pkt_ovf_sticky <= 1'b0;
        end else begin
            if (orb_done)        done_sticky    <= 1'b1;
            if (orb_error)       err_sticky     <= 1'b1;
            if (kp_overflow)     kp_ovf_sticky  <= 1'b1;
            if (packer_overflow) pkt_ovf_sticky <= 1'b1;
        end
    end

    assign irq = irq_en && (done_sticky || err_sticky);

    // ------------------------------------------------------------------------
    // Logic doc thanh ghi CSR (do tre 1 chu ky - chuan Avalon-MM Read Latency = 1)
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            avs_csr_readdata <= 32'd0;
        end else if (avs_csr_read) begin
            case (avs_csr_address)
                3'd0: avs_csr_readdata <= {30'd0, irq_en, 1'b0};
                3'd1: avs_csr_readdata <= {
                          25'd0,
                          load_mode,
                          image_loaded,
                          pkt_ovf_sticky,
                          kp_ovf_sticky,
                          err_sticky,
                          done_sticky,
                          orb_busy
                      };
                3'd2: avs_csr_readdata <= dma_base_addr_reg;
                3'd3: avs_csr_readdata <= {24'd0, fast_thresh_reg};
                3'd4: avs_csr_readdata <= {20'd0, out_count};
                3'd5: avs_csr_readdata <= words_written;
                3'd6: avs_csr_readdata <= IP_SIGNATURE;
                default: avs_csr_readdata <= 32'd0;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Khoi tao nhan xu ly chinh orb_top
    // ------------------------------------------------------------------------
    orb_top #(
        .ADDR_W        (ADDR_W),
        .DRAIN_CYC     (DRAIN_CYC),
        .DMA_BURST_LEN (DMA_BURST_LEN)
    ) u_orb_top (
        .clk                (clk),
        .rst_n              (rst_n),

        // Nap anh
        .avs_mem_address    (avs_mem_address),
        .avs_mem_write      (avs_mem_write),
        .avs_mem_writedata  (avs_mem_writedata),
        .avs_mem_byteenable (avs_mem_byteenable),
        .avs_mem_waitrequest(avs_mem_waitrequest),

        .avs_ctrl_address   (avs_ctrl_address),
        .avs_ctrl_write     (avs_ctrl_write),
        .avs_ctrl_writedata (avs_ctrl_writedata),
        .avs_ctrl_read      (avs_ctrl_read),
        .avs_ctrl_readdata  (avs_ctrl_readdata),

        .image_loaded       (image_loaded),
        .load_mode          (load_mode),

        // Dieu khien & trang thai
        .orb_start          (orb_start_pulse),
        .fast_threshold     (fast_thresh_reg),
        .dma_base_addr      (dma_base_addr_reg),
        .orb_busy           (orb_busy),
        .orb_done           (orb_done),
        .orb_error          (orb_error),
        .kp_overflow        (kp_overflow),
        .packer_overflow    (packer_overflow),
        .out_count          (out_count),
        .words_written      (words_written),

        // DMA SDRAM Master
        .avm_address        (avm_address),
        .avm_write          (avm_write),
        .avm_writedata      (avm_writedata),
        .avm_byteenable     (avm_byteenable),
        .avm_burstcount     (avm_burstcount),
        .avm_waitrequest    (avm_waitrequest),

        // SRAM ngoai
        .sram_addr          (sram_addr),
        .sram_dq            (sram_dq),
        .sram_ce_n          (sram_ce_n),
        .sram_oe_n          (sram_oe_n),
        .sram_we_n          (sram_we_n),
        .sram_ub_n          (sram_ub_n),
        .sram_lb_n          (sram_lb_n)
    );

endmodule