// ============================================================================
// DE2_ORB_top.v
// Top-level Quartus cho kit Altera DE2 (EP2C35F672C6)
// ============================================================================
module DE2_ORB_top (
    input  wire        CLOCK_50,
    input  wire [0:0]  KEY,

    // SDRAM (16-bit tren kit DE2)
    output wire [11:0] DRAM_ADDR,
    output wire        DRAM_BA_0,
    output wire        DRAM_BA_1,
    output wire        DRAM_CAS_N,
    output wire        DRAM_CKE,
    output wire        DRAM_CLK,
    output wire        DRAM_CS_N,
    inout  wire [15:0] DRAM_DQ,
    output wire        DRAM_LDQM,
    output wire        DRAM_UDQM,
    output wire        DRAM_RAS_N,
    output wire        DRAM_WE_N,

    // SRAM (16-bit tren kit DE2)
    output wire [17:0] SRAM_ADDR,
    inout  wire [15:0] SRAM_DQ,
    output wire        SRAM_CE_N,
    output wire        SRAM_OE_N,
    output wire        SRAM_WE_N,
    output wire        SRAM_LB_N,
    output wire        SRAM_UB_N
);

    wire [1:0] sdram_ba;
    wire [1:0] sdram_dqm;

    assign {DRAM_BA_1, DRAM_BA_0} = sdram_ba;
    assign {DRAM_UDQM, DRAM_LDQM} = sdram_dqm;

    system u_sys (
        .clk_clk       (CLOCK_50),
        .reset_reset_n (KEY[0]),
        .sdram_clk_clk (DRAM_CLK),

        .sdram_addr    (DRAM_ADDR),
        .sdram_ba      (sdram_ba),
        .sdram_cas_n   (DRAM_CAS_N),
        .sdram_cke     (DRAM_CKE),
        .sdram_cs_n    (DRAM_CS_N),
        .sdram_dq      (DRAM_DQ),
        .sdram_dqm     (sdram_dqm),
        .sdram_ras_n   (DRAM_RAS_N),
        .sdram_we_n    (DRAM_WE_N),

        .sram_addr     (SRAM_ADDR),
        .sram_dq       (SRAM_DQ),
        .sram_ce_n     (SRAM_CE_N),
        .sram_oe_n     (SRAM_OE_N),
        .sram_we_n     (SRAM_WE_N),
        .sram_ub_n     (SRAM_UB_N),
        .sram_lb_n     (SRAM_LB_N)
    );

endmodule