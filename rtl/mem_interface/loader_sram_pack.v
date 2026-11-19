// ============================================================================
// loader_sram_pack.v
// Cau noi giua image_loader (dia chi byte 19 bit, du lieu 8 bit, ghi tung pixel)
// va sram_controller (dia chi word 18 bit, du lieu 16 bit, UB/LB luon = 0).
//
// Anh den theo thu tu raster nen dia chi byte tang dan tung 1:
//   - byte CHAN (addr[0] = 0): chi chot vao low_r, bao ready ngay cho loader,
//     khong truy cap SRAM.
//   - byte LE   (addr[0] = 1): ghi 1 word = {byte_le, low_r} tai addr[18:1],
//     ready lay tu sram_controller.
//
// Khop voi image_reader: word = {pixel cot le [15:8], pixel cot chan [7:0]}.
// Yeu cau: so byte cua anh la so chan (640 x 480 = 307200).
//
// Loader giu we = 1 cho den khi ready = 1 (we & ready = giao dich duoc nhan).
// ============================================================================
module loader_sram_pack (
    input  wire        clk,
    input  wire        rst_n,

    // ---- phia image_loader ----
    input  wire [18:0] ld_addr,         // dia chi byte
    input  wire [7:0]  ld_wdata,
    input  wire        ld_we,
    output wire        ld_ready,

    // ---- phia sram_controller ----
    output wire [17:0] sr_addr,         // dia chi word
    output wire [15:0] sr_wdata,
    output wire        sr_we,
    input  wire        sr_ready
);

    wire is_odd = ld_addr[0];

    reg [7:0] low_r;                    // byte chan dang cho ghep

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            low_r <= 8'd0;
        else if (ld_we && !is_odd)
            low_r <= ld_wdata;
    end

    assign sr_addr  = ld_addr[18:1];
    assign sr_wdata = {ld_wdata, low_r};
    assign sr_we    = ld_we & is_odd;
    assign ld_ready = is_odd ? sr_ready : 1'b1;

endmodule
