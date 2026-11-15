//==============================================================================
// candidate_bank.v
// Dual-port RAM lưu danh sách candidate của một hàng ảnh.
// Mỗi entry = {Score[7:0], X[9:0]} = 18 bit
// Depth = 610 entries (address 0..609)
// Port A : Write-only (từ Score module)
// Port B : Read-only  (tới NMS)
//==============================================================================

module candidate_bank #(
    parameter ADDR_WIDTH = 10,
    parameter DATA_WIDTH = 18,
    parameter DEPTH      = 610
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ---- Write port (Score) ----
    input  wire                     wr_en,
    input  wire [ADDR_WIDTH-1:0]    wr_addr,
    input  wire [DATA_WIDTH-1:0]    wr_data,   // {score[7:0], x[9:0]}

    // ---- Read port (NMS) ----
    input  wire                     rd_en,
    input  wire [ADDR_WIDTH-1:0]    rd_addr,
    output wire [DATA_WIDTH-1:0]    rd_data
);

    //------------------------------------------------------------------
    // Dual-port RAM (map vao M4K)
    //   - Mang nho nam trong process chi co clk (KHONG reset bat dong bo,
    //     KHONG xoa dau ra dong bo) de Quartus suy ra duoc RAM.
    //   - Doc dong bo 1 chu ky; rd_q la thanh ghi dau ra cua RAM.
    //   - no_rw_check: bo logic bypass doc-trong-luc-ghi (NMS chi doc bank
    //     FULL, Score chi ghi bank WRITING nen khong bao gio trung dia chi).
    //   - Hanh vi ben ngoai giu nguyen: rd_data = 0 khi chu ky truoc khong
    //     co doc hop le (rd_en && rd_addr < DEPTH); dung rd_vld de che.
    //------------------------------------------------------------------
    (* ramstyle = "M4K, no_rw_check" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    reg [DATA_WIDTH-1:0] rd_q;
    reg                  rd_vld;

    wire wr_ok = wr_en && (wr_addr < DEPTH);
    wire rd_ok = rd_en && (rd_addr < DEPTH);

    always @(posedge clk) begin
        if (wr_ok)
            mem[wr_addr] <= wr_data;
        rd_q <= mem[rd_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rd_vld <= 1'b0;
        else        rd_vld <= rd_ok;
    end

    assign rd_data = rd_vld ? rd_q : {DATA_WIDTH{1'b0}};

endmodule
