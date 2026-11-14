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
    output reg  [DATA_WIDTH-1:0]    rd_data
);

    //------------------------------------------------------------------
    // Dual-port RAM (synthesizer map to M4K)
    // Không clear memory khi reset – tiết kiệm logic, FPGA BRAM không cần
    //------------------------------------------------------------------
    (* ramstyle = "M4K" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_data <= {DATA_WIDTH{1'b0}};
        end else begin
            // Write port
            if (wr_en && (wr_addr < DEPTH))
                mem[wr_addr] <= wr_data;

            // Read port (registered, 1-cycle latency)
            if (rd_en && (rd_addr < DEPTH))
                rd_data <= mem[rd_addr];
            else
                rd_data <= {DATA_WIDTH{1'b0}};
        end
    end

endmodule
