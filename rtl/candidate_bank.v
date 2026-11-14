//==============================================================================
// candidate_bank.v
// Dual-port RAM lưu danh sách candidate của một hàng ảnh.
// Mỗi entry = {Score[7:0], X[9:0]} = 18 bit
// Depth = 610 entries (address 0..609)
// Port A : Write-only (từ Score module)
// Port B : Read-only  (tới NMS)
//==============================================================================

module candidate_bank #(
    parameter ADDR_WIDTH = 10,          // 2^10 = 1024 > 610
    parameter DATA_WIDTH = 18,          // 8-bit score + 10-bit X
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
    // Simple dual-port RAM (synthesizer will map to M4K / M9K / M10K)
    //------------------------------------------------------------------
    (* ramstyle = "M4K" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Optional: clear on reset (có thể bỏ để tiết kiệm logic)
            for (i = 0; i < DEPTH; i = i + 1)
                mem[i] <= {DATA_WIDTH{1'b0}};
            rd_data <= {DATA_WIDTH{1'b0}};
        end else begin
            // Write port
            if (wr_en && (wr_addr < DEPTH))
                mem[wr_addr] <= wr_data;

            // Read port (registered output)
            if (rd_en && (rd_addr < DEPTH))
                rd_data <= mem[rd_addr];
            else
                rd_data <= {DATA_WIDTH{1'b0}};
        end
    end

endmodule
