// -----------------------------------------------------------------------------
// orb_result_packer.v
// Dong goi 1 keypoint (288 bit) thanh 18 word 16-bit va day vao Result FIFO.
// Dong bo tin hieu src_done cho orb_dma_write.
// -----------------------------------------------------------------------------
module orb_result_packer (
    input  wire         clk,
    input  wire         rst,            // sync, active-high
    input  wire         start,          // xung 1 chu ky (xoa co orb_done_latched)

    // Giao tiep tu ORB core
    input  wire         orb_done,       // xung 1 chu ky hoac muc 1 khi ORB xu ly xong
    input  wire         out_we,
    input  wire [9:0]   out_x,
    input  wire [8:0]   out_y,
    input  wire [7:0]   out_score,
    input  wire [4:0]   out_angle,
    input  wire [255:0] out_desc,

    // Giao tiep toi Result FIFO (scfifo 16-bit)
    input  wire         fifo_full,
    output wire [15:0]  fifo_wdata,
    output wire         fifo_wr,

    // Giao tiep toi orb_dma_write
    output wire         src_done,
    output wire         packer_overflow // bao loi neu out_we den khi chua day xong 18 word
);

    reg [303:0] shift_reg;
    reg [4:0]   word_cnt;       // dem tu 19 ve 0
    reg         orb_done_latched;
    reg         overflow_reg;

    wire busy = (word_cnt != 5'd0);

    // Chot trang thai orb_done (phong truong hop orb_done chi la xung 1 chu ky)
    always @(posedge clk) begin
        if (rst || start) begin
            orb_done_latched <= 1'b0;
        end else if (orb_done) begin
            orb_done_latched <= 1'b1;
        end
    end

    // Thanh ghi dich va bo dem 19 word
    always @(posedge clk) begin
        if (rst || start) begin
            shift_reg    <= 304'd0;
            word_cnt     <= 5'd0;
            overflow_reg <= 1'b0;
        end else begin
            if (out_we) begin
                if (busy) begin
                    overflow_reg <= 1'b1; 
                end else begin
                    // Dong goi 304 bit (19 word, Word 0 nam o [15:0])
                    shift_reg <= {
                        out_desc[255:0],                        // Word 3..18 (16 word)
                        3'd0, out_angle[4:0], out_score[7:0],   // Word 2
                        7'd0, out_y[8:0],                       // Word 1
                        6'd0, out_x[9:0]                        // Word 0
                    };
                    word_cnt <= 5'd19;
                end
            end else if (busy && !fifo_full) begin
                shift_reg <= {16'd0, shift_reg[303:16]};
                word_cnt  <= word_cnt - 5'd1;
            end
        end
    end
    assign fifo_wdata      = shift_reg[15:0];
    assign fifo_wr         = busy && !fifo_full;
    assign packer_overflow = overflow_reg;

    // Chi bao src_done cho DMA khi ORB da xong VA Packer da day het word vao FIFO
    assign src_done        = orb_done_latched && !busy && !out_we;

endmodule