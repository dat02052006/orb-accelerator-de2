// ============================================================================
// keypoint_buffer.v
// Bo dem keypoint don gian giua nms_top va centroid_ctrl (thay keypoint_selector).
//
//   nms_top --(kp_valid, kp_data[27:0], kp_ready)--> keypoint_buffer
//   keypoint_buffer --(kp_count, rd_*)--> centroid_ctrl
//
// - kp_data tu NMS  : {score[7:0], row_id[9:0], x[9:0]}   (28 bit)
//   row_id la chi so hang tinh tu hang tam dau tien (y = 3), nen:
//   y_anh = row_id + Y_OFFSET (mac dinh 3), cat con 9 bit.
// - Du lieu luu     : {score[7:0], y[8:0], x[9:0]}        (27 bit)
//   khop kp_rd_data cua centroid_ctrl.
// - Ghi tuan tu theo thu tu NMS, kp_count = so keypoint da ghi.
// - clear (xung) : xoa kp_count va co overflow, bat dau frame moi.
// - Day (kp_count = 2^ADDR_W): kp_ready = 0 de NMS dung lai; neu van co
//   kp_valid thi bi bo va overflow = 1 (khong bao gio ghi de).
// - Doc : rd_en o chu ky t -> rd_data hop le o chu ky t+1 (giu den lan doc sau).
//   Khop S_FETCH -> S_LATCH cua centroid_ctrl.
// - Chi dung 1 bank: kp_count on dinh khi centroid_ctrl chay, vi he thong xu ly
//   tung frame (NMS xong frame -> orientation -> frame ke tiep sau clear).
// ============================================================================
module keypoint_buffer #(
    parameter ADDR_W   = 10,            // 2^ADDR_W keypoint (trung voi centroid_ctrl)
    parameter Y_OFFSET = 3              // y hang tam dau tien cua line buffer
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 clear,          // xung: bat dau frame moi

    // tu nms_top
    input  wire                 kp_valid,
    input  wire [27:0]          kp_data,        // {score[7:0], y[9:0], x[9:0]}
    output wire                 kp_ready,

    // trang thai
    output wire [ADDR_W:0]      kp_count,
    output reg                  overflow,

    // sang centroid_ctrl
    input  wire                 rd_en,
    input  wire [ADDR_W-1:0]    rd_addr,
    output reg  [26:0]          rd_data         // {score[7:0], y[8:0], x[9:0]}
);

    localparam DEPTH = (1 << ADDR_W);

    reg [ADDR_W:0] count;

    wire full  = count[ADDR_W];                 // count == 2^ADDR_W
    wire wr_ok = kp_valid & ~full;

    // y anh = row_id + Y_OFFSET (hang tam dau tien o y = Y_OFFSET)
    wire [9:0]  y_sum   = kp_data[19:10] + Y_OFFSET;
    wire [26:0] wr_word = {kp_data[27:20], y_sum[8:0], kp_data[9:0]};

    // ---------------- bo dem + co overflow ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count    <= {(ADDR_W+1){1'b0}};
            overflow <= 1'b0;
        end else if (clear) begin
            count    <= {(ADDR_W+1){1'b0}};
            overflow <= 1'b0;
        end else begin
            if (wr_ok)
                count <= count + {{ADDR_W{1'b0}}, 1'b1};
            if (kp_valid & full)
                overflow <= 1'b1;
        end
    end

    // ---------------- RAM (map vao M4K, khong reset) ----------------
    (* ramstyle = "M4K" *) reg [26:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (wr_ok)
            mem[count[ADDR_W-1:0]] <= wr_word;
    end

    always @(posedge clk) begin
        if (rd_en)
            rd_data <= mem[rd_addr];
    end

    assign kp_ready = ~full;
    assign kp_count = count;

endmodule
