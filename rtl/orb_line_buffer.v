module orb_line_buffer (
    input  wire        clk,

    // ---- cong ghi (tu image_reader) ----
    input  wire        lb_we,
    input  wire [4:0]  lb_row,          // 0..6
    input  wire [8:0]  lb_col_word,     // 0..319
    input  wire [15:0] lb_wdata,        // [7:0] cot chan, [15:8] cot le

    // ---- cong doc: 7 pixel cung cot ----
    input  wire        rd_en,
    input  wire [9:0]  rd_col,          // toa do pixel 0..639
    output reg         rd_valid,        // du lieu hop le sau 1 chu ky
    output wire [7:0]  rd_p0,           // hang 0
    output wire [7:0]  rd_p1,
    output wire [7:0]  rd_p2,
    output wire [7:0]  rd_p3,
    output wire [7:0]  rd_p4,
    output wire [7:0]  rd_p5,
    output wire [7:0]  rd_p6            // hang 6
);

    // ---------------- 7 bo nho hang: 320 word x 16 bit ----------------
    (* ramstyle = "M4K" *) reg [15:0] mem0 [0:319];
    (* ramstyle = "M4K" *) reg [15:0] mem1 [0:319];
    (* ramstyle = "M4K" *) reg [15:0] mem2 [0:319];
    (* ramstyle = "M4K" *) reg [15:0] mem3 [0:319];
    (* ramstyle = "M4K" *) reg [15:0] mem4 [0:319];
    (* ramstyle = "M4K" *) reg [15:0] mem5 [0:319];
    (* ramstyle = "M4K" *) reg [15:0] mem6 [0:319];

    reg [15:0] word0, word1, word2, word3, word4, word5, word6;

    // ---------------- giai ma ghi ----------------
    wire we0 = lb_we && (lb_row == 5'd0);
    wire we1 = lb_we && (lb_row == 5'd1);
    wire we2 = lb_we && (lb_row == 5'd2);
    wire we3 = lb_we && (lb_row == 5'd3);
    wire we4 = lb_we && (lb_row == 5'd4);
    wire we5 = lb_we && (lb_row == 5'd5);
    wire we6 = lb_we && (lb_row == 5'd6);

    wire [8:0] rd_addr = rd_col[9:1];   // dia chi word khi doc

    // ---------------- ghi / doc dong bo tung hang ----------------
    always @(posedge clk) begin
        if (we0) mem0[lb_col_word] <= lb_wdata;
        word0 <= mem0[rd_addr];
    end

    always @(posedge clk) begin
        if (we1) mem1[lb_col_word] <= lb_wdata;
        word1 <= mem1[rd_addr];
    end

    always @(posedge clk) begin
        if (we2) mem2[lb_col_word] <= lb_wdata;
        word2 <= mem2[rd_addr];
    end

    always @(posedge clk) begin
        if (we3) mem3[lb_col_word] <= lb_wdata;
        word3 <= mem3[rd_addr];
    end

    always @(posedge clk) begin
        if (we4) mem4[lb_col_word] <= lb_wdata;
        word4 <= mem4[rd_addr];
    end

    always @(posedge clk) begin
        if (we5) mem5[lb_col_word] <= lb_wdata;
        word5 <= mem5[rd_addr];
    end

    always @(posedge clk) begin
        if (we6) mem6[lb_col_word] <= lb_wdata;
        word6 <= mem6[rd_addr];
    end

    // ---------------- chon byte chan/le (tre 1 chu ky cung du lieu) ----------------
    reg col_lsb_d;

    always @(posedge clk) begin
        col_lsb_d <= rd_col[0];
        rd_valid  <= rd_en;
    end

    assign rd_p0 = col_lsb_d ? word0[15:8] : word0[7:0];
    assign rd_p1 = col_lsb_d ? word1[15:8] : word1[7:0];
    assign rd_p2 = col_lsb_d ? word2[15:8] : word2[7:0];
    assign rd_p3 = col_lsb_d ? word3[15:8] : word3[7:0];
    assign rd_p4 = col_lsb_d ? word4[15:8] : word4[7:0];
    assign rd_p5 = col_lsb_d ? word5[15:8] : word5[7:0];
    assign rd_p6 = col_lsb_d ? word6[15:8] : word6[7:0];

endmodule