module orb_window_generator (
    input  wire         clk,
    input  wire         rst_n,

    input  wire         clear,          // xung: bat dau quet hang moi (xoa bo dem cot)

    // ---- tu orb_line_buffer ----
    input  wire         in_valid,       // rd_valid
    input  wire [7:0]   in_p0,          // hang 0
    input  wire [7:0]   in_p1,
    input  wire [7:0]   in_p2,
    input  wire [7:0]   in_p3,
    input  wire [7:0]   in_p4,
    input  wire [7:0]   in_p5,
    input  wire [7:0]   in_p6,          // hang 6

    // ---- cua so 7x7 ----
    output reg          window_valid,   // xung cung chu ky voi cua so moi
    output reg  [9:0]   win_center_x,   // toa do x cua pixel tam w33
    output wire [391:0] window_flat,    // {w00..w06, w10..w16, ..., w60..w66}

    // ---- 49 thanh ghi cua so (xuat rieng de tien noi day) ----
    output reg  [7:0]   w00, w01, w02, w03, w04, w05, w06,
    output reg  [7:0]   w10, w11, w12, w13, w14, w15, w16,
    output reg  [7:0]   w20, w21, w22, w23, w24, w25, w26,
    output reg  [7:0]   w30, w31, w32, w33, w34, w35, w36,
    output reg  [7:0]   w40, w41, w42, w43, w44, w45, w46,
    output reg  [7:0]   w50, w51, w52, w53, w54, w55, w56,
    output reg  [7:0]   w60, w61, w62, w63, w64, w65, w66
);

    // ---------------- bo dem cot ----------------
    reg [2:0] fill_cnt;     // so cot da nap, bao hoa o 6
    reg [9:0] in_col;       // chi so cot cua pixel dang duoc nap

    wire fill_full = (fill_cnt == 3'd6);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fill_cnt     <= 3'd0;
            in_col       <= 10'd0;
            window_valid <= 1'b0;
            win_center_x <= 10'd0;
        end else begin
            window_valid <= 1'b0;
            if (clear) begin
                fill_cnt <= 3'd0;
                in_col   <= 10'd0;
            end else if (in_valid) begin
                in_col <= in_col + 10'd1;
                if (!fill_full) begin
                    fill_cnt <= fill_cnt + 3'd1;
                end else begin
                    window_valid <= 1'b1;
                    win_center_x <= in_col - 10'd3;
                end
            end
        end
    end

    // ---------------- 49 thanh ghi cua so: dich trai, nap cot 6 ----------------
    always @(posedge clk) begin
        if (in_valid) begin
            // hang 0
            w00 <= w01;  w01 <= w02;  w02 <= w03;  w03 <= w04;  w04 <= w05;  w05 <= w06;  w06 <= in_p0;
            // hang 1
            w10 <= w11;  w11 <= w12;  w12 <= w13;  w13 <= w14;  w14 <= w15;  w15 <= w16;  w16 <= in_p1;
            // hang 2
            w20 <= w21;  w21 <= w22;  w22 <= w23;  w23 <= w24;  w24 <= w25;  w25 <= w26;  w26 <= in_p2;
            // hang 3
            w30 <= w31;  w31 <= w32;  w32 <= w33;  w33 <= w34;  w34 <= w35;  w35 <= w36;  w36 <= in_p3;
            // hang 4
            w40 <= w41;  w41 <= w42;  w42 <= w43;  w43 <= w44;  w44 <= w45;  w45 <= w46;  w46 <= in_p4;
            // hang 5
            w50 <= w51;  w51 <= w52;  w52 <= w53;  w53 <= w54;  w54 <= w55;  w55 <= w56;  w56 <= in_p5;
            // hang 6
            w60 <= w61;  w61 <= w62;  w62 <= w63;  w63 <= w64;  w64 <= w65;  w65 <= w66;  w66 <= in_p6;
        end
    end

    // ---------------- bus phang ----------------
    assign window_flat = {
        w00, w01, w02, w03, w04, w05, w06,
        w10, w11, w12, w13, w14, w15, w16,
        w20, w21, w22, w23, w24, w25, w26,
        w30, w31, w32, w33, w34, w35, w36,
        w40, w41, w42, w43, w44, w45, w46,
        w50, w51, w52, w53, w54, w55, w56,
        w60, w61, w62, w63, w64, w65, w66
    };

endmodule