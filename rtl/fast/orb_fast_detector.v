// ============================================================================
// orb_fast_detector.v
// FAST-9 comparator cho ORB accelerator (Cyclone II / DE2)
//
// Pixel i tren duong tron ban kinh 3, thu tu lien tiep quanh vong:
//   p0 = (0,-3)  p1 = (1,-3)  p2 = (2,-2)  p3 = (3,-1)
//   p4 = (3, 0)  p5 = (3, 1)  p6 = (2, 2)  p7 = (1, 3)
//   p8 = (0, 3)  p9 = (-1,3)  p10= (-2,2)  p11= (-3,1)
//   p12= (-3,0)  p13= (-3,-1) p14= (-2,-2) p15= (-1,-3)
//
// bright_i : Ii > Ip + T
// dark_i   : Ii < Ip - T
// Corner   : >= 9 pixel lien tiep (vong tron, co wrap) cung sang hoac cung toi
//
// Bien: chi pixel trung tam co x,y cach bien >= BORDER (15) moi duoc xet.
// Latency: 4 chu ky, throughput 1 cua so / chu ky.
//
// pack (136 bit): {p15, p14, ..., p1, p0, ip}
//   - LSB [7:0]   = ip  (pixel trung tam)
//   - [15:8]      = p0  (huong 12 gio)
//   - [23:16]     = p1  (ke tiep theo chieu kim dong ho)
//   - ...
//   - [135:128]   = p15
//   pack duoc pipeline 4 stage de dong bo voi out_valid.
// ============================================================================
module orb_fast_detector #(
    parameter IMG_W   = 640,
    parameter IMG_H   = 480,
    parameter BORDER  = 15,
    parameter COORD_W = 10
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 in_valid,       // window_valid tu window generator
    input  wire [COORD_W-1:0]   in_x,           // toa do cot cua pixel trung tam
    input  wire [COORD_W-1:0]   in_y,           // toa do hang cua pixel trung tam
    input  wire [7:0]           threshold,      // FAST threshold T

    input  wire [7:0]           ip,             // pixel trung tam
    input  wire [7:0] p0,  p1,  p2,  p3,
    input  wire [7:0] p4,  p5,  p6,  p7,
    input  wire [7:0] p8,  p9,  p10, p11,
    input  wire [7:0] p12, p13, p14, p15,

    output reg                  out_valid,
    output reg                  is_corner,
    output reg  [COORD_W-1:0]   out_x,
    output reg  [COORD_W-1:0]   out_y,

    // 17 pixel: {p15..p0, ip}, LSB = ip (trung tam)
    output reg  [135:0]        pack
);

    // ------------------------------------------------------------------
    // Gioi han vung hop le (cach bien >= BORDER)
    // ------------------------------------------------------------------
    localparam [COORD_W-1:0] X_MIN = BORDER;
    localparam [COORD_W-1:0] X_MAX = IMG_W - BORDER - 1;
    localparam [COORD_W-1:0] Y_MIN = BORDER;
    localparam [COORD_W-1:0] Y_MAX = IMG_H - BORDER - 1;

    wire border_ok_w;
    assign border_ok_w = (in_x >= X_MIN) && (in_x <= X_MAX) &&
                         (in_y >= Y_MIN) && (in_y <= Y_MAX);

    // ------------------------------------------------------------------
    // Nguong: Ip+T (9 bit, khong tran) va Ip-T (bao hoa ve 0)
    // ------------------------------------------------------------------
    wire [8:0] thr_hi;   // Ip + T
    wire [7:0] thr_lo;   // max(Ip - T, 0)

    assign thr_hi = {1'b0, ip} + {1'b0, threshold};
    assign thr_lo = (ip > threshold) ? (ip - threshold) : 8'd0;

    // ------------------------------------------------------------------
    // Stage 1: 16 bo so sanh sang hon + 16 bo so sanh toi hon + chot pack
    // ------------------------------------------------------------------
    reg [15:0] bright_s1;
    reg [15:0] dark_s1;
    reg        valid_s1;
    reg        border_s1;
    reg [COORD_W-1:0] x_s1, y_s1;
    reg [135:0]      pack_s1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1  <= 1'b0;
            border_s1 <= 1'b0;
            bright_s1 <= 16'd0;
            dark_s1   <= 16'd0;
            x_s1      <= {COORD_W{1'b0}};
            y_s1      <= {COORD_W{1'b0}};
            pack_s1   <= 136'd0;
        end else begin
            valid_s1  <= in_valid;
            border_s1 <= border_ok_w;
            x_s1      <= in_x;
            y_s1      <= in_y;

            // Dong goi 17 pixel: ip o LSB, p0..p15 theo chieu kim dong ho
            pack_s1 <= {p15, p14, p13, p12,
                        p11, p10, p9,  p8,
                        p7,  p6,  p5,  p4,
                        p3,  p2,  p1,  p0,
                        ip};

            // sang hon: Ii > Ip + T
            bright_s1[0]  <= ({1'b0, p0 } > thr_hi);
            bright_s1[1]  <= ({1'b0, p1 } > thr_hi);
            bright_s1[2]  <= ({1'b0, p2 } > thr_hi);
            bright_s1[3]  <= ({1'b0, p3 } > thr_hi);
            bright_s1[4]  <= ({1'b0, p4 } > thr_hi);
            bright_s1[5]  <= ({1'b0, p5 } > thr_hi);
            bright_s1[6]  <= ({1'b0, p6 } > thr_hi);
            bright_s1[7]  <= ({1'b0, p7 } > thr_hi);
            bright_s1[8]  <= ({1'b0, p8 } > thr_hi);
            bright_s1[9]  <= ({1'b0, p9 } > thr_hi);
            bright_s1[10] <= ({1'b0, p10} > thr_hi);
            bright_s1[11] <= ({1'b0, p11} > thr_hi);
            bright_s1[12] <= ({1'b0, p12} > thr_hi);
            bright_s1[13] <= ({1'b0, p13} > thr_hi);
            bright_s1[14] <= ({1'b0, p14} > thr_hi);
            bright_s1[15] <= ({1'b0, p15} > thr_hi);

            // toi hon: Ii < Ip - T
            dark_s1[0]  <= (p0  < thr_lo);
            dark_s1[1]  <= (p1  < thr_lo);
            dark_s1[2]  <= (p2  < thr_lo);
            dark_s1[3]  <= (p3  < thr_lo);
            dark_s1[4]  <= (p4  < thr_lo);
            dark_s1[5]  <= (p5  < thr_lo);
            dark_s1[6]  <= (p6  < thr_lo);
            dark_s1[7]  <= (p7  < thr_lo);
            dark_s1[8]  <= (p8  < thr_lo);
            dark_s1[9]  <= (p9  < thr_lo);
            dark_s1[10] <= (p10 < thr_lo);
            dark_s1[11] <= (p11 < thr_lo);
            dark_s1[12] <= (p12 < thr_lo);
            dark_s1[13] <= (p13 < thr_lo);
            dark_s1[14] <= (p14 < thr_lo);
            dark_s1[15] <= (p15 < thr_lo);
        end
    end

    // ------------------------------------------------------------------
    // Stage 2: AND 3 pixel lien tiep, b3[i] = b[i] & b[i+1] & b[i+2] (wrap 16)
    // ------------------------------------------------------------------
    wire [15:0] b3_w;
    wire [15:0] d3_w;

    assign b3_w[0]  = bright_s1[0]  & bright_s1[1]  & bright_s1[2];
    assign b3_w[1]  = bright_s1[1]  & bright_s1[2]  & bright_s1[3];
    assign b3_w[2]  = bright_s1[2]  & bright_s1[3]  & bright_s1[4];
    assign b3_w[3]  = bright_s1[3]  & bright_s1[4]  & bright_s1[5];
    assign b3_w[4]  = bright_s1[4]  & bright_s1[5]  & bright_s1[6];
    assign b3_w[5]  = bright_s1[5]  & bright_s1[6]  & bright_s1[7];
    assign b3_w[6]  = bright_s1[6]  & bright_s1[7]  & bright_s1[8];
    assign b3_w[7]  = bright_s1[7]  & bright_s1[8]  & bright_s1[9];
    assign b3_w[8]  = bright_s1[8]  & bright_s1[9]  & bright_s1[10];
    assign b3_w[9]  = bright_s1[9]  & bright_s1[10] & bright_s1[11];
    assign b3_w[10] = bright_s1[10] & bright_s1[11] & bright_s1[12];
    assign b3_w[11] = bright_s1[11] & bright_s1[12] & bright_s1[13];
    assign b3_w[12] = bright_s1[12] & bright_s1[13] & bright_s1[14];
    assign b3_w[13] = bright_s1[13] & bright_s1[14] & bright_s1[15];
    assign b3_w[14] = bright_s1[14] & bright_s1[15] & bright_s1[0];
    assign b3_w[15] = bright_s1[15] & bright_s1[0]  & bright_s1[1];

    assign d3_w[0]  = dark_s1[0]  & dark_s1[1]  & dark_s1[2];
    assign d3_w[1]  = dark_s1[1]  & dark_s1[2]  & dark_s1[3];
    assign d3_w[2]  = dark_s1[2]  & dark_s1[3]  & dark_s1[4];
    assign d3_w[3]  = dark_s1[3]  & dark_s1[4]  & dark_s1[5];
    assign d3_w[4]  = dark_s1[4]  & dark_s1[5]  & dark_s1[6];
    assign d3_w[5]  = dark_s1[5]  & dark_s1[6]  & dark_s1[7];
    assign d3_w[6]  = dark_s1[6]  & dark_s1[7]  & dark_s1[8];
    assign d3_w[7]  = dark_s1[7]  & dark_s1[8]  & dark_s1[9];
    assign d3_w[8]  = dark_s1[8]  & dark_s1[9]  & dark_s1[10];
    assign d3_w[9]  = dark_s1[9]  & dark_s1[10] & dark_s1[11];
    assign d3_w[10] = dark_s1[10] & dark_s1[11] & dark_s1[12];
    assign d3_w[11] = dark_s1[11] & dark_s1[12] & dark_s1[13];
    assign d3_w[12] = dark_s1[12] & dark_s1[13] & dark_s1[14];
    assign d3_w[13] = dark_s1[13] & dark_s1[14] & dark_s1[15];
    assign d3_w[14] = dark_s1[14] & dark_s1[15] & dark_s1[0];
    assign d3_w[15] = dark_s1[15] & dark_s1[0]  & dark_s1[1];

    reg [15:0] b3_s2;
    reg [15:0] d3_s2;
    reg        valid_s2;
    reg        border_s2;
    reg [COORD_W-1:0] x_s2, y_s2;
    reg [135:0]      pack_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s2  <= 1'b0;
            border_s2 <= 1'b0;
            b3_s2     <= 16'd0;
            d3_s2     <= 16'd0;
            x_s2      <= {COORD_W{1'b0}};
            y_s2      <= {COORD_W{1'b0}};
            pack_s2   <= 136'd0;
        end else begin
            valid_s2  <= valid_s1;
            border_s2 <= border_s1;
            b3_s2     <= b3_w;
            d3_s2     <= d3_w;
            x_s2      <= x_s1;
            y_s2      <= y_s1;
            pack_s2   <= pack_s1;
        end
    end

    // ------------------------------------------------------------------
    // Stage 3: run9[i] = b3[i] & b3[i+3] & b3[i+6]  (9 pixel lien tiep)
    //          roi OR 16 vi tri bat dau
    // ------------------------------------------------------------------
    wire [15:0] b9_w;
    wire [15:0] d9_w;

    assign b9_w[0]  = b3_s2[0]  & b3_s2[3]  & b3_s2[6];
    assign b9_w[1]  = b3_s2[1]  & b3_s2[4]  & b3_s2[7];
    assign b9_w[2]  = b3_s2[2]  & b3_s2[5]  & b3_s2[8];
    assign b9_w[3]  = b3_s2[3]  & b3_s2[6]  & b3_s2[9];
    assign b9_w[4]  = b3_s2[4]  & b3_s2[7]  & b3_s2[10];
    assign b9_w[5]  = b3_s2[5]  & b3_s2[8]  & b3_s2[11];
    assign b9_w[6]  = b3_s2[6]  & b3_s2[9]  & b3_s2[12];
    assign b9_w[7]  = b3_s2[7]  & b3_s2[10] & b3_s2[13];
    assign b9_w[8]  = b3_s2[8]  & b3_s2[11] & b3_s2[14];
    assign b9_w[9]  = b3_s2[9]  & b3_s2[12] & b3_s2[15];
    assign b9_w[10] = b3_s2[10] & b3_s2[13] & b3_s2[0];
    assign b9_w[11] = b3_s2[11] & b3_s2[14] & b3_s2[1];
    assign b9_w[12] = b3_s2[12] & b3_s2[15] & b3_s2[2];
    assign b9_w[13] = b3_s2[13] & b3_s2[0]  & b3_s2[3];
    assign b9_w[14] = b3_s2[14] & b3_s2[1]  & b3_s2[4];
    assign b9_w[15] = b3_s2[15] & b3_s2[2]  & b3_s2[5];

    assign d9_w[0]  = d3_s2[0]  & d3_s2[3]  & d3_s2[6];
    assign d9_w[1]  = d3_s2[1]  & d3_s2[4]  & d3_s2[7];
    assign d9_w[2]  = d3_s2[2]  & d3_s2[5]  & d3_s2[8];
    assign d9_w[3]  = d3_s2[3]  & d3_s2[6]  & d3_s2[9];
    assign d9_w[4]  = d3_s2[4]  & d3_s2[7]  & d3_s2[10];
    assign d9_w[5]  = d3_s2[5]  & d3_s2[8]  & d3_s2[11];
    assign d9_w[6]  = d3_s2[6]  & d3_s2[9]  & d3_s2[12];
    assign d9_w[7]  = d3_s2[7]  & d3_s2[10] & d3_s2[13];
    assign d9_w[8]  = d3_s2[8]  & d3_s2[11] & d3_s2[14];
    assign d9_w[9]  = d3_s2[9]  & d3_s2[12] & d3_s2[15];
    assign d9_w[10] = d3_s2[10] & d3_s2[13] & d3_s2[0];
    assign d9_w[11] = d3_s2[11] & d3_s2[14] & d3_s2[1];
    assign d9_w[12] = d3_s2[12] & d3_s2[15] & d3_s2[2];
    assign d9_w[13] = d3_s2[13] & d3_s2[0]  & d3_s2[3];
    assign d9_w[14] = d3_s2[14] & d3_s2[1]  & d3_s2[4];
    assign d9_w[15] = d3_s2[15] & d3_s2[2]  & d3_s2[5];

    reg        any_bright_s3;
    reg        any_dark_s3;
    reg        valid_s3;
    reg        border_s3;
    reg [COORD_W-1:0] x_s3, y_s3;
    reg [135:0]      pack_s3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s3      <= 1'b0;
            border_s3     <= 1'b0;
            any_bright_s3 <= 1'b0;
            any_dark_s3   <= 1'b0;
            x_s3          <= {COORD_W{1'b0}};
            y_s3          <= {COORD_W{1'b0}};
            pack_s3       <= 136'd0;
        end else begin
            valid_s3      <= valid_s2;
            border_s3     <= border_s2;
            any_bright_s3 <= |b9_w;
            any_dark_s3   <= |d9_w;
            x_s3          <= x_s2;
            y_s3          <= y_s2;
            pack_s3       <= pack_s2;
        end
    end

    // ------------------------------------------------------------------
    // Stage 4: quyet dinh corner / non-corner, loai pixel sat bien
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            is_corner <= 1'b0;
            out_x     <= {COORD_W{1'b0}};
            out_y     <= {COORD_W{1'b0}};
            pack      <= 136'd0;
        end else begin
            out_valid <= valid_s3;
            is_corner <= (any_bright_s3 | any_dark_s3) & border_s3;
            out_x     <= x_s3;
            out_y     <= y_s3;
            pack      <= pack_s3;
        end
    end

endmodule