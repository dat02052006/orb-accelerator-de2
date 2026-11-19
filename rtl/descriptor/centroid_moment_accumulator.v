module centroid_moment_accumulator (
    input  wire               clk,
    input  wire               rst_n,

    // ---- dieu khien ----
    input  wire               clear,        // xung: xoa m10/m01 (phat cung luc voi start cua patch reader)
    input  wire               patch_done,   // done cua centroid_patch_reader

    // ---- nghe luong ghi patch cua centroid_patch_reader ----
    input  wire               pb_we,
    input  wire [5:0]         pb_row,       // 0..42 (patch 43x43)
    input  wire [4:0]         pb_word,      // 0..21
    input  wire [15:0]        pb_wdata,     // [7:0] = pixel cot chan, [15:8] = pixel cot le
    input  wire               patch_x0_odd,

    // ---- ket qua ----
    output reg  signed [23:0] m10,          // sum dx * I
    output reg  signed [23:0] m01,          // sum dy * I
    output reg                mom_valid     // xung 1 chu ky: m10/m01 da cong xong toan patch
);

    // 1: dung bang umax cua OpenCV (khop mo hinh phan mem)
    // 0: dung mat na tron dx^2 + dy^2 <= 15^2
    localparam USE_UMAX = 1'b1;

    // ======================================================
    // Stage 1: chot luong pb_* (patch reader chi giu du lieu trong chu ky pb_we)
    // ======================================================
    reg        in_v;
    reg [5:0]  in_row;
    reg [4:0]  in_word;
    reg [15:0] in_data;
    reg        in_odd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_v    <= 1'b0;
            in_row  <= 6'd0;
            in_word <= 5'd0;
            in_data <= 16'd0;
            in_odd  <= 1'b0;
        end else begin
            in_v <= pb_we;
            if (pb_we) begin
                in_row  <= pb_row;
                in_word <= pb_word;
                in_data <= pb_wdata;
                in_odd  <= patch_x0_odd;
            end
        end
    end

    // ======================================================
    // Stage 2: toa do trong patch + mat na (combinational tu in_*)
    //   Patch 43x43, tam (21,21); chi cong cua so 31x31 o giua.
    //   cot chan  c0 = 2*word - x0_odd,  dx0 = c0 - 21
    //   cot le    c1 = c0 + 1,           dx1 = dx0 + 1
    //   dy = row - 21
    // ======================================================
    wire signed [7:0] dx0_f = $signed({2'b00, in_word, 1'b0}) - $signed({7'd0, in_odd}) - 8'sd21;   // -22..21
    wire signed [7:0] dx1_f = dx0_f + 8'sd1;                                                         // -21..22
    wire signed [7:0] dy_f  = $signed({2'b00, in_row}) - 8'sd21;                                     // -21..21

    wire signed [5:0] dx0 = dx0_f[5:0];
    wire signed [5:0] dx1 = dx1_f[5:0];
    wire signed [5:0] dy  = dy_f[5:0];

    wire [4:0] ady  = dy[5]  ? (~dy[4:0]  + 5'd1) : dy[4:0];
    wire [4:0] adx0 = dx0[5] ? (~dx0[4:0] + 5'd1) : dx0[4:0];
    wire [4:0] adx1 = dx1[5] ? (~dx1[4:0] + 5'd1) : dx1[4:0];

    wire dy_in = ~ady[4];       // |dy| <= 15: nam trong cua so 31x31

    // nua do rong cua hang theo |dy|
    reg [3:0] umax_cv;      // bang umax cua OpenCV (HALF_PATCH_SIZE = 15)
    reg [3:0] umax_ci;      // floor(sqrt(225 - dy^2))
    always @(*) begin
        case (ady[3:0])
            4'd0:    begin umax_cv = 4'd15; umax_ci = 4'd15; end
            4'd1:    begin umax_cv = 4'd15; umax_ci = 4'd14; end
            4'd2:    begin umax_cv = 4'd15; umax_ci = 4'd14; end
            4'd3:    begin umax_cv = 4'd15; umax_ci = 4'd14; end
            4'd4:    begin umax_cv = 4'd14; umax_ci = 4'd14; end
            4'd5:    begin umax_cv = 4'd14; umax_ci = 4'd14; end
            4'd6:    begin umax_cv = 4'd14; umax_ci = 4'd13; end
            4'd7:    begin umax_cv = 4'd13; umax_ci = 4'd13; end
            4'd8:    begin umax_cv = 4'd13; umax_ci = 4'd12; end
            4'd9:    begin umax_cv = 4'd12; umax_ci = 4'd12; end
            4'd10:   begin umax_cv = 4'd11; umax_ci = 4'd11; end
            4'd11:   begin umax_cv = 4'd10; umax_ci = 4'd10; end
            4'd12:   begin umax_cv = 4'd9;  umax_ci = 4'd9;  end
            4'd13:   begin umax_cv = 4'd8;  umax_ci = 4'd7;  end
            4'd14:   begin umax_cv = 4'd6;  umax_ci = 4'd5;  end
            default: begin umax_cv = 4'd3;  umax_ci = 4'd0;  end
        endcase
    end

    wire [3:0] umax = USE_UMAX ? umax_cv : umax_ci;

    // mat na tung pixel; |dx| > 15 tu bi loai vi umax <= 15, |dy| > 15 bi loai boi dy_in
    wire mask0 = dy_in && (adx0 <= {1'b0, umax});
    wire mask1 = dy_in && (adx1 <= {1'b0, umax});

    wire [7:0] i0m  = mask0 ? in_data[7:0]  : 8'd0;
    wire [7:0] i1m  = mask1 ? in_data[15:8] : 8'd0;
    wire [8:0] sum9 = {1'b0, i0m} + {1'b0, i1m};

    reg               s_v;
    reg signed [5:0]  s_dy;
    reg signed [5:0]  s_dx0;
    reg        [8:0]  s_sum;       // I0 + I1 (da che mat na)
    reg        [7:0]  s_i1;        // I1 (da che mat na)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_v   <= 1'b0;
            s_dy  <= 6'sd0;
            s_dx0 <= 6'sd0;
            s_sum <= 9'd0;
            s_i1  <= 8'd0;
        end else begin
            s_v <= in_v;
            if (in_v) begin
                s_dy  <= dy;
                s_dx0 <= dx0;
                s_sum <= sum9;
                s_i1  <= i1m;
            end
        end
    end

    // ======================================================
    // Stage 3: 1 bo nhan dung chung, 2 pha lien tiep cho moi word
    //   pha A (s_v = 1) : dy  * (I0+I1)
    //   pha B (ph_b = 1): dx0 * (I0+I1)
    // Moi word cach nhau >= 2 chu ky nen s_* van con hieu luc o ca 2 pha.
    // ======================================================
    reg        ph_b;           // = s_v tre 1 chu ky (dang o pha B; prod_r dang giu dy*S)
    reg        ph_c;           // = ph_b tre 1 chu ky (prod_r dang giu dx0*S)
    reg signed [15:0] prod_r;
    reg        [7:0]  i1_p;

    wire signed [9:0]  mul_a = ph_b ? {{4{s_dx0[5]}}, s_dx0} : {{4{s_dy[5]}}, s_dy};
    wire signed [9:0]  mul_b = {1'b0, s_sum};
    wire signed [19:0] mul_p = mul_a * mul_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ph_b   <= 1'b0;
            ph_c   <= 1'b0;
            prod_r <= 16'sd0;
            i1_p   <= 8'd0;
        end else begin
            ph_b   <= s_v;
            ph_c   <= ph_b;
            prod_r <= mul_p[15:0];
            i1_p   <= s_i1;
        end
    end

    // ======================================================
    // Stage 4: cong don
    //   m01 += dy*S            (khi ph_b)
    //   m10 += dx0*S + I1      (khi ph_c)
    // ======================================================
    wire signed [23:0] prod_ext = {{8{prod_r[15]}}, prod_r};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m01 <= 24'sd0;
            m10 <= 24'sd0;
        end else if (clear) begin
            m01 <= 24'sd0;
            m10 <= 24'sd0;
        end else begin
            if (ph_b) m01 <= m01 + prod_ext;
            if (ph_c) m10 <= m10 + prod_ext + $signed({16'd0, i1_p});
        end
    end

    // ======================================================
    // mom_valid: sau khi patch_done va pipeline da can het
    // ======================================================
    wire pipe_busy = pb_we | in_v | s_v | ph_b | ph_c;
    reg  fin_pend;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fin_pend  <= 1'b0;
            mom_valid <= 1'b0;
        end else begin
            mom_valid <= 1'b0;
            if (patch_done) begin
                fin_pend <= 1'b1;
            end else if (fin_pend && !pipe_busy) begin
                fin_pend  <= 1'b0;
                mom_valid <= 1'b1;
            end
        end
    end

endmodule
