module centroid_angle (
    input  wire               clk,
    input  wire               rst_n,

    input  wire               start,        // xung bat dau; m10/m01 phai hop le tai chu ky nay
    input  wire signed [23:0] m10,
    input  wire signed [23:0] m01,

    output wire               busy,
    output wire               done,         // xung 1 chu ky; angle_bin hop le tu day den start ke tiep
    output wire [4:0]         angle_bin     // 0..29, bin k co tam o 12*k do (tron ve gan nhat)
);

    // ------------------------------------------------------------------
    // Y tuong: theta = atan2(m01, m10), luong tu hoa 30 bin, moi bin 12 do.
    //  1) Gap ve goc phan tu thu nhat: alpha = atan(|m01| / |m10|), 0..90 do
    //  2) q = so nguong nam duoi alpha trong {6,18,30,42,54,66,78} do (0..7),
    //     tim bang 3 buoc nhi phan, moi buoc so |m01| voi |m10| * tan(nguong)
    //     (khong chia):  |m01| * 2^13 > |m10| * C,  C = round(tan * 2^13)
    //  3) Lay doi xung theo dau cua m10, m01:
    //       m10>=0, m01>=0 : bin = q
    //       m10>=0, m01< 0 : bin = (30 - q) mod 30
    //       m10< 0, m01>=0 : bin = 15 - q
    //       m10< 0, m01< 0 : bin = 15 + q
    // m10 = m01 = 0 cho bin 0 (giong atan2(0,0) = 0).
    // ------------------------------------------------------------------

    // ---------------- state encoding ----------------
    localparam [1:0] S_IDLE = 2'd0,
                     S_MUL  = 2'd1,
                     S_CMP  = 2'd2,
                     S_DONE = 2'd3;

    reg [1:0] state, next_state;

    // ---------------- datapath registers ----------------
    reg [23:0] abs_x;           // |m10|
    reg [23:0] abs_y;           // |m01|
    reg        neg_x;
    reg        neg_y;
    reg [1:0]  step;            // 0..2
    reg [2:0]  q;               // {b2, b1, b0}
    reg [39:0] prod_r;          // |m10| * C

    // nguong cua buoc hien tai: chi so i = 1..7 ung voi goc 6, 18, 30, 42, 54, 66, 78 do
    //   buoc 0: i = 4            (42 do)
    //   buoc 1: i = {b2, 1, 0}   (18 hoac 66 do)
    //   buoc 2: i = {b2, b1, 1}  (6, 30, 54 hoac 78 do)
    reg [2:0] thr_idx;
    always @(*) begin
        case (step)
            2'd0:    thr_idx = 3'd4;
            2'd1:    thr_idx = {q[2], 1'b1, 1'b0};
            default: thr_idx = {q[2], q[1], 1'b1};
        endcase
    end

    // C = round(tan(nguong) * 2^13)
    reg [15:0] thr_c;
    always @(*) begin
        case (thr_idx)
            3'd1:    thr_c = 16'd861;       // tan  6
            3'd2:    thr_c = 16'd2662;      // tan 18
            3'd3:    thr_c = 16'd4730;      // tan 30
            3'd4:    thr_c = 16'd7376;      // tan 42
            3'd5:    thr_c = 16'd11275;     // tan 54
            3'd6:    thr_c = 16'd18400;     // tan 66
            3'd7:    thr_c = 16'd38540;     // tan 78
            default: thr_c = 16'd0;
        endcase
    end

    wire [39:0] mul_p = abs_x * thr_c;
    wire [39:0] lhs   = {3'd0, abs_y, 13'd0};
    wire        gt    = (lhs > prod_r);

    // ======================================================
    // FSM phan 1: next-state transition logic
    // ======================================================
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start) next_state = S_MUL;
            end
            S_MUL:  next_state = S_CMP;
            S_CMP:  begin
                if (step == 2'd2) next_state = S_DONE;
                else              next_state = S_MUL;
            end
            S_DONE: next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end

    // ======================================================
    // FSM phan 2: state register update
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    // ======================================================
    // Datapath
    // ======================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            abs_x  <= 24'd0;
            abs_y  <= 24'd0;
            neg_x  <= 1'b0;
            neg_y  <= 1'b0;
            step   <= 2'd0;
            q      <= 3'd0;
            prod_r <= 40'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start) begin
                        neg_x <= m10[23];
                        neg_y <= m01[23];
                        abs_x <= m10[23] ? (~m10 + 24'd1) : m10;
                        abs_y <= m01[23] ? (~m01 + 24'd1) : m01;
                        step  <= 2'd0;
                        q     <= 3'd0;
                    end
                end
                S_MUL: begin
                    prod_r <= mul_p;
                end
                S_CMP: begin
                    case (step)
                        2'd0:    q[2] <= gt;
                        2'd1:    q[1] <= gt;
                        default: q[0] <= gt;
                    endcase
                    if (step != 2'd2) step <= step + 2'd1;
                end
                default: ;
            endcase
        end
    end

    // ======================================================
    // FSM phan 3: combinational output logic
    // ======================================================
    reg [4:0] bin_c;
    always @(*) begin
        case ({neg_x, neg_y})
            2'b00:   bin_c = {2'b00, q};
            2'b01:   bin_c = (q == 3'd0) ? 5'd0 : (5'd30 - {2'b00, q});
            2'b10:   bin_c = 5'd15 - {2'b00, q};
            default: bin_c = 5'd15 + {2'b00, q};
        endcase
    end

    assign angle_bin = bin_c;
    assign busy      = (state != S_IDLE);
    assign done      = (state == S_DONE);

endmodule
