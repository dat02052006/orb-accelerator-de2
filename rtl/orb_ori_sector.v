//------------------------------------------------------------------------------
// Moments -> sector (0..29). Multi-cycle binary search.
// FSM divided into 3 blocks (classic style):
//   1. next-state combinational logic
//   2. sequential state + datapath register update
//   3. pure combinational output logic
//------------------------------------------------------------------------------
module orb_ori_sector (
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    input  wire signed [21:0] m10,
    input  wire signed [21:0] m01,
    output reg                done,
    output reg         [4:0]  sector
);

    // State encoding
    localparam [2:0] S_IDLE = 3'd0;
    localparam [2:0] S_LOOK = 3'd1;
    localparam [2:0] S_MUL  = 3'd2;
    localparam [2:0] S_CMP  = 3'd3;
    localparam [2:0] S_END  = 3'd4;

    // State registers
    reg [2:0] st;
    reg [2:0] next_st;

    // Datapath registers (updated only in sequential block)
    reg signed [21:0] fx, fy;
    reg [4:0]         base;
    reg               zero;
    reg [3:0]         lo, hi;
    reg signed [32:0] py, px;

    // Combinational intermediate signals (computed in output logic block)
    reg signed [10:0] cos_c, sin_c;
    wire [4:0]         sum   = {1'b0, lo} + {1'b0, hi};
    wire [3:0]         mid   = sum[4:1];
    wire signed [33:0] cross = py - px;
    wire               neg   = m01[21] | ((m01 == 22'sd0) & m10[21]);

    //----------------------------------------------------------------------
    // 1. Next-state combinational logic
    //----------------------------------------------------------------------
    always @(*) begin
        next_st = st;   // default hold
        case (st)
            S_IDLE: begin
                if (start)
                    next_st = S_LOOK;
                else
                    next_st = S_IDLE;
            end

            S_LOOK: begin
                if (zero || ((hi - lo) == 4'd1))
                    next_st = S_END;
                else
                    next_st = S_MUL;
            end

            S_MUL: begin
                next_st = S_CMP;
            end

            S_CMP: begin
                next_st = S_LOOK;
            end

            S_END: begin
                next_st = S_IDLE;
            end

            default: begin
                next_st = S_IDLE;
            end
        endcase
    end

    //----------------------------------------------------------------------
    // 2. Sequential state + datapath register update
    //    (all sequential elements live here)
    //----------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            st     <= S_IDLE;
            done   <= 1'b0;
            sector <= 5'd0;
            fx     <= 22'sd0;
            fy     <= 22'sd0;
            base   <= 5'd0;
            zero   <= 1'b0;
            lo     <= 4'd0;
            hi     <= 4'd0;
            py     <= 33'sd0;
            px     <= 33'sd0;
        end else begin
            // state update
            st <= next_st;

            // default: done is a 1-cycle pulse
            done <= 1'b0;

            case (st)
                S_IDLE: begin
                    if (start) begin
                        fx   <= neg ? -m10 : m10;
                        fy   <= neg ? -m01 : m01;
                        base <= neg ? 5'd15 : 5'd0;
                        zero <= (m10 == 22'sd0) && (m01 == 22'sd0);
                        lo   <= 4'd0;
                        hi   <= 4'd15;
                    end
                end

                S_MUL: begin
                    // capture product (cos_c/sin_c are combinational from mid)
                    py <= fy * cos_c;
                    px <= fx * sin_c;
                end

                S_CMP: begin
                    // cross = py - px; if ~cross[33] then vector past boundary
                    if (~cross[33])
                        lo <= mid;
                    else
                        hi <= mid;
                end

                S_END: begin
                    sector <= base + {1'b0, lo};
                    done   <= 1'b1;
                end

                default: begin
                    // hold
                end
            endcase
        end
    end

    //----------------------------------------------------------------------
    // 3. Pure combinational output / intermediate logic
    //    (no sequential assignment here)
    //----------------------------------------------------------------------
    always @(*) begin
        // default values
        cos_c = 11'sd0;
        sin_c = 11'sd0;

        // cos/sin of 12*mid degrees (Q10, scaled by 1024)
        // only meaningful when st == S_LOOK / S_MUL, but always computed
        case (mid)
            4'd1: begin
                cos_c =  11'sd1002;
                sin_c =  11'sd213;
            end
            4'd2: begin
                cos_c =  11'sd935;
                sin_c =  11'sd416;
            end
            4'd3: begin
                cos_c =  11'sd828;
                sin_c =  11'sd602;
            end
            4'd4: begin
                cos_c =  11'sd685;
                sin_c =  11'sd761;
            end
            4'd5: begin
                cos_c =  11'sd512;
                sin_c =  11'sd887;
            end
            4'd6: begin
                cos_c =  11'sd316;
                sin_c =  11'sd974;
            end
            4'd7: begin
                cos_c =  11'sd107;
                sin_c =  11'sd1018;
            end
            4'd8: begin
                cos_c = -11'sd107;
                sin_c =  11'sd1018;
            end
            4'd9: begin
                cos_c = -11'sd316;
                sin_c =  11'sd974;
            end
            4'd10: begin
                cos_c = -11'sd512;
                sin_c =  11'sd887;
            end
            4'd11: begin
                cos_c = -11'sd685;
                sin_c =  11'sd761;
            end
            4'd12: begin
                cos_c = -11'sd828;
                sin_c =  11'sd602;
            end
            4'd13: begin
                cos_c = -11'sd935;
                sin_c =  11'sd416;
            end
            default: begin   // mid == 14 or 0
                cos_c = -11'sd1002;
                sin_c =  11'sd213;
            end
        endcase
    end

endmodule
