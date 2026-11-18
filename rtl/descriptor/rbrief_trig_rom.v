// Tu dong sinh boi gen_rbrief_rot.py - KHONG sua tay.
// cos/sin cua 12*bin do (sign=+1), co dau, FRAC=16 (18 bit); tie = luat hoa 0.5 cua bin
//   0: half-even  1: half-up  2: half-down  3: xa 0  4: gan 0
module rbrief_trig_rom (
    input  wire [4:0]          bin,
    output reg  signed [17:0] cos_q,
    output reg  signed [17:0] sin_q,
    output reg  [2:0]          tie_q
);
    always @(*) begin
        case (bin)
            5'd0: begin cos_q = 18'sd65536; sin_q = 18'sd0; tie_q = 3'd0; end
            5'd1: begin cos_q = 18'sd64104; sin_q = 18'sd13626; tie_q = 3'd0; end
            5'd2: begin cos_q = 18'sd59870; sin_q = 18'sd26656; tie_q = 3'd0; end
            5'd3: begin cos_q = 18'sd53020; sin_q = 18'sd38521; tie_q = 3'd0; end
            5'd4: begin cos_q = 18'sd43852; sin_q = 18'sd48703; tie_q = 3'd0; end
            5'd5: begin cos_q = 18'sd32768; sin_q = 18'sd56756; tie_q = 3'd4; end
            5'd6: begin cos_q = 18'sd20252; sin_q = 18'sd62328; tie_q = 3'd0; end
            5'd7: begin cos_q = 18'sd6850; sin_q = 18'sd65177; tie_q = 3'd0; end
            5'd8: begin cos_q = -18'sd6850; sin_q = 18'sd65177; tie_q = 3'd0; end
            5'd9: begin cos_q = -18'sd20252; sin_q = 18'sd62328; tie_q = 3'd0; end
            5'd10: begin cos_q = -18'sd32768; sin_q = 18'sd56756; tie_q = 3'd3; end
            5'd11: begin cos_q = -18'sd43852; sin_q = 18'sd48703; tie_q = 3'd0; end
            5'd12: begin cos_q = -18'sd53020; sin_q = 18'sd38521; tie_q = 3'd0; end
            5'd13: begin cos_q = -18'sd59870; sin_q = 18'sd26656; tie_q = 3'd0; end
            5'd14: begin cos_q = -18'sd64104; sin_q = 18'sd13626; tie_q = 3'd0; end
            5'd15: begin cos_q = -18'sd65536; sin_q = 18'sd0; tie_q = 3'd0; end
            5'd16: begin cos_q = -18'sd64104; sin_q = -18'sd13626; tie_q = 3'd0; end
            5'd17: begin cos_q = -18'sd59870; sin_q = -18'sd26656; tie_q = 3'd0; end
            5'd18: begin cos_q = -18'sd53020; sin_q = -18'sd38521; tie_q = 3'd0; end
            5'd19: begin cos_q = -18'sd43852; sin_q = -18'sd48703; tie_q = 3'd0; end
            5'd20: begin cos_q = -18'sd32768; sin_q = -18'sd56756; tie_q = 3'd4; end
            5'd21: begin cos_q = -18'sd20252; sin_q = -18'sd62328; tie_q = 3'd0; end
            5'd22: begin cos_q = -18'sd6850; sin_q = -18'sd65177; tie_q = 3'd0; end
            5'd23: begin cos_q = 18'sd6850; sin_q = -18'sd65177; tie_q = 3'd0; end
            5'd24: begin cos_q = 18'sd20252; sin_q = -18'sd62328; tie_q = 3'd0; end
            5'd25: begin cos_q = 18'sd32768; sin_q = -18'sd56756; tie_q = 3'd4; end
            5'd26: begin cos_q = 18'sd43852; sin_q = -18'sd48703; tie_q = 3'd0; end
            5'd27: begin cos_q = 18'sd53020; sin_q = -18'sd38521; tie_q = 3'd0; end
            5'd28: begin cos_q = 18'sd59870; sin_q = -18'sd26656; tie_q = 3'd0; end
            5'd29: begin cos_q = 18'sd64104; sin_q = -18'sd13626; tie_q = 3'd0; end
            default: begin cos_q = 18'sd0; sin_q = 18'sd0; tie_q = 3'd0; end
        endcase
    end
endmodule
