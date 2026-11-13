`timescale 1ns/1ps
module orb_fast9 #(
    parameter X_BITS = 10,
    parameter Y_BITS = 9
) (
    input  wire clk, reset, clear,
    input  wire [7:0] threshold,
    input  wire window_valid,
    output wire window_ready,
    input  wire [391:0] window_data,
    input  wire [X_BITS-1:0] window_x,
    input  wire [Y_BITS-1:0] window_y,
    input  wire [1:0] window_level,
    output reg  fast_valid,
    input  wire fast_ready,
    output reg  [127:0] ring_data,
    output reg  [7:0] center_data,
    output reg  is_corner,
    output reg  [X_BITS-1:0] fast_x,
    output reg  [Y_BITS-1:0] fast_y,
    output reg  [1:0] fast_level
);

    wire advance = !reset && !clear && (!fast_valid || fast_ready);
    assign window_ready = advance;

    // Tâm (3,3)
    wire [7:0] center = window_data[199:192];

    // Ghép 16 pixel vòng tròn: index 0 ở LSB, index 15 ở MSB
    wire [127:0] cycle = {
        window_data[23:16],    // idx 15 (0,2)
        window_data[71:64],    // idx 14 (1,1)
        window_data[119:112],  // idx 13 (2,0)
        window_data[175:168],  // idx 12 (3,0)
        window_data[231:224],  // idx 11 (4,0)
        window_data[295:288],  // idx 10 (5,1)
        window_data[359:352],  // idx  9 (6,2)
        window_data[367:360],  // idx  8 (6,3)
        window_data[375:368],  // idx  7 (6,4)
        window_data[327:320],  // idx  6 (5,5)
        window_data[279:272],  // idx  5 (4,6)
        window_data[223:216],  // idx  4 (3,6)
        window_data[167:160],  // idx  3 (2,6)
        window_data[103:96],   // idx  2 (1,5)
        window_data[39:32],    // idx  1 (0,4)
        window_data[31:24]     // idx  0 (0,3)
    };

    // -----------------------------------------------------------------
    // reg khai báo ở module scope, gán trong always @*
    // -----------------------------------------------------------------
    reg  [15:0] bright;
    reg  [15:0] dark;
    reg  [15:0] arc_pass;
    reg  [23:0] bright_wrap;
    reg  [23:0] dark_wrap;
    reg  signed [8:0] limit;
    reg  signed [8:0] negative_limit;
    reg  signed [8:0] d0, d1, d2, d3, d4, d5, d6, d7;
    reg  signed [8:0] d8, d9, d10, d11, d12, d13, d14, d15;
    reg  corner_now;

    always @* begin
        // Ngưỡng signed
        limit          = $signed({1'b0, threshold});
        negative_limit = -limit;

        // 16 hiệu pixel - tâm
        d0  = $signed({1'b0, cycle[7:0]})    - $signed({1'b0, center});
        d1  = $signed({1'b0, cycle[15:8]})   - $signed({1'b0, center});
        d2  = $signed({1'b0, cycle[23:16]})  - $signed({1'b0, center});
        d3  = $signed({1'b0, cycle[31:24]})  - $signed({1'b0, center});
        d4  = $signed({1'b0, cycle[39:32]})  - $signed({1'b0, center});
        d5  = $signed({1'b0, cycle[47:40]})  - $signed({1'b0, center});
        d6  = $signed({1'b0, cycle[55:48]})  - $signed({1'b0, center});
        d7  = $signed({1'b0, cycle[63:56]})  - $signed({1'b0, center});
        d8  = $signed({1'b0, cycle[71:64]})  - $signed({1'b0, center});
        d9  = $signed({1'b0, cycle[79:72]})  - $signed({1'b0, center});
        d10 = $signed({1'b0, cycle[87:80]})  - $signed({1'b0, center});
        d11 = $signed({1'b0, cycle[95:88]})  - $signed({1'b0, center});
        d12 = $signed({1'b0, cycle[103:96]}) - $signed({1'b0, center});
        d13 = $signed({1'b0, cycle[111:104]})- $signed({1'b0, center});
        d14 = $signed({1'b0, cycle[119:112]})- $signed({1'b0, center});
        d15 = $signed({1'b0, cycle[127:120]})- $signed({1'b0, center});

        // bright / dark
        bright[0]  = d0  > limit;   
        bright[1]  = d1  > limit;   
        bright[2]  = d2  > limit;   
        bright[3]  = d3  > limit;   
        bright[4]  = d4  > limit;   
        bright[5]  = d5  > limit;   
        bright[6]  = d6  > limit;   
        bright[7]  = d7  > limit;   
        bright[8]  = d8  > limit;   
        bright[9]  = d9  > limit;   
        bright[10] = d10 > limit;   
        bright[11] = d11 > limit;   
        bright[12] = d12 > limit;   
        bright[13] = d13 > limit;   
        bright[14] = d14 > limit;   
        bright[15] = d15 > limit;   
		  
		  dark[0]  = d0  < negative_limit;
		  dark[1]  = d1  < negative_limit;
		  dark[2]  = d2  < negative_limit;
		  dark[3]  = d3  < negative_limit;
		  dark[4]  = d4  < negative_limit;
		  dark[5]  = d5  < negative_limit;
		  dark[6]  = d6  < negative_limit;
		  dark[7]  = d7  < negative_limit;
		  dark[8]  = d8  < negative_limit;
		  dark[9]  = d9  < negative_limit;
		  dark[10] = d10 < negative_limit;
		  dark[11] = d11 < negative_limit;
		  dark[12] = d12 < negative_limit; 
		  dark[13] = d13 < negative_limit;
		  dark[14] = d14 < negative_limit;
		  dark[15] = d15 < negative_limit;
		  
        // Wrap 8 bit đầu vào cuối
        bright_wrap = {bright[7:0], bright};
        dark_wrap   = {dark[7:0],   dark};

        // 16 cung 9 pixel
        arc_pass[0]  = (&bright_wrap[8:0])   || (&dark_wrap[8:0]);
        arc_pass[1]  = (&bright_wrap[9:1])   || (&dark_wrap[9:1]);
        arc_pass[2]  = (&bright_wrap[10:2])  || (&dark_wrap[10:2]);
        arc_pass[3]  = (&bright_wrap[11:3])  || (&dark_wrap[11:3]);
        arc_pass[4]  = (&bright_wrap[12:4])  || (&dark_wrap[12:4]);
        arc_pass[5]  = (&bright_wrap[13:5])  || (&dark_wrap[13:5]);
        arc_pass[6]  = (&bright_wrap[14:6])  || (&dark_wrap[14:6]);
        arc_pass[7]  = (&bright_wrap[15:7])  || (&dark_wrap[15:7]);
        arc_pass[8]  = (&bright_wrap[16:8])  || (&dark_wrap[16:8]);
        arc_pass[9]  = (&bright_wrap[17:9])  || (&dark_wrap[17:9]);
        arc_pass[10] = (&bright_wrap[18:10]) || (&dark_wrap[18:10]);
        arc_pass[11] = (&bright_wrap[19:11]) || (&dark_wrap[19:11]);
        arc_pass[12] = (&bright_wrap[20:12]) || (&dark_wrap[20:12]);
        arc_pass[13] = (&bright_wrap[21:13]) || (&dark_wrap[21:13]);
        arc_pass[14] = (&bright_wrap[22:14]) || (&dark_wrap[22:14]);
        arc_pass[15] = (&bright_wrap[23:15]) || (&dark_wrap[23:15]);

        corner_now = |arc_pass;
    end

    // -----------------------------------------------------------------
    // Đăng ký đầu ra
    // -----------------------------------------------------------------
    always @(posedge clk) begin
        if (reset || clear) begin
            fast_valid  <= 1'b0;
            ring_data   <= 128'd0;
            center_data <= 8'd0;
            is_corner   <= 1'b0;
            fast_x      <= 0;
            fast_y      <= 0;
            fast_level  <= 0;
        end else if (advance) begin
            fast_valid <= window_valid;
            if (window_valid) begin
                ring_data   <= cycle;
                center_data <= center;
                is_corner   <= corner_now;
                fast_x      <= window_x;
                fast_y      <= window_y;
                fast_level  <= window_level;
            end
        end
    end

endmodule