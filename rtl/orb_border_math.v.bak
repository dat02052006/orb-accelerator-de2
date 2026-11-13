`timescale 1ns/1ps
// Gradient orientation dominance on a real 2x2 cell. Signed sums cannot overflow.
module orb_border_cell(input wire [7:0] a,b,c,d,output wire [9:0] h,v);
 wire signed [8:0] dx0=$signed({1'b0,b})-$signed({1'b0,a});
 wire signed [8:0] dx1=$signed({1'b0,d})-$signed({1'b0,c});
 wire signed [8:0] dy0=$signed({1'b0,c})-$signed({1'b0,a});
 wire signed [8:0] dy1=$signed({1'b0,d})-$signed({1'b0,b});
 wire signed [9:0] gx=$signed({dx0[8],dx0})+$signed({dx1[8],dx1});
 wire signed [9:0] gy=$signed({dy0[8],dy0})+$signed({dy1[8],dy1});
 wire [9:0] ax=gx[9]?-gx:gx;
 wire [9:0] ay=gy[9]?-gy:gy;
 assign h=(ax>ay)?ax-ay:10'd0;
 assign v=(ay>ax)?ay-ax:10'd0;
endmodule
module orb_border_max4(input wire [9:0] a,b,c,d,output wire [9:0] m);
 wire [9:0] ab=(a>b)?a:b;
 wire [9:0] cd=(c>d)?c:d;
 assign m=(ab>cd)?ab:cd;
endmodule
