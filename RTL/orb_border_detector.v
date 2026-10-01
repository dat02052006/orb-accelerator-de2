`timescale 1ns/1ps
// Four-stage border-only detector; real directional 3x3 neighborhood, no artificial pixels.
module orb_border_detector #(parameter X_BITS=10,Y_BITS=9)(
 input wire clk,reset,clear,
 input wire [X_BITS-1:0] width,
 input wire [Y_BITS-1:0] height,
 input wire [7:0] threshold,
 input wire in_valid,
 output wire in_ready,
 input wire [391:0] window_data,
 input wire [X_BITS-1:0] in_x,
 input wire [Y_BITS-1:0] in_y,
 input wire [1:0] in_level,
 output reg out_valid,
 input wire out_ready,
 output reg [7:0] score,
 output reg corner,
 output reg [X_BITS-1:0] out_x,
 output reg [Y_BITS-1:0] out_y,
 output reg [1:0] out_level,
 output wire empty
);
 wire advance=!reset && !clear && (!out_valid || out_ready);
 assign in_ready=advance;
 reg v1,v2,v3;
 assign empty=!v1 && !v2 && !v3 && !out_valid;
 reg [55:0] row0;
 reg [55:0] row1;
 reg [55:0] row2;
 always @* begin
  if(in_y<3) begin
   row0=window_data[223:168];
   row1=window_data[279:224];
   row2=window_data[335:280];
  end else if(in_y>height-4) begin
   row0=window_data[223:168];
   row1=window_data[167:112];
   row2=window_data[111:56];
  end else begin
   row0=window_data[167:112];
   row1=window_data[223:168];
   row2=window_data[279:224];
  end
 end
 reg [7:0] q0;
 reg [7:0] q1;
 reg [7:0] q2;
 reg [7:0] q3;
 reg [7:0] q4;
 reg [7:0] q5;
 reg [7:0] q6;
 reg [7:0] q7;
 reg [7:0] q8;
 always @* begin
  if(in_x<3) begin
   q0=row0[31:24];
   q1=row0[39:32];
   q2=row0[47:40];
   q3=row1[31:24];
   q4=row1[39:32];
   q5=row1[47:40];
   q6=row2[31:24];
   q7=row2[39:32];
   q8=row2[47:40];
  end else if(in_x>width-4) begin
   q0=row0[31:24];
   q1=row0[23:16];
   q2=row0[15:8];
   q3=row1[31:24];
   q4=row1[23:16];
   q5=row1[15:8];
   q6=row2[31:24];
   q7=row2[23:16];
   q8=row2[15:8];
  end else begin
   q0=row0[23:16];
   q1=row0[31:24];
   q2=row0[39:32];
   q3=row1[23:16];
   q4=row1[31:24];
   q5=row1[39:32];
   q6=row2[23:16];
   q7=row2[31:24];
   q8=row2[39:32];
  end
 end
 wire [9:0] h0,cell_v0;
 reg [9:0] hr0,vr0;
 orb_border_cell cell0(.a(q0),.b(q1),.c(q3),.d(q4),.h(h0),.v(cell_v0));
 wire [9:0] h1,cell_v1;
 reg [9:0] hr1,vr1;
 orb_border_cell cell1(.a(q1),.b(q2),.c(q4),.d(q5),.h(h1),.v(cell_v1));
 wire [9:0] h2,cell_v2;
 reg [9:0] hr2,vr2;
 orb_border_cell cell2(.a(q3),.b(q4),.c(q6),.d(q7),.h(h2),.v(cell_v2));
 wire [9:0] h3,cell_v3;
 reg [9:0] hr3,vr3;
 orb_border_cell cell3(.a(q4),.b(q5),.c(q7),.d(q8),.h(h3),.v(cell_v3));
 wire [9:0] max_h,max_v;
 orb_border_max4 mh(.a(hr0),.b(hr1),.c(hr2),.d(hr3),.m(max_h));
 orb_border_max4 mv(.a(vr0),.b(vr1),.c(vr2),.d(vr3),.m(max_v));
 wire [9:0] strength=(max_h<max_v)?max_h:max_v;
 reg [7:0] margin2,threshold1,threshold2,score3;
 reg corner3;
 reg [X_BITS-1:0] x1;
 reg [Y_BITS-1:0] y1;
 reg [1:0] level1;
 reg [X_BITS-1:0] x2;
 reg [Y_BITS-1:0] y2;
 reg [1:0] level2;
 reg [X_BITS-1:0] x3;
 reg [Y_BITS-1:0] y3;
 reg [1:0] level3;
 always @(posedge clk) begin
  if(reset || clear) begin
   v1<=0; v2<=0; v3<=0; out_valid<=0; margin2<=0; threshold1<=0; threshold2<=0; score3<=0; corner3<=0;
   score<=0; corner<=0; out_x<=0; out_y<=0; out_level<=0;
   hr0<=0; vr0<=0;
   hr1<=0; vr1<=0;
   hr2<=0; vr2<=0;
   hr3<=0; vr3<=0;
   x1<=0; y1<=0; level1<=0;
   x2<=0; y2<=0; level2<=0;
   x3<=0; y3<=0; level3<=0;
  end else if(advance) begin
   v1<=in_valid; v2<=v1; v3<=v2; out_valid<=v3;
   if(in_valid) begin
    threshold1<=threshold; x1<=in_x; y1<=in_y; level1<=in_level;
    hr0<=h0; vr0<=cell_v0;
    hr1<=h1; vr1<=cell_v1;
    hr2<=h2; vr2<=cell_v2;
    hr3<=h3; vr3<=cell_v3;
   end
   if(v1) begin
    margin2<=strength[8:1]; threshold2<=threshold1; x2<=x1; y2<=y1; level2<=level1;
   end
   if(v2) begin
    corner3<=margin2>threshold2;
    score3<=(margin2>threshold2)?margin2-8'd1:8'd0;
    x3<=x2; y3<=y2; level3<=level2;
   end
   if(v3) begin
    score<=score3; corner<=corner3; out_x<=x3; out_y<=y3; out_level<=level3;
   end
  end
 end
endmodule
