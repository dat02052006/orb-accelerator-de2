`timescale 1ns/1ps
// Full-raster NMS: compare ONLY existing neighbors. Equal scores still suppress.
module orb_nms3x3_v2 #(parameter MAX_WIDTH=640,X_BITS=10,Y_BITS=9)(
 input wire clk,reset,clear,
 input wire [X_BITS-1:0] cfg_width,
 input wire [Y_BITS-1:0] cfg_height,
 input wire [1:0] cfg_level,
 input wire score_valid,
 output wire score_ready,
 input wire [7:0] score,
 input wire score_corner,
 input wire [X_BITS-1:0] score_x,
 input wire [Y_BITS-1:0] score_y,
 input wire [1:0] score_level,
 output reg keypoint_valid,
 input wire keypoint_ready,
 output reg [7:0] keypoint_score,
 output reg [X_BITS-1:0] keypoint_x,
 output reg [Y_BITS-1:0] keypoint_y,
 output reg [1:0] keypoint_level,
 output reg done,
 output wire empty
);
 localparam REAL=2'd0,FLUSH=2'd1,TAIL=2'd2,IDLE=2'd3;
 reg [1:0] phase;
 reg [X_BITS-1:0] width,flush_x;
 reg [Y_BITS-1:0] height;
 reg [1:0] level;
 reg bank;
 wire advance=!reset && !clear && (!keypoint_valid || keypoint_ready);
 assign score_ready=advance && phase==REAL;
 wire fire=advance && ((phase==REAL && score_valid) || phase==FLUSH);
 wire [X_BITS-1:0] addr=phase==REAL?score_x:flush_x;
 wire [Y_BITS-1:0] step_y=phase==REAL?score_y:height;
 wire [8:0] data=phase==REAL?{score_corner,score}:9'd0;
 wire [8:0] q0,q1;
 orb_nms_row_ram #(.MAX_WIDTH(MAX_WIDTH),.X_BITS(X_BITS)) row0(
  .clk(clk),.enable(fire),.write_enable(!bank),.address(addr),.write_data(data),.read_data(q0));
 orb_nms_row_ram #(.MAX_WIDTH(MAX_WIDTH),.X_BITS(X_BITS)) row1(
  .clk(clk),.enable(fire),.write_enable(bank),.address(addr),.write_data(data),.read_data(q1));
 reg column_valid,column_bank;
 reg [8:0] column_bottom;
 reg [X_BITS-1:0] column_x;
 reg [Y_BITS-1:0] column_y;
 wire [8:0] column_top=column_bank?q1:q0;
 wire [8:0] column_mid=column_bank?q0:q1;
 reg [17:0] ht,hm,hb;
 reg [26:0] tt,tm,tb;
 reg tail_valid;
 reg [Y_BITS-1:0] tail_y;
 wire emit_tail=tail_valid && ((column_valid && column_x==0) || (!column_valid && phase==TAIL));
 wire emit_normal=column_valid && column_x>=1 && column_y>=1;
 wire decision_valid=emit_tail || emit_normal;
 wire [80:0] box=emit_tail?{9'd0,tb[26:9],9'd0,tm[26:9],9'd0,tt[26:9]}:
                           {column_bottom,hb,column_mid,hm,column_top,ht};
 wire [X_BITS-1:0] dx=emit_tail?width-1'b1:column_x-1'b1;
 wire [Y_BITS-1:0] dy=emit_tail?tail_y:column_y-1'b1;
 wire [7:0] cs=box[43:36];
 wire keep=box[44] &&
  (dx==0 || dy==0 || cs>box[7:0]) &&
  (dy==0 || cs>box[16:9]) &&
  (dx==width-1'b1 || dy==0 || cs>box[25:18]) &&
  (dx==0 || cs>box[34:27]) &&
  (dx==width-1'b1 || cs>box[52:45]) &&
  (dx==0 || dy==height-1'b1 || cs>box[61:54]) &&
  (dy==height-1'b1 || cs>box[70:63]) &&
  (dx==width-1'b1 || dy==height-1'b1 || cs>box[79:72]);
 assign empty=phase==IDLE && !column_valid && !tail_valid && !keypoint_valid;
 always @(posedge clk) begin
  if(reset || clear) begin
   phase<=clear?REAL:IDLE; width<=cfg_width; height<=cfg_height; level<=cfg_level;
   bank<=0; flush_x<=0; column_valid<=0; column_bank<=0; column_bottom<=0;
   column_x<=0; column_y<=0; ht<=0; hm<=0; hb<=0; tt<=0; tm<=0; tb<=0; tail_valid<=0; tail_y<=0;
   keypoint_valid<=0; keypoint_score<=0; keypoint_x<=0; keypoint_y<=0; keypoint_level<=0; done<=0;
  end else begin
   done<=0;
   if(advance) begin
    column_valid<=fire;
    if(fire) begin
     column_bank<=bank; column_bottom<=data; column_x<=addr; column_y<=step_y;
     if(addr==width-1'b1) bank<=!bank;
     if(phase==REAL && score_x==width-1'b1 && score_y==height-1'b1) begin phase<=FLUSH; flush_x<=0; end
     if(phase==FLUSH) begin
      if(flush_x==width-1'b1) phase<=TAIL; else flush_x<=flush_x+1'b1;
     end
    end
    keypoint_valid<=0;
    if(column_valid) begin
     if(column_x==0) begin ht<={column_top,9'd0}; hm<={column_mid,9'd0}; hb<={column_bottom,9'd0}; end
     else begin ht<={column_top,ht[17:9]}; hm<={column_mid,hm[17:9]}; hb<={column_bottom,hb[17:9]}; end
     if(column_x==width-1'b1 && column_y>=1) begin
      tail_valid<=1; tail_y<=column_y-1'b1;
      tt<={column_top,ht}; tm<={column_mid,hm}; tb<={column_bottom,hb};
     end
    end
    if(emit_tail) tail_valid<=0;
    if(decision_valid && keep) begin
     keypoint_valid<=1; keypoint_score<=cs; keypoint_x<=dx; keypoint_y<=dy; keypoint_level<=level;
    end
    if(phase==TAIL && !column_valid && !tail_valid) begin phase<=IDLE; done<=1; end
   end
  end
 end
endmodule
