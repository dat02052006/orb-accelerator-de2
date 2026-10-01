`timescale 1ns/1ps
// Full-image centered container. Out-of-image slots are INVALID, never detector samples.
module orb_dense_window #(parameter X_BITS=10, Y_BITS=9)(
 input wire clk,reset,clear,advance,flush_tail,
 input wire [X_BITS-1:0] width,
 input wire column_valid,
 input wire [55:0] column_data,
 input wire [X_BITS-1:0] column_x,
 input wire [Y_BITS-1:0] column_y,
 output reg window_valid,
 output reg [391:0] window_data,
 output reg [X_BITS-1:0] center_x,
 output reg [Y_BITS-1:0] center_y,
 output wire empty
);
 reg [1:0] tail_count;
 reg [X_BITS-1:0] tail_x;
 reg [Y_BITS-1:0] tail_y;
 reg [47:0] history0;
 reg [55:0] tail0;
 reg [47:0] history1;
 reg [55:0] tail1;
 reg [47:0] history2;
 reg [55:0] tail2;
 reg [47:0] history3;
 reg [55:0] tail3;
 reg [47:0] history4;
 reg [55:0] tail4;
 reg [47:0] history5;
 reg [55:0] tail5;
 reg [47:0] history6;
 reg [55:0] tail6;
 assign empty=!window_valid && tail_count==0;
 wire emit_tail=(tail_count!=0) && ((column_valid && column_x<3) || (!column_valid && flush_tail));
 always @(posedge clk) begin
  if(reset || clear) begin
   window_valid<=0; window_data<=0; center_x<=0; center_y<=0;
   tail_count<=0; tail_x<=0; tail_y<=0;
   history0<=0; tail0<=0;
   history1<=0; tail1<=0;
   history2<=0; tail2<=0;
   history3<=0; tail3<=0;
   history4<=0; tail4<=0;
   history5<=0; tail5<=0;
   history6<=0; tail6<=0;
  end else if(advance) begin
   window_valid<=0;
   if(column_valid) begin
    if(column_x==0) begin
     history0<={column_data[7:0],40'd0};
     history1<={column_data[15:8],40'd0};
     history2<={column_data[23:16],40'd0};
     history3<={column_data[31:24],40'd0};
     history4<={column_data[39:32],40'd0};
     history5<={column_data[47:40],40'd0};
     history6<={column_data[55:48],40'd0};
    end else begin
     history0<={column_data[7:0],history0[47:8]};
     history1<={column_data[15:8],history1[47:8]};
     history2<={column_data[23:16],history2[47:8]};
     history3<={column_data[31:24],history3[47:8]};
     history4<={column_data[39:32],history4[47:8]};
     history5<={column_data[47:40],history5[47:8]};
     history6<={column_data[55:48],history6[47:8]};
    end
    if(column_x>=3 && column_y>=3) begin
     window_valid<=1; center_x<=column_x-3; center_y<=column_y-3;
     window_data[55:0]<={column_data[7:0],history0};
     window_data[111:56]<={column_data[15:8],history1};
     window_data[167:112]<={column_data[23:16],history2};
     window_data[223:168]<={column_data[31:24],history3};
     window_data[279:224]<={column_data[39:32],history4};
     window_data[335:280]<={column_data[47:40],history5};
     window_data[391:336]<={column_data[55:48],history6};
     if(column_x==width-1'b1) begin
      tail_count<=3; tail_x<=width-3; tail_y<=column_y-3;
      tail0<={column_data[7:0],history0};
      tail1<={column_data[15:8],history1};
      tail2<={column_data[23:16],history2};
      tail3<={column_data[31:24],history3};
      tail4<={column_data[39:32],history4};
      tail5<={column_data[47:40],history5};
      tail6<={column_data[55:48],history6};
     end
    end
   end
   if(emit_tail) begin
    window_valid<=1; center_x<=tail_x; center_y<=tail_y;
    tail_count<=tail_count-1'b1; tail_x<=tail_x+1'b1;
    window_data[55:0]<={8'd0,tail0[55:8]};
    tail0<={8'd0,tail0[55:8]};
    window_data[111:56]<={8'd0,tail1[55:8]};
    tail1<={8'd0,tail1[55:8]};
    window_data[167:112]<={8'd0,tail2[55:8]};
    tail2<={8'd0,tail2[55:8]};
    window_data[223:168]<={8'd0,tail3[55:8]};
    tail3<={8'd0,tail3[55:8]};
    window_data[279:224]<={8'd0,tail4[55:8]};
    tail4<={8'd0,tail4[55:8]};
    window_data[335:280]<={8'd0,tail5[55:8]};
    tail5<={8'd0,tail5[55:8]};
    window_data[391:336]<={8'd0,tail6[55:8]};
    tail6<={8'd0,tail6[55:8]};
   end
  end
 end
endmodule
