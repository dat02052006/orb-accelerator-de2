`timescale 1ns/1ps
// Ordered fork/join. Eight one-bit source tags; both data paths are four stages.
module orb_hybrid_detector #(parameter X_BITS=10,Y_BITS=9)(
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
 output wire map_valid,
 input wire map_ready,
 output wire [7:0] map_score,
 output wire map_corner,map_source,
 output wire [X_BITS-1:0] map_x,
 output wire [Y_BITS-1:0] map_y,
 output wire [1:0] map_level,
 output wire empty
);
 reg [7:0] routes;
 reg [3:0] count;
 wire border_select=in_x<3 || in_x>width-4 || in_y<3 || in_y>height-4;
 wire fast_in_ready,border_in_ready;
 wire fast_valid,fast_ready,fast_corner;
 wire [127:0] ring;
 wire [7:0] center;
 wire [X_BITS-1:0] fx,sx,bx;
 wire [Y_BITS-1:0] fy,sy,by;
 wire [1:0] fl,sl,bl;
 wire score_valid,score_corner,score_ready,score_empty;
 wire [7:0] score;
 wire border_valid,border_corner,border_ready,border_empty;
 wire [7:0] border_score;
 assign map_source=routes[0];
 assign map_valid=!reset && !clear && count!=0 && (routes[0]?border_valid:score_valid);
 assign map_score=routes[0]?border_score:score;
 assign map_corner=routes[0]?border_corner:score_corner;
 assign map_x=routes[0]?bx:sx;
 assign map_y=routes[0]?by:sy;
 assign map_level=routes[0]?bl:sl;
 wire pop=map_valid && map_ready;
 wire space=count<8 || pop;
 assign in_ready=!reset && !clear && space && (border_select?border_in_ready:fast_in_ready);
 wire push=in_valid && in_ready;
 assign score_ready=!reset && !clear && count!=0 && !routes[0] && map_ready;
 assign border_ready=!reset && !clear && count!=0 && routes[0] && map_ready;
 assign empty=count==0 && !fast_valid && score_empty && border_empty;
 orb_fast9 #(.X_BITS(X_BITS),.Y_BITS(Y_BITS)) fast(
  .clk(clk),.reset(reset),.clear(clear),.threshold(threshold),
  .window_valid(in_valid && space && !border_select && !reset && !clear),.window_ready(fast_in_ready),
  .window_data(window_data),.window_x(in_x),.window_y(in_y),.window_level(in_level),
  .fast_valid(fast_valid),.fast_ready(fast_ready),.ring_data(ring),.center_data(center),
  .is_corner(fast_corner),.fast_x(fx),.fast_y(fy),.fast_level(fl));
 orb_fast_score #(.X_BITS(X_BITS),.Y_BITS(Y_BITS)) scorer(
  .clk(clk),.reset(reset),.clear(clear),.fast_valid(fast_valid),.fast_ready(fast_ready),
  .ring_data(ring),.center_data(center),.is_corner(fast_corner),.fast_x(fx),.fast_y(fy),.fast_level(fl),
  .score_valid(score_valid),.score_ready(score_ready),.score(score),.score_corner(score_corner),
  .score_x(sx),.score_y(sy),.score_level(sl),.empty(score_empty));
 orb_border_detector #(.X_BITS(X_BITS),.Y_BITS(Y_BITS)) border(
  .clk(clk),.reset(reset),.clear(clear),.width(width),.height(height),.threshold(threshold),
  .in_valid(in_valid && space && border_select && !reset && !clear),.in_ready(border_in_ready),
  .window_data(window_data),.in_x(in_x),.in_y(in_y),.in_level(in_level),
  .out_valid(border_valid),.out_ready(border_ready),.score(border_score),.corner(border_corner),
  .out_x(bx),.out_y(by),.out_level(bl),.empty(border_empty));
 always @(posedge clk) begin
  if(reset || clear) begin routes<=0; count<=0; end
  else begin
   case({push,pop})
    2'b01: begin routes<={1'b0,routes[7:1]}; count<=count-1'b1; end
    2'b10: begin
     count<=count+1'b1;
     case(count)
      4'd0: routes[0]<=border_select;
      4'd1: routes[1]<=border_select;
      4'd2: routes[2]<=border_select;
      4'd3: routes[3]<=border_select;
      4'd4: routes[4]<=border_select;
      4'd5: routes[5]<=border_select;
      4'd6: routes[6]<=border_select;
      4'd7: routes[7]<=border_select;
      default: routes<=routes;
     endcase
    end
    2'b11: begin
     case(count)
      4'd1: routes<={7'd0,border_select};
      4'd2: routes<={6'd0,border_select,routes[1]};
      4'd3: routes<={5'd0,border_select,routes[2:1]};
      4'd4: routes<={4'd0,border_select,routes[3:1]};
      4'd5: routes<={3'd0,border_select,routes[4:1]};
      4'd6: routes<={2'd0,border_select,routes[5:1]};
      4'd7: routes<={1'd0,border_select,routes[6:1]};
      4'd8: routes<={border_select,routes[7:1]};
      default: routes<=routes;
     endcase
    end
    default: begin routes<=routes; count<=count; end
   endcase
  end
 end
endmodule
