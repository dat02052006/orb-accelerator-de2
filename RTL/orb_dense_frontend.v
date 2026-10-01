`timescale 1ns/1ps
// Six synchronous row RAMs; three alignment rows plus autonomous right-tail flush.
module orb_dense_frontend #(parameter MAX_WIDTH=640,MAX_HEIGHT=480,X_BITS=10,Y_BITS=9)(
 input wire clk,reset,start_valid,
 output wire start_ready,
 input wire [X_BITS-1:0] cfg_width,
 input wire [Y_BITS-1:0] cfg_height,
 input wire [1:0] cfg_level,
 output reg busy,done,
 input wire [7:0] pixel_data,
 input wire pixel_valid,
 output wire pixel_ready,
 input wire pixel_sof,pixel_eol,
 output wire [391:0] window_data,
 output wire window_valid,
 input wire window_ready,
 output wire [X_BITS-1:0] center_x,
 output wire [Y_BITS-1:0] center_y,
 output reg [1:0] level_id
);
 localparam REAL=2'd0, FLUSH=2'd1, TAIL=2'd2;
 reg [1:0] phase;
 reg [X_BITS-1:0] width,x;
 reg [Y_BITS-1:0] height,y;
 reg [2:0] write_bank;
 wire legal=cfg_width>=7 && cfg_width<=MAX_WIDTH && cfg_height>=7 && cfg_height<=MAX_HEIGHT && cfg_height<=((1<<Y_BITS)-3) && cfg_level<=2;
 assign start_ready=!reset && !busy && legal;
 wire start_fire=start_valid && start_ready;
 wire advance=!reset && (!window_valid || window_ready);
 assign pixel_ready=busy && phase==REAL && advance;
 wire step_fire=busy && advance && ((phase==REAL && pixel_valid) || phase==FLUSH);
 wire [7:0] step_data=(phase==REAL)?pixel_data:8'd0;
 wire column_valid,window_empty;
 wire [55:0] column_data;
 wire [X_BITS-1:0] column_x;
 wire [Y_BITS-1:0] column_y;
 orb_line_buffer #(.MAX_WIDTH(MAX_WIDTH),.X_BITS(X_BITS),.Y_BITS(Y_BITS)) lines(
  .clk(clk),.reset(reset),.clear(start_fire),.advance(advance),.pixel_fire(step_fire),
  .pixel_data(step_data),.pixel_x(x),.pixel_y(y),.write_bank(write_bank),
  .column_valid(column_valid),.column_data(column_data),.column_x(column_x),.column_y(column_y));
 orb_dense_window #(.X_BITS(X_BITS),.Y_BITS(Y_BITS)) windows(
  .clk(clk),.reset(reset),.clear(start_fire),.advance(advance),.flush_tail(phase==TAIL),
  .width(width),.column_valid(column_valid),.column_data(column_data),.column_x(column_x),.column_y(column_y),
  .window_valid(window_valid),.window_data(window_data),.center_x(center_x),.center_y(center_y),.empty(window_empty));
 always @(posedge clk) begin
  if(reset) begin
   busy<=0; done<=0; phase<=REAL; width<=0; height<=0; x<=0; y<=0; write_bank<=0; level_id<=0;
  end else begin
   done<=0;
   if(start_fire) begin
    busy<=1; phase<=REAL; width<=cfg_width; height<=cfg_height; level_id<=cfg_level;
    x<=0; y<=0; write_bank<=0;
   end else if(busy) begin
    if(step_fire) begin
     if(x==width-1'b1) begin
      x<=0;
      if(write_bank==5) write_bank<=0; else write_bank<=write_bank+1'b1;
      if(phase==REAL && y==height-1'b1) phase<=FLUSH;
      if(phase==FLUSH && y==height+2) phase<=TAIL;
      else y<=y+1'b1;
     end else x<=x+1'b1;
    end
    if(phase==TAIL && !column_valid && window_empty) begin busy<=0; done<=1; end
   end
  end
 end
endmodule
