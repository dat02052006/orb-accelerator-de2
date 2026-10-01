`timescale 1ns/1ps
module tb_border;
 reg clk=0; always #10 clk=~clk;
 reg reset=1,clear=0,in_valid=0,out_ready=1;
 wire in_ready,out_valid,corner,empty;
 reg [7:0] threshold=0;
 reg [391:0] window_data=0;
 reg [9:0] in_x=0; reg [8:0] in_y=0; reg [1:0] in_level=0;
 wire [7:0] score;wire [9:0] out_x;wire [8:0] out_y;wire [1:0] out_level;
 orb_border_detector dut(.clk(clk),.reset(reset),.clear(clear),.width(10'd13),.height(9'd13),.threshold(threshold),
 .in_valid(in_valid),.in_ready(in_ready),.window_data(window_data),.in_x(in_x),.in_y(in_y),.in_level(in_level),
 .out_valid(out_valid),.out_ready(out_ready),.score(score),.corner(corner),.out_x(out_x),.out_y(out_y),.out_level(out_level),.empty(empty));
 reg [31:0] params[0:123];reg [391:0] windows[0:2047];reg [26:0] meta[0:2047];reg [8:0] expected[0:2047];
 integer count,received=0,i,cycles=0;reg [31:0] rng=32'h671afd32;
 reg stalled=0;reg [30:0] held;
 task fail;input [511:0] message;begin $display("FAIL BORDER: %0s case=%0d",message,received);$finish;end endtask
 always @(negedge clk) begin rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};out_ready=reset || rng[3:0]>4;end
 always @(posedge clk) begin
  cycles=cycles+1;if(cycles>100000)fail("timeout");
  if(reset)stalled=0;else begin
   if(stalled && {out_valid,corner,score,out_x,out_y,out_level}!==held)fail("output stall");
   held={out_valid,corner,score,out_x,out_y,out_level};stalled=out_valid&&!out_ready;
   if(out_valid&&out_ready)begin
    if(received>=count)fail("extra packet");
    if({corner,score}!==expected[received] || out_x!==meta[received][18:9] || out_y!==meta[received][8:0])fail("reference mismatch");
    if(score>254)fail("score range");received=received+1;
   end
  end
 end
 initial begin
  $readmemh("vectors/parameters.hex",params);count=params[1];
  $readmemh("vectors/border_windows.hex",windows,0,count-1);$readmemh("vectors/border_params.hex",meta,0,count-1);$readmemh("vectors/border_expected.hex",expected,0,count-1);
  repeat(3)@(negedge clk);reset=0;
  for(i=0;i<count;i=i+1)begin
   if(rng[7:5]==0)begin in_valid=0;@(negedge clk);end
   in_valid=1;window_data=windows[i];threshold=meta[i][26:19];in_x=meta[i][18:9];in_y=meta[i][8:0];
   @(posedge clk);while(!in_ready)@(posedge clk);@(negedge clk);
  end
  in_valid=0;wait(received==count);repeat(4)@(negedge clk);if(!empty)fail("drain");
  $display("ALL BORDER DETECTOR TESTS PASSED: %0d cases, invalid slots poisoned",count);$finish;
 end
endmodule
