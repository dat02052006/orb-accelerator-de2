`timescale 1ns/1ps
module tb_hybrid;
 reg clk=0;always #10 clk=~clk;
 reg reset=1,in_valid=0,map_ready=1;wire in_ready,map_valid,map_corner,map_source,empty;
 reg [391:0] window_data=0;reg [9:0] in_x=0;reg [8:0] in_y=0;
 wire [7:0] map_score;wire [9:0] map_x;wire [8:0] map_y;wire [1:0] map_level;
 orb_hybrid_detector dut(.clk(clk),.reset(reset),.clear(1'b0),.width(10'd23),.height(9'd19),.threshold(8'd20),.in_valid(in_valid),.in_ready(in_ready),.window_data(window_data),.in_x(in_x),.in_y(in_y),.in_level(2'd0),.map_valid(map_valid),.map_ready(map_ready),.map_score(map_score),.map_corner(map_corner),.map_source(map_source),.map_x(map_x),.map_y(map_y),.map_level(map_level),.empty(empty));
 reg [7:0] pixels[0:436];reg [9:0] expected[0:436];integer holds[0:8];integer slot,i,r,c,x,y,received=0,cycles=0;
 reg stalled=0;reg [29:0] held;reg [31:0] rng=32'h194a91de;
 task fail;input [511:0] m;begin $display("FAIL HYBRID: %0s index=%0d",m,received);$finish;end endtask
 always @(negedge clk)begin
  rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};slot=-1;
  if(map_x==2&&map_y==0)slot=0;
  if(map_x==2&&map_y==3)slot=1;
  if(map_x==3&&map_y==3)slot=2;
  if(map_x==19&&map_y==3)slot=3;
  if(map_x==20&&map_y==3)slot=4;
  if(map_x==3&&map_y==15)slot=5;
  if(map_x==3&&map_y==16)slot=6;
  if(map_x==22&&map_y==18)slot=7;
  if(map_x==0&&map_y==0)slot=8;
  if(reset)map_ready=1;
  else if(map_valid&&slot>=0&&holds[slot]<13)begin map_ready=0;holds[slot]=holds[slot]+1;end
  else map_ready=rng[3:0]>4;
 end
 always @(posedge clk)begin
  cycles=cycles+1;if(cycles>100000)fail("timeout");
  if(reset)stalled=0;else begin
   if(stalled&&{map_valid,map_source,map_corner,map_score,map_x,map_y}!==held)fail("stall alignment");held={map_valid,map_source,map_corner,map_score,map_x,map_y};stalled=map_valid&&!map_ready;
   if(dut.count>8)fail("route overflow");
   if(dut.fast.window_valid&&(in_x<3||in_x>19||in_y<3||in_y>15))fail("FAST received border");
   if(dut.border.in_valid&&in_x>=3&&in_x<=19&&in_y>=3&&in_y<=15)fail("Border received interior");
   if(map_valid&&map_ready)begin
    x=received%23;y=received/23;
    if(received>=437||{map_source,map_corner,map_score}!==expected[received]||map_x!==x[9:0]||map_y!==y[8:0])fail("oracle");received=received+1;
   end
  end
 end
 initial begin
  for(i=0;i<9;i=i+1)holds[i]=0;
  $readmemh("vectors/f10_pixels.hex",pixels);$readmemh("vectors/f10_map.hex",expected);
  repeat(3)@(negedge clk);reset=0;
  for(i=0;i<437;i=i+1)begin
   if(rng[7:5]==0)begin in_valid=0;@(negedge clk);end
   in_x=i%23;in_y=i/23;
   for(r=0;r<7;r=r+1)for(c=0;c<7;c=c+1)begin x=in_x+c-3;y=in_y+r-3;window_data[(r*7+c)*8+:8]=(x>=0&&x<23&&y>=0&&y<19)?pixels[y*23+x]:8'ha7;end
   in_valid=1;@(posedge clk);while(!in_ready)@(posedge clk);@(negedge clk);
  end
  in_valid=0;wait(received==437);repeat(4)@(negedge clk);if(!empty)fail("drain");
  for(i=0;i<9;i=i+1)if(holds[i]!=13)fail("stall coverage");
  $display("ALL HYBRID TRANSITION TESTS PASSED: nine forced boundary/transition stalls");$finish;
 end
endmodule
