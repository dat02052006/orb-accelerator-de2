`timescale 1ns/1ps
module tb_detector_v2;
 reg clk=0;always #10 clk=~clk;
 reg reset=1,start_valid=0;wire start_ready,busy,done;
 reg [9:0] cfg_width=0;reg [8:0] cfg_height=0;reg [1:0] cfg_level=0;reg [7:0] cfg_threshold=0;
 reg pixel_valid=0,pixel_sof=0,pixel_eol=0;reg [7:0] pixel_data=0;wire pixel_ready;
 wire keypoint_valid;reg keypoint_ready=1;wire [7:0] keypoint_score;wire [9:0] keypoint_x;wire [8:0] keypoint_y;wire [1:0] keypoint_level;
 orb_detector_frontend dut(.clk(clk),.reset(reset),.start_valid(start_valid),.start_ready(start_ready),.cfg_width(cfg_width),.cfg_height(cfg_height),.cfg_level(cfg_level),.cfg_threshold(cfg_threshold),.busy(busy),.done(done),.pixel_data(pixel_data),.pixel_valid(pixel_valid),.pixel_ready(pixel_ready),.pixel_sof(pixel_sof),.pixel_eol(pixel_eol),.keypoint_valid(keypoint_valid),.keypoint_ready(keypoint_ready),.keypoint_score(keypoint_score),.keypoint_x(keypoint_x),.keypoint_y(keypoint_y),.keypoint_level(keypoint_level));
 reg rvalid=0,rsof=0,reol=0;reg [7:0] rdata=0;wire rready,rsready,rbusy,rdone,rkvalid;
 wire [7:0] rkscore;wire [9:0] rkx;wire [8:0] rky;wire [1:0] rklevel;
 orb_detector_frontend_v1 ref_dut(.clk(clk),.reset(reset),.start_valid(start_valid),.start_ready(rsready),.cfg_width(cfg_width),.cfg_height(cfg_height),.cfg_level(cfg_level),.cfg_threshold(cfg_threshold),.busy(rbusy),.done(rdone),.pixel_data(rdata),.pixel_valid(rvalid),.pixel_ready(rready),.pixel_sof(rsof),.pixel_eol(reol),.keypoint_valid(rkvalid),.keypoint_ready(1'b1),.keypoint_score(rkscore),.keypoint_x(rkx),.keypoint_y(rky),.keypoint_level(rklevel));
 reg [31:0] params[0:123];reg [7:0] pixels[0:307199];reg [9:0] expected[0:307199];reg [26:0] keys[0:307199],rkeys[0:307199];reg [255:0] path;
 integer frame=0,w,h,lev,threshold,mode,nkeys,nrkeys;
 integer sent=0,map_seen=0,win_seen=0,key_seen=0,core_seen=0,ref_seen=0,ref_scores=0;
 integer cycles=0,last_cycle=0,first_cycle=0,start_cycle=0,end_cycle=0,hold_count=0;
 integer ex,ey,xx,yy,rr,cc,idx,f,base;
 reg active=0,done_seen=0,rdone_seen=0,stalled=0,map_stalled=0,input_stalled=0,previous_done=0;
 reg [29:0] held;reg [31:0] map_held;reg [10:0] input_held;reg [31:0] rng=32'h84901ab7;
 task fail;input [511:0] m;begin $display("FAIL V2 %0s frame=%0d sent=%0d win=%0d map=%0d key=%0d time=%0t",m,frame,sent,win_seen,map_seen,key_seen,$time);$finish;end endtask
 always @(negedge clk)begin
  rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
  if(reset||!active||mode==0)keypoint_ready=1;
  else if(keypoint_valid&&key_seen==nkeys-1&&hold_count<23)begin keypoint_ready=0;hold_count=hold_count+1;end
  else keypoint_ready=rng[3:0]>5;
 end
 always @(posedge clk)begin
  cycles=cycles+1;if(cycles>5000000)fail("timeout");
  if(reset)begin stalled=0;map_stalled=0;input_stalled=0;previous_done=0;end
  else begin
   if(previous_done&&done)fail("done pulse");previous_done=done;
   if(stalled&&{keypoint_valid,keypoint_x,keypoint_y,keypoint_score,keypoint_level}!==held)fail("keypoint stall");held={keypoint_valid,keypoint_x,keypoint_y,keypoint_score,keypoint_level};stalled=keypoint_valid&&!keypoint_ready;
   if(map_stalled&&{dut.map_valid,dut.map_source,dut.map_corner,dut.map_score,dut.map_x,dut.map_y,dut.map_level}!==map_held)fail("map stall");map_held={dut.map_valid,dut.map_source,dut.map_corner,dut.map_score,dut.map_x,dut.map_y,dut.map_level};map_stalled=dut.map_valid&&!dut.map_ready;
   if(input_stalled&&{pixel_valid,pixel_data,pixel_sof,pixel_eol}!==input_held)fail("source stall");input_held={pixel_valid,pixel_data,pixel_sof,pixel_eol};input_stalled=pixel_valid&&!pixel_ready;
   if(active)begin
    if(pixel_valid&&pixel_ready)begin if(mode==0&&sent>0&&cycles!=last_cycle+1)fail("not 1 pixel/clock");if(sent==0)first_cycle=cycles;last_cycle=cycles;sent=sent+1;end
    if(dut.window_valid&&dut.window_ready)begin
     ex=win_seen%w;ey=win_seen/w;if(win_seen>=w*h||dut.window_x!==ex[9:0]||dut.window_y!==ey[8:0])fail("window order");
     if(w<64||ex<3||ex>w-4||ey<3||ey>h-4)begin
      for(rr=0;rr<7;rr=rr+1)for(cc=0;cc<7;cc=cc+1)begin
       xx=ex+cc-3;yy=ey+rr-3;
       if(xx>=0&&xx<w&&yy>=0&&yy<h)begin
        if(sent<=yy*w+xx)fail("future tap");
        if(dut.window_data[(rr*7+cc)*8+:8]!==pixels[yy*w+xx])begin $display("window center %0d %0d cell %0d %0d",ex,ey,rr,cc);fail("real tap");end
       end
      end
     end
     win_seen=win_seen+1;
    end
    if(dut.map_valid&&dut.map_ready)begin
     ex=map_seen%w;ey=map_seen/w;if(map_seen>=w*h)fail("extra map");
     if({dut.map_source,dut.map_corner,dut.map_score}!==expected[map_seen]||dut.map_x!==ex[9:0]||dut.map_y!==ey[8:0]||dut.map_level!==lev[1:0])begin $display("map %0d %0d got %h expected %h",ex,ey,{dut.map_source,dut.map_corner,dut.map_score},expected[map_seen]);fail("map oracle");end
     if(dut.map_score>254)fail("score range");map_seen=map_seen+1;
    end
    if(keypoint_valid&&keypoint_ready)begin
     if(key_seen>=nkeys||{keypoint_x,keypoint_y,keypoint_score}!==keys[key_seen]||keypoint_level!==lev[1:0])fail("keypoint oracle");
     if(keypoint_x>=4&&keypoint_x<=w-5&&keypoint_y>=4&&keypoint_y<=h-5)begin if(core_seen>=nrkeys||{keypoint_x,keypoint_y,keypoint_score}!==rkeys[core_seen])fail("V1/V2 core differs");core_seen=core_seen+1;end
     key_seen=key_seen+1;
    end
    if(ref_dut.score_valid&&ref_dut.score_ready)begin idx=ref_dut.score_y*w+ref_dut.score_x;if({ref_dut.score_corner,ref_dut.score}!==expected[idx][8:0])fail("actual V1 score");ref_scores=ref_scores+1;end
    if(rkvalid)begin if(ref_seen>=nrkeys||{rkx,rky,rkscore}!==rkeys[ref_seen])fail("actual V1 keypoint");ref_seen=ref_seen+1;end
    if(rdone)rdone_seen=1;
    if(done)begin if(busy||sent!=w*h||map_seen!=w*h||win_seen!=w*h||key_seen!=nkeys)fail("early done");done_seen=1;end_cycle=cycles;end
    if(keypoint_valid&&!keypoint_ready&&(!busy||done))fail("done during stall");
   end
  end
 end
 task send_v2;input integer limit;integer j;begin
  for(j=0;j<limit;j=j+1)begin
   if(mode!=0&&rng[7:5]==0)begin pixel_valid=0;@(negedge clk);end
   pixel_valid=1;pixel_data=pixels[j];pixel_sof=(j==0);pixel_eol=(j%w==w-1);@(posedge clk);while(!pixel_ready)@(posedge clk);@(negedge clk);
  end pixel_valid=0;pixel_sof=0;pixel_eol=0;
 end endtask
 task send_v1;input integer limit;integer j;begin
  for(j=0;j<limit;j=j+1)begin rvalid=1;rdata=pixels[j];rsof=(j==0);reol=(j%w==w-1);@(posedge clk);while(!rready)@(posedge clk);@(negedge clk);end rvalid=0;rsof=0;reol=0;
 end endtask
 task run_frame;input integer index,abort_after;integer limit;begin
  @(negedge clk);frame=index;base=3+index*7;w=params[base];h=params[base+1];lev=params[base+2];threshold=params[base+3];mode=params[base+4];nkeys=params[base+5];nrkeys=params[base+6];
  $sformat(path,"vectors/f%0d_pixels.hex",index);$readmemh(path,pixels,0,w*h-1);$sformat(path,"vectors/f%0d_map.hex",index);$readmemh(path,expected,0,w*h-1);
  if(nkeys>0)begin $sformat(path,"vectors/f%0d_keys.hex",index);$readmemh(path,keys,0,nkeys-1);end
  if(nrkeys>0)begin $sformat(path,"vectors/f%0d_v1keys.hex",index);$readmemh(path,rkeys,0,nrkeys-1);end
  sent=0;win_seen=0;map_seen=0;key_seen=0;core_seen=0;ref_seen=0;ref_scores=0;hold_count=0;done_seen=0;rdone_seen=0;active=1;
  cfg_width=w;cfg_height=h;cfg_level=lev;cfg_threshold=threshold;start_valid=1;start_cycle=cycles;@(posedge clk);while(!start_ready||!rsready)@(posedge clk);@(negedge clk);start_valid=0;
  limit=abort_after>0?abort_after:w*h;fork send_v2(limit);send_v1(limit);join
  if(abort_after>0)begin reset=1;active=0;repeat(3)@(negedge clk);reset=0;$display("PASS reset mid-frame");end
  else begin
   wait(done_seen&&rdone_seen);@(negedge clk);
   if(core_seen!=nrkeys||ref_seen!=nrkeys||ref_scores!=(w-6)*(h-6))fail("V1 counts");if(mode!=0&&nkeys>0&&hold_count!=23)fail("last stall coverage");
   active=0;$display("PASS V2 frame=%0d %0dx%0d map=%0d keys=%0d V1keys=%0d cycles=%0d inputspan=%0d",index,w,h,map_seen,key_seen,ref_seen,end_cycle-start_cycle,last_cycle-first_cycle+1);
  end
 end endtask
 initial begin
  $readmemh("vectors/parameters.hex",params);repeat(3)@(negedge clk);reset=0;
  for(f=0;f<params[0];f=f+1)begin if(f==10)run_frame(f,350);run_frame(f,0);end
  repeat(4)@(negedge clk);if(busy||done||keypoint_valid)fail("idle");$display("ALL STREAMING TESTS PASSED");$display("ALL DETECTOR V2 TESTS PASSED: actual V1/V2 core comparison");$finish;
 end
endmodule
