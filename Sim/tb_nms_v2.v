`timescale 1ns/1ps
module tb_nms_v2;
    reg clk=0;
    always #10 clk=~clk;
    reg reset=1,clear=0;
    reg [9:0] cfg_width=19;
    reg [8:0] cfg_height=15;
    wire done;
    reg score_valid=0,score_corner=0;
    wire score_ready;
    reg [7:0] score=0;
    reg [9:0] score_x=0;
    reg [8:0] score_y=0;
    reg [1:0] score_level=0;
    wire keypoint_valid,empty;
    reg keypoint_ready=1;
    wire [7:0] keypoint_score;
    wire [9:0] keypoint_x;
    wire [8:0] keypoint_y;
    wire [1:0] keypoint_level;
    orb_nms3x3_v2 dut (
        .clk(clk),.reset(reset),.clear(clear),.cfg_width(cfg_width),.cfg_height(cfg_height),.cfg_level(map[1:0]),.done(done),
        .score_valid(score_valid),.score_ready(score_ready),.score(score),.score_corner(score_corner),
        .score_x(score_x),.score_y(score_y),.score_level(score_level),
        .keypoint_valid(keypoint_valid),.keypoint_ready(keypoint_ready),
        .keypoint_score(keypoint_score),.keypoint_x(keypoint_x),.keypoint_y(keypoint_y),
        .keypoint_level(keypoint_level),.empty(empty)
    );
    reg [31:0] params[0:123];
    reg [8:0] scores[0:1023];
    reg [26:0] keys[0:1023];
    integer map=0,w,h,nscore,nkey,received=0,n=0,base,cycles=0,hold_count=0;
    reg [255:0] path;
    reg active=0,stalled=0;
    reg [29:0] held;
    reg [31:0] rng=32'h581abb21;
    task fail;
        input [511:0] message;
        begin $display("FAIL NMS: %0s map=%0d received=%0d",message,map,received); $finish; end
    endtask
    always @(negedge clk) begin
        rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
        if(reset || !active) keypoint_ready=1;
        else if(keypoint_valid && received==nkey-1 && hold_count<17) begin
            keypoint_ready=0; hold_count=hold_count+1;
        end else keypoint_ready=(rng[3:0]>=6);
    end
    always @(posedge clk) begin
        cycles=cycles+1;
        if(cycles>100000) fail("timeout");
        if(reset || clear) stalled=0;
        else begin
            if(stalled && {keypoint_valid,keypoint_x,keypoint_y,keypoint_score,keypoint_level}!==held)
                fail("output changed while blocked");
            held={keypoint_valid,keypoint_x,keypoint_y,keypoint_score,keypoint_level};
            stalled=keypoint_valid && !keypoint_ready;
            if(active && keypoint_valid && keypoint_ready) begin
                if(received>=nkey) fail("unexpected keypoint (tie/border/zero)");
                if({keypoint_x,keypoint_y,keypoint_score}!==keys[received] || keypoint_level!==map[1:0])
                    fail("wrong keypoint");
                received=received+1;
            end
        end
    end
    initial begin
        $readmemh("vectors/parameters.hex",params);
        repeat(3) @(negedge clk); reset=0;
        for(map=0;map<params[2];map=map+1) begin
            base=3+params[0]*7+map*3;
            w=params[base]; h=params[base+1]; nscore=w*h; nkey=params[base+2];
            $sformat(path,"vectors/n%0d_map.hex",map);
            $readmemh(path,scores,0,nscore-1);
            if(nkey>0) begin
                $sformat(path,"vectors/n%0d_keys.hex",map);
                $readmemh(path,keys,0,nkey-1);
            end
            clear=1; cfg_width=w; cfg_height=h; received=0; hold_count=0;
            @(negedge clk); clear=0; active=1;
            for(n=0;n<nscore;n=n+1) begin
                if(rng[7:5]==0) begin score_valid=0; @(negedge clk); end
                score_valid=1; score_corner=scores[n][8]; score=scores[n][7:0];
                score_x=n%w; score_y=n/w; score_level=map;
                @(posedge clk); while(!score_ready) @(posedge clk);
                @(negedge clk);
            end
            score_valid=0;
            wait(empty); repeat(2) @(negedge clk);
            if(received!=nkey || (nkey>0 && hold_count!=17)) fail("drain or final stall");
            active=0;
            $display("PASS NMS map=%0d scores=%0d keypoints=%0d",map,nscore,nkey);
        end
        $display("ALL NMS V2 TESTS PASSED"); $finish;
    end
endmodule
