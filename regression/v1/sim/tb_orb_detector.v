`timescale 1ns/1ps
module tb_orb_detector;
    reg clk=0;
    always #10 clk=~clk;
    reg reset=1,start_valid=0;
    wire start_ready;
    reg [9:0] cfg_width=0;
    reg [8:0] cfg_height=0;
    reg [1:0] cfg_level=0;
    reg [7:0] cfg_threshold=0;
    wire busy,done;
    reg [7:0] pixel_data=0;
    reg pixel_valid=0,pixel_sof=0,pixel_eol=0;
    wire pixel_ready;
    wire keypoint_valid;
    reg keypoint_ready=1;
    wire [7:0] keypoint_score;
    wire [9:0] keypoint_x;
    wire [8:0] keypoint_y;
    wire [1:0] keypoint_level;
    orb_detector_frontend dut (
        .clk(clk),.reset(reset),.start_valid(start_valid),.start_ready(start_ready),
        .cfg_width(cfg_width),.cfg_height(cfg_height),.cfg_level(cfg_level),.cfg_threshold(cfg_threshold),
        .busy(busy),.done(done),.pixel_data(pixel_data),.pixel_valid(pixel_valid),
        .pixel_ready(pixel_ready),.pixel_sof(pixel_sof),.pixel_eol(pixel_eol),
        .keypoint_valid(keypoint_valid),.keypoint_ready(keypoint_ready),
        .keypoint_score(keypoint_score),.keypoint_x(keypoint_x),.keypoint_y(keypoint_y),
        .keypoint_level(keypoint_level)
    );
    reg [31:0] params[0:70];
    reg [7:0] pixels[0:307199];
    reg [8:0] scores[0:300515];
    reg [26:0] keys[0:300515];
    integer frame=0,w,h,lev,threshold,mode,nscore,nkey;
    integer sent=0,fast_seen=0,score_seen=0,key_seen=0,total_scores=0,total_keys=0;
    integer base,n,ex,ey,cycles=0,last_input_cycle=0,final_hold=0,first_input_cycle=0;
    reg [255:0] path;
    reg active=0,output_stalled=0,input_stalled=0,previous_done=0;
    reg [29:0] held_output;
    reg [10:0] held_input;
    reg [31:0] rng=32'ha012c572;
    task fail;
        input [511:0] message;
        begin
            $display("FAIL DETECTOR: %0s frame=%0d input=%0d FAST=%0d score=%0d key=%0d time=%0t",
                     message,frame,sent,fast_seen,score_seen,key_seen,$time);
            $finish;
        end
    endtask
    always @(negedge clk) begin
        rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
        if(reset || !active || mode==0) keypoint_ready=1;
        else if(mode==2 && keypoint_valid && key_seen==nkey-1 && final_hold<23) begin
            keypoint_ready=0; final_hold=final_hold+1;
        end else keypoint_ready=(rng[3:0]>=6);
    end
    always @(posedge clk) begin
        cycles=cycles+1;
        if(cycles>5000000) fail("timeout");
        if(reset) begin
            output_stalled=0; input_stalled=0; previous_done=0;
        end else begin
            if(previous_done && done) fail("done pulse longer than one cycle");
            previous_done=done;
            if(output_stalled && {keypoint_valid,keypoint_x,keypoint_y,keypoint_score,keypoint_level}!==held_output)
                fail("keypoint changed during stall");
            held_output={keypoint_valid,keypoint_x,keypoint_y,keypoint_score,keypoint_level};
            output_stalled=keypoint_valid && !keypoint_ready;
            if(input_stalled && {pixel_valid,pixel_data,pixel_sof,pixel_eol}!==held_input)
                fail("source violated ready/valid");
            held_input={pixel_valid,pixel_data,pixel_sof,pixel_eol};
            input_stalled=pixel_valid && !pixel_ready;
            if(start_valid && start_ready && busy) fail("start accepted while busy");
            if(active) begin
                if(pixel_valid && pixel_ready) begin
                    if(mode==0 && sent>0 && cycles!=last_input_cycle+1)
                        fail("unstalled pixel throughput");
                    if(sent==0) first_input_cycle=cycles;
                    last_input_cycle=cycles;
                    if(pixel_sof !== (sent==0) || pixel_eol !== (sent%w==w-1) || pixel_data!==pixels[sent])
                        fail("source pixel/markers");
                    sent=sent+1;
                end
                if(dut.fast_valid && dut.fast_ready) begin
                    if(fast_seen>=nscore) fail("extra FAST packet");
                    ex=3+fast_seen%(w-6); ey=3+fast_seen/(w-6);
                    if(dut.fast_corner!==scores[fast_seen][8] || dut.fast_x!==ex[9:0] ||
                       dut.fast_y!==ey[8:0] || dut.fast_level!==lev[1:0])
                        fail("FAST packet/oracle");
                    fast_seen=fast_seen+1;
                end
                if(dut.score_valid && dut.score_ready) begin
                    if(score_seen>=nscore) fail("extra score packet");
                    ex=3+score_seen%(w-6); ey=3+score_seen/(w-6);
                    if({dut.score_corner,dut.score}!==scores[score_seen] || dut.score_x!==ex[9:0] ||
                       dut.score_y!==ey[8:0] || dut.score_level!==lev[1:0]) begin
                        $display("got score=%h expected=%h at (%0d,%0d)",
                            {dut.score_corner,dut.score},scores[score_seen],ex,ey);
                        fail("dense score packet/oracle");
                    end
                    score_seen=score_seen+1; total_scores=total_scores+1;
                end
                if(keypoint_valid && keypoint_ready) begin
                    if(key_seen>=nkey) fail("unexpected keypoint");
                    if({keypoint_x,keypoint_y,keypoint_score}!==keys[key_seen] || keypoint_level!==lev[1:0]) begin
                        $display("got key=%h expected=%h",{keypoint_x,keypoint_y,keypoint_score},keys[key_seen]);
                        fail("NMS keypoint/oracle");
                    end
                    if(keypoint_x<4 || keypoint_x>w-5 || keypoint_y<4 || keypoint_y>h-5)
                        fail("NMS border");
                    key_seen=key_seen+1; total_keys=total_keys+1;
                end
                if(done && (busy || sent!=w*h || fast_seen!=nscore || score_seen!=nscore || key_seen!=nkey))
                    fail("done before complete drain");
                if(keypoint_valid && !keypoint_ready && (!busy || done))
                    fail("completion while keypoint is blocked");
            end
        end
    end
    task run_frame;
        input integer index,abort_after;
        begin
            @(negedge clk);
            frame=index; base=3+index*7;
            w=params[base]; h=params[base+1]; lev=params[base+2]; threshold=params[base+3];
            mode=params[base+4]; nscore=params[base+5]; nkey=params[base+6];
            $sformat(path,"vectors/frame_%0d_pixels.hex",index);
            $readmemh(path,pixels,0,w*h-1);
            $sformat(path,"vectors/frame_%0d_scores.hex",index);
            $readmemh(path,scores,0,nscore-1);
            if(nkey>0) begin
                $sformat(path,"vectors/frame_%0d_keypoints.hex",index);
                $readmemh(path,keys,0,nkey-1);
            end
            sent=0; fast_seen=0; score_seen=0; key_seen=0; final_hold=0; active=1;
            cfg_width=w; cfg_height=h; cfg_level=lev; cfg_threshold=threshold; start_valid=1;
            @(posedge clk); while(!start_ready) @(posedge clk);
            @(negedge clk); start_valid=0;
            for(n=0;n<w*h;n=n+1) begin
                if(mode!=0 && rng[7:5]==0) begin pixel_valid=0; @(negedge clk); end
                pixel_valid=1; pixel_data=pixels[n]; pixel_sof=(n==0); pixel_eol=(n%w==w-1);
                @(posedge clk); while(!pixel_ready) @(posedge clk);
                @(negedge clk);
                if(abort_after>0 && n+1==abort_after) begin
                    pixel_valid=0; reset=1; active=0;
                    repeat(3) @(negedge clk);
                    reset=0; n=w*h;
                    $display("PASS reset during detector pipeline after %0d pixels",abort_after);
                end
            end
            pixel_valid=0; pixel_sof=0; pixel_eol=0;
            if(abort_after==0) begin
                wait(done); @(negedge clk);
                if(busy || sent!=w*h || fast_seen!=nscore || score_seen!=nscore || key_seen!=nkey)
                    fail("final packet counts");
                if(mode==2 && nkey>0 && final_hold!=23) fail("final keypoint stall coverage");
                if(mode==0 && last_input_cycle-first_input_cycle+1!=w*h) fail("input cycle budget");
                active=0;
                $display("PASS frame=%0d %0dx%0d level=%0d threshold=%0d dense=%0d keys=%0d mode=%0d",
                    index,w,h,lev,threshold,nscore,nkey,mode);
            end
        end
    endtask
    integer f;
    initial begin
        $readmemh("vectors/parameters.hex",params);
        repeat(3) @(negedge clk); reset=0;
        for(f=0;f<params[1];f=f+1) begin
            if(f==2) run_frame(2,350);
            run_frame(f,0);
        end
        repeat(5) @(negedge clk);
        if(busy || done || keypoint_valid) fail("not idle after final frame");
        $display("ALL DETECTOR TESTS PASSED: dense=%0d keypoints=%0d",total_scores,total_keys);
        $finish;
    end
endmodule
