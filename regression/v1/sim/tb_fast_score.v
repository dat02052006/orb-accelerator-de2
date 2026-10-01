`timescale 1ns/1ps
module tb_fast_score;
    reg clk=0;
    always #10 clk=~clk;
    reg reset=1, clear=0;
    reg [7:0] threshold=0;
    reg window_valid=0;
    wire window_ready;
    reg [391:0] window_data=0;
    reg [9:0] window_x=0;
    reg [8:0] window_y=0;
    reg [1:0] window_level=0;
    wire fast_valid,fast_ready,corner;
    wire [127:0] ring_data;
    wire [7:0] center_data;
    wire [9:0] fast_x;
    wire [8:0] fast_y;
    wire [1:0] fast_level;
    wire score_valid,score_corner,empty;
    reg score_ready=1;
    wire [7:0] score;
    wire [9:0] score_x;
    wire [8:0] score_y;
    wire [1:0] score_level;
    orb_fast9 fast (
        .clk(clk),.reset(reset),.clear(clear),.threshold(threshold),
        .window_valid(window_valid),.window_ready(window_ready),.window_data(window_data),
        .window_x(window_x),.window_y(window_y),.window_level(window_level),
        .fast_valid(fast_valid),.fast_ready(fast_ready),.ring_data(ring_data),
        .center_data(center_data),.is_corner(corner),.fast_x(fast_x),.fast_y(fast_y),.fast_level(fast_level)
    );
    orb_fast_score scorer (
        .clk(clk),.reset(reset),.clear(clear),.fast_valid(fast_valid),.fast_ready(fast_ready),
        .ring_data(ring_data),.center_data(center_data),.is_corner(corner),
        .fast_x(fast_x),.fast_y(fast_y),.fast_level(fast_level),
        .score_valid(score_valid),.score_ready(score_ready),.score(score),.score_corner(score_corner),
        .score_x(score_x),.score_y(score_y),.score_level(score_level),.empty(empty)
    );
    reg [31:0] params[0:70];
    reg [391:0] windows[0:4095];
    reg [7:0] thresholds[0:4095];
    reg [8:0] expected[0:4095];
    integer count=0,received=0,detected=0,sent=0,cycles=0,last_input_cycle=0;
    integer i,ex,ey,lev,final_hold=0;
    reg [31:0] rng=32'h9123a4bd;
    reg stalled=0;
    reg [30:0] held;
    task fail;
        input [511:0] message;
        begin $display("FAIL FAST/SCORE: %0s case=%0d",message,received); $finish; end
    endtask
    always @(negedge clk) begin
        rng={rng[30:0],rng[31]^rng[21]^rng[1]^rng[0]};
        if(reset || sent<100) score_ready=1;
        else if(score_valid && score_x==(count-1)%600 && score_y==(count-1)/600+3 && final_hold<19) begin
            score_ready=0; final_hold=final_hold+1;
        end else score_ready=(rng[3:0]>=5);
    end
    always @(posedge clk) begin
        cycles=cycles+1;
        if(cycles>100000) fail("timeout");
        if(reset) stalled=0;
        else begin
            if(stalled && {score_valid,score,score_corner,score_x,score_y,score_level} !== held)
                fail("output changed during stall");
            held={score_valid,score,score_corner,score_x,score_y,score_level};
            stalled=score_valid && !score_ready;
            if(window_valid && window_ready) begin
                if(sent>0 && sent<100 && cycles!=last_input_cycle+1) fail("unstalled throughput");
                last_input_cycle=cycles; sent=sent+1;
            end
            if(fast_valid && fast_ready) begin
                if(corner !== expected[detected][8]) fail("FAST predicate");
                detected=detected+1;
            end
            if(score_valid && score_ready) begin
                if(received>=count) fail("extra score");
                ex=received%600; ey=received/600+3; lev=received%3;
                if({score_corner,score} !== expected[received]) begin
                    $display("got=%h expected=%h",{score_corner,score},expected[received]);
                    fail("score differs from binary-search oracle");
                end
                if(score_x!==ex[9:0] || score_y!==ey[8:0] || score_level!==lev[1:0])
                    fail("metadata alignment");
                received=received+1;
            end
        end
    end
    initial begin
        $readmemh("vectors/parameters.hex",params);
        count=params[0];
        $readmemh("vectors/unit_windows.hex",windows,0,count-1);
        $readmemh("vectors/unit_thresholds.hex",thresholds,0,count-1);
        $readmemh("vectors/unit_expected.hex",expected,0,count-1);
        repeat(3) @(negedge clk); reset=0;
        for(i=0;i<count;i=i+1) begin
            if(i>=100 && rng[6:4]==0) begin window_valid=0; @(negedge clk); end
            window_valid=1; window_data=windows[i]; threshold=thresholds[i];
            window_x=i%600; window_y=i/600+3; window_level=i%3;
            @(posedge clk); while(!window_ready) @(posedge clk);
            @(negedge clk);
        end
        window_valid=0;
        wait(received==count); repeat(4) @(negedge clk);
        if(!empty || fast_valid || detected!=count || sent!=count || final_hold!=19)
            fail("drain or stall coverage");
        $display("ALL FAST/SCORE TESTS PASSED: %0d cases",count);
        $finish;
    end
endmodule
