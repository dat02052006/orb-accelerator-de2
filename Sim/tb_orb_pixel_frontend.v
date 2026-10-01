`timescale 1ns/1ps
module tb_orb_pixel_frontend;
    reg clk = 0;
    always #10 clk = ~clk; // 50 MHz
    reg reset = 1;
    reg start_valid = 0;
    wire start_ready;
    reg [9:0] cfg_width = 0;
    reg [8:0] cfg_height = 0;
    reg [1:0] cfg_level = 0;
    wire busy, done;
    reg [7:0] pixel_data = 0;
    reg pixel_valid = 0, pixel_sof = 0, pixel_eol = 0;
    wire pixel_ready;
    wire [391:0] window_data;
    wire window_valid;
    reg window_ready = 1;
    wire [9:0] center_x;
    wire [8:0] center_y;
    wire [1:0] level_id;
    orb_pixel_frontend dut (
        .clk(clk), .reset(reset), .start_valid(start_valid),
        .start_ready(start_ready), .cfg_width(cfg_width),
        .cfg_height(cfg_height), .cfg_level(cfg_level), .busy(busy), .done(done),
        .pixel_data(pixel_data), .pixel_valid(pixel_valid),
        .pixel_ready(pixel_ready), .pixel_sof(pixel_sof), .pixel_eol(pixel_eol),
        .window_data(window_data), .window_valid(window_valid),
        .window_ready(window_ready), .center_x(center_x),
        .center_y(center_y), .level_id(level_id)
    );

    integer frame_w = 0, frame_h = 0, frame_level = 0, seed = 0;
    integer input_count = 0, output_count = 0, total_checked = 0;
    integer first_input_cycle = 0, previous_input_cycle = 0;
    integer mode = 0, last_hold = 0, cycles = 0;
    integer ex, ey, r, c, px, py;
    reg active = 0;
    reg output_stalled = 0;
    reg [413:0] held_output;
    reg input_stalled = 0;
    reg [9:0] held_input;
    reg [31:0] rng = 32'h57a109de;
    reg [7:0] expected_pixel;

    function [7:0] image_pixel;
        input integer fx, fy, fs;
        begin
            image_pixel = ((fx*17) ^ (fy*29) ^ ((fx*fy)*3) ^ (fs*71));
        end
    endfunction

    task fail;
        input [511:0] message;
        begin
            $display("FAIL: %0s time=%0t input=%0d output=%0d", message,
                     $time, input_count, output_count);
            $finish;
        end
    endtask

    // Independent consumer: random backpressure plus forced final-window stall.
    always @(negedge clk) begin
        rng = {rng[30:0], rng[31]^rng[21]^rng[1]^rng[0]};
        if (reset || !active)
            window_ready = 1;
        else if (mode == 2 && window_valid &&
                 center_x == frame_w-4 && center_y == frame_h-4 && last_hold < 17) begin
            window_ready = 0;
            last_hold = last_hold + 1;
        end else if (mode != 0)
            window_ready = (rng[3:0] >= 5);
        else
            window_ready = 1;
    end

    always @(posedge clk) begin
        cycles = cycles + 1;
        if (cycles > 3000000)
            fail("simulation timeout");
        if (reset) begin
            output_stalled = 0;
            input_stalled = 0;
        end else begin
            if (output_stalled &&
                {window_valid, window_data, center_x, center_y, level_id} !== held_output)
                fail("window changed while stalled");
            held_output = {window_valid, window_data, center_x, center_y, level_id};
            output_stalled = window_valid && !window_ready;
            if (input_stalled && {pixel_data,pixel_sof,pixel_eol} !== held_input)
                fail("test source changed a stalled pixel");
            if (input_stalled && !pixel_valid)
                fail("test source withdrew valid while stalled");
            held_input = {pixel_data,pixel_sof,pixel_eol};
            input_stalled = pixel_valid && !pixel_ready;
            if (start_valid && start_ready && busy)
                fail("accepted start while busy");
            if (active) begin
                if (pixel_valid && pixel_ready) begin
                    if (mode == 0 && input_count > 0 && cycles != previous_input_cycle+1)
                        fail("not one pixel per cycle without stalls");
                    if (input_count == 0) first_input_cycle = cycles;
                    previous_input_cycle = cycles;
                    px = input_count % frame_w;
                    py = input_count / frame_w;
                    if (pixel_sof !== (input_count == 0) || pixel_eol !== (px == frame_w-1))
                        fail("incorrect source SOF/EOL");
                    if (pixel_data !== image_pixel(px, py, seed))
                        fail("incorrect source data");
                    input_count = input_count + 1;
                end
                if (window_valid && window_ready) begin
                    ex = 3 + (output_count % (frame_w-6));
                    ey = 3 + (output_count / (frame_w-6));
                    if (output_count >= (frame_w-6)*(frame_h-6))
                        fail("too many windows");
                    if (center_x !== ex[9:0] || center_y !== ey[8:0] || level_id !== frame_level[1:0])
                        fail("incorrect window center or level");
                    if (input_count <= (ey+3)*frame_w+(ex+3))
                        fail("window emitted before its last pixel arrived");
                    for (r = 0; r < 7; r = r+1)
                        for (c = 0; c < 7; c = c+1) begin
                            expected_pixel = image_pixel(ex-3+c, ey-3+r, seed);
                            if (window_data[(r*7+c)*8 +: 8] !== expected_pixel) begin
                                $display("Mismatch center=(%0d,%0d) cell=(%0d,%0d) got=%h expected=%h",
                                    ex,ey,r,c,window_data[(r*7+c)*8 +: 8],expected_pixel);
                                fail("incorrect window pixel");
                            end
                        end
                    output_count = output_count + 1;
                    total_checked = total_checked + 1;
                end
                if (done && (busy || input_count != frame_w*frame_h ||
                    output_count != (frame_w-6)*(frame_h-6)))
                    fail("done before full frame and output drain");
            end
        end
    end

    task run_frame;
        input integer w, h, lev, image_seed, stall_mode, abort_after;
        integer n;
        begin
            @(negedge clk);
            frame_w = w; frame_h = h; frame_level = lev; seed = image_seed;
            input_count = 0; output_count = 0; last_hold = 0;
            mode = stall_mode; active = 1;
            cfg_width = w; cfg_height = h; cfg_level = lev;
            start_valid = 1;
            @(posedge clk);
            while (!start_ready) @(posedge clk);
            @(negedge clk);
            start_valid = 0;
            for (n = 0; n < w*h; n = n+1) begin
                if (stall_mode != 0 && rng[7:5] == 0) begin
                    pixel_valid = 0;
                    @(negedge clk);
                end
                pixel_data = image_pixel(n%w,n/w,image_seed);
                pixel_sof = (n == 0); pixel_eol = (n%w == w-1);
                pixel_valid = 1;
                @(posedge clk);
                while (!pixel_ready) @(posedge clk);
                @(negedge clk);
                if (abort_after > 0 && n+1 == abort_after) begin
                    pixel_valid = 0; reset = 1; active = 0;
                    repeat (3) @(negedge clk);
                    reset = 0;
                    $display("PASS reset abort after %0d pixels", abort_after);
                    n = w*h;
                end
            end
            pixel_valid = 0; pixel_sof = 0; pixel_eol = 0;
            if (abort_after == 0) begin
                wait (done);
                @(negedge clk);
                if (input_count != w*h || output_count != (w-6)*(h-6))
                    fail("incorrect final counts");
                if (mode == 2 && last_hold != 17)
                    fail("final-window stall was not exercised");
                if (mode == 0 && previous_input_cycle-first_input_cycle+1 != w*h)
                    fail("incorrect unstalled input cycle budget");
                active = 0;
                $display("PASS %0dx%0d level=%0d mode=%0d windows=%0d",w,h,lev,stall_mode,output_count);
            end
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        reset = 0;
        // An illegal configuration must not be accepted (no error recovery contract).
        cfg_width = 6; cfg_height = 9; cfg_level = 0; start_valid = 1;
        @(negedge clk);
        if (start_ready || busy) fail("illegal width accepted");
        start_valid = 0;
        run_frame(7,7,0,1,0,0);
        run_frame(13,11,1,2,2,0);
        run_frame(19,15,2,3,1,0);
        run_frame(23,17,0,4,1,180);
        run_frame(9,8,1,5,2,0);
        run_frame(640,480,0,6,0,0);
        run_frame(320,240,1,7,1,0);
        run_frame(160,120,2,8,2,0);
        // Full-resolution random stalls as well as the ideal-throughput case.
        run_frame(640,480,0,9,2,0);
        repeat (4) @(negedge clk);
        if (busy || window_valid || done) fail("pipeline did not return idle");
        $display("ALL TESTS PASSED: checked %0d complete windows (%0d pixels)",total_checked,total_checked*49);
        $finish;
    end
endmodule
