//==============================================================================
//  orb_orientation.v
//
//  ORB orientation by intensity centroid (IC_Angle of OpenCV):
//    - circular patch radius 15 inside a 31x31 window (umax table)
//    - m10 = sum(u * I), m01 = sum(v * I)   (u = +right, v = +down)
//    - angle = atan2(m01, m10) quantised to 30 sectors of 12 deg
//      sector = floor(angle / 12), angle in [0, 360)
//
//  Streaming architecture
//    * pixels  : ring of 32 row-RAMs (slot = pix_y mod 32), 640 x 8 each
//    * keypoint: NMS stream (raster order) goes into a FIFO and waits until
//                pixel (x+15, y+15) has been written, then one 31-column
//                sweep reads the whole patch column by column
//                (1 address -> 32 pixels, one per row-RAM)
//    * datapath: 8-stage pipeline (mask -> weight -> adder trees -> accumulate)
//    * sector  : 4-step binary search with cross products (no divider/CORDIC)
//
//  Throughput: one keypoint per 32 clocks. A keypoint must be started before
//  the stream reaches row y+17 (its rows are overwritten after that), otherwise
//  it is dropped and counted in drop_cnt.
//
//  Assumptions (from the keypoint-selection spec):
//    image 640x480, kp_x in [15,624], kp_y in [15,464] (anything else is
//    silently ignored), keypoints arrive in raster order, sync active-high rst.
//
//  Pure Verilog: no function, no generate, no for-loop.
//  FSM (orb_ori_sector) is split into 3 blocks.
//==============================================================================

//------------------------------------------------------------------------------
// One ring slot: 640 x 8 RAM, 1 write port, 1 read port (1 cycle read latency)
//------------------------------------------------------------------------------
module orb_ori_rowram (
    input  wire       clk,
    input  wire       we,
    input  wire [9:0] waddr,
    input  wire [7:0] wdata,
    input  wire [9:0] raddr,
    output reg  [7:0] rdata
);
    (* ramstyle = "M4K" *) reg [7:0] mem [0:639];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule
