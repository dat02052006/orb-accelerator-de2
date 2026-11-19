//============================================================================
// image_loader.v
// Nhan anh 640x480 (8 bit/pixel) tu Nios II, kiem tra kich thuoc,
// ghi lan luot tung pixel vao SRAM thong qua sram_controller.
//
//   Addr(x,y) = y*640 + x,   0 .. 307199
//
// Pixel den theo thu tu raster (hang 0 truoc, x tang dan) nen dia chi
// chi la mot bo dem tang 1 sau moi lan ghi thanh cong, khong can nhan.
//
// Giao tiep voi Nios II (kieu Avalon-ST sink):
//   start        : xung 1 chu ky bat dau nap anh moi
//   img_width    : chieu rong anh khai bao, gia tri hop le = 640
//   img_height   : chieu cao anh khai bao,  gia tri hop le = 480
//   pixel_valid  : Nios II co pixel hop le
//   pixel_data   : pixel 8 bit
//   pixel_ready  : loader san sang nhan pixel (pixel nhan khi valid & ready)
//
// Giao tiep voi sram_controller:
//   sram_addr    : dia chi pixel 0..307199 (19 bit)
//   sram_wr_data : du lieu ghi 8 bit
//   sram_we      : giu muc 1 cho den khi sram_ready = 1
//   sram_ready   : xung bao giao dich ghi hoan tat
//============================================================================
module image_loader #(
    parameter IMG_WIDTH  = 640,
    parameter IMG_HEIGHT = 480,
    parameter ADDR_W     = 19                       // 2^19 = 524288 > 307200
)(
    input  wire              clk,
    input  wire              rst_n,

    // ---- phia Nios II ----
    input  wire              start,
    input  wire [9:0]        img_width,
    input  wire [9:0]        img_height,
    input  wire              pixel_valid,
    input  wire [7:0]        pixel_data,
    output reg               pixel_ready,

    // ---- phia sram_controller ----
    output wire [ADDR_W-1:0] sram_addr,
    output wire [7:0]        sram_wr_data,
    output reg               sram_we,
    input  wire              sram_ready,

    // ---- trang thai ----
    output reg               image_loaded,
    output reg               size_error
);

    //------------------------------------------------------------------
    // Hang so
    //------------------------------------------------------------------
    localparam [ADDR_W-1:0] LAST_ADDR = IMG_WIDTH*IMG_HEIGHT - 1;   // 307199

    localparam [2:0] S_IDLE  = 3'd0,
                     S_RECV  = 3'd1,    // cho pixel tu Nios II
                     S_WRITE = 3'd2,    // ghi pixel vao SRAM
                     S_DONE  = 3'd3,    // du 307200 byte
                     S_ERROR = 3'd4;    // sai kich thuoc

    reg  [2:0]        state, next_state;

    // Datapath
    reg  [ADDR_W-1:0] addr_cnt;
    reg  [7:0]        data_reg;

    // Tin hieu dieu khien tu khoi output logic
    reg               addr_clr;
    reg               addr_inc;
    reg               load_data;

    wire size_ok   = (img_width == IMG_WIDTH) && (img_height == IMG_HEIGHT);
    wire last_addr = (addr_cnt == LAST_ADDR);

    assign sram_addr    = addr_cnt;
    assign sram_wr_data = data_reg;

    //------------------------------------------------------------------
    // FSM phan 1: next-state transition logic
    //------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start) begin
                    if (size_ok) next_state = S_RECV;
                    else         next_state = S_ERROR;
                end
            end

            S_RECV: begin
                if (pixel_valid) next_state = S_WRITE;
            end

            S_WRITE: begin
                if (sram_ready) begin
                    if (last_addr) next_state = S_DONE;
                    else           next_state = S_RECV;
                end
            end

            S_DONE: begin                       // cho phep nap anh moi
                if (start) begin
                    if (size_ok) next_state = S_RECV;
                    else         next_state = S_ERROR;
                end
            end

            S_ERROR: begin
                if (start) begin
                    if (size_ok) next_state = S_RECV;
                    else         next_state = S_ERROR;
                end
            end

            default: next_state = S_IDLE;
        endcase
    end

    //------------------------------------------------------------------
    // FSM phan 2: state register update
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    //------------------------------------------------------------------
    // FSM phan 3: combinational output logic
    //------------------------------------------------------------------
    always @(*) begin
        pixel_ready  = 1'b0;
        sram_we      = 1'b0;
        image_loaded = 1'b0;
        size_error   = 1'b0;
        addr_clr     = 1'b0;
        addr_inc     = 1'b0;
        load_data    = 1'b0;

        case (state)
            S_IDLE: begin
                addr_clr = 1'b1;
            end

            S_RECV: begin
                pixel_ready = 1'b1;
                load_data   = pixel_valid;      // chot pixel khi valid & ready
            end

            S_WRITE: begin
                sram_we  = 1'b1;
                addr_inc = sram_ready & ~last_addr;   // tang dia chi sau khi ghi xong
            end

            S_DONE: begin
                image_loaded = 1'b1;
                addr_clr     = start;
            end

            S_ERROR: begin
                size_error = 1'b1;
                addr_clr   = start;
            end

            default: begin
                addr_clr = 1'b1;
            end
        endcase
    end

    //------------------------------------------------------------------
    // Datapath registers (dieu khien boi cac tin hieu o tren)
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr_cnt <= {ADDR_W{1'b0}};
            data_reg <= 8'd0;
        end
        else begin
            if (addr_clr)      addr_cnt <= {ADDR_W{1'b0}};
            else if (addr_inc) addr_cnt <= addr_cnt + 1'b1;

            if (load_data)     data_reg <= pixel_data;
        end
    end

endmodule
