//==============================================================================
// nms_top.v
// Kết nối toàn bộ NMS:
//   Candidate Buffer  ←→  Controller + Neighbor Search + Comparator
//                     →  Keypoint Buffer
//==============================================================================

module nms_top (
    input  wire         clk,
    input  wire         rst_n,

    // ---- Từ Candidate Buffer ----
    input  wire         bank_ready,
    input  wire [9:0]   bank_count_0,
    input  wire [9:0]   bank_count_1,
    input  wire [9:0]   bank_count_2,
    input  wire [9:0]   bank_row_0,
    input  wire [9:0]   bank_row_1,
    input  wire [9:0]   bank_row_2,
    input  wire [2:0]   bank_state_0,
    input  wire [2:0]   bank_state_1,
    input  wire [2:0]   bank_state_2,

    // 3 read ports
    output wire         rd_en_0,
    output wire [9:0]   rd_addr_0,
    input  wire [17:0]  rd_data_0,

    output wire         rd_en_1,
    output wire [9:0]   rd_addr_1,
    input  wire [17:0]  rd_data_1,

    output wire         rd_en_2,
    output wire [9:0]   rd_addr_2,
    input  wire [17:0]  rd_data_2,

    // Báo NMS xong 1 bank
    output wire         nms_done,
    output wire [1:0]   nms_done_bank,

    // ---- Tới Keypoint Buffer ----
    output wire         kp_valid,
    output wire [27:0]  kp_data,            // {score[7:0], y[9:0], x[9:0]}
    input  wire         kp_ready,

    // Status
    output wire         nms_busy
);

    //------------------------------------------------------------------
    // Internal signals
    //------------------------------------------------------------------
    wire        nb_start;
    wire        nb_busy;
    wire        nb_done;
    wire [9:0]  center_x;
    wire [9:0]  center_y;
    wire [1:0]  center_bank;
    wire [7:0]  center_score;       // latched từ controller

    wire [7:0]  nb_score_0, nb_score_1, nb_score_2, nb_score_3;
    wire [7:0]  nb_score_4, nb_score_5, nb_score_6, nb_score_7;
    wire [7:0]  nb_valid;

    wire        keep;

    wire        center_rd_en;
    wire [9:0]  center_rd_addr;
    wire [1:0]  center_rd_bank;
    reg  [17:0] center_rd_data;

    // Neighbor search read controls
    wire        nb_rd_en_0, nb_rd_en_1, nb_rd_en_2;
    wire [9:0]  nb_rd_addr_0, nb_rd_addr_1, nb_rd_addr_2;

    //------------------------------------------------------------------
    // Mux 3 read ports (controller và neighbor search không chồng chéo)
    //------------------------------------------------------------------
    assign rd_en_0   = nb_rd_en_0 | (center_rd_en && (center_rd_bank == 2'd0));
    assign rd_addr_0 = nb_rd_en_0 ? nb_rd_addr_0 :
                       (center_rd_en && (center_rd_bank == 2'd0)) ? center_rd_addr : 10'd0;

    assign rd_en_1   = nb_rd_en_1 | (center_rd_en && (center_rd_bank == 2'd1));
    assign rd_addr_1 = nb_rd_en_1 ? nb_rd_addr_1 :
                       (center_rd_en && (center_rd_bank == 2'd1)) ? center_rd_addr : 10'd0;

    assign rd_en_2   = nb_rd_en_2 | (center_rd_en && (center_rd_bank == 2'd2));
    assign rd_addr_2 = nb_rd_en_2 ? nb_rd_addr_2 :
                       (center_rd_en && (center_rd_bank == 2'd2)) ? center_rd_addr : 10'd0;

    always @(*) begin
        case (center_rd_bank)
            2'd0:    center_rd_data = rd_data_0;
            2'd1:    center_rd_data = rd_data_1;
            2'd2:    center_rd_data = rd_data_2;
            default: center_rd_data = 18'd0;
        endcase
    end

    //------------------------------------------------------------------
    // Neighbor Search
    //------------------------------------------------------------------
    nms_neighbor_search u_neighbor (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (nb_start),
        .busy           (nb_busy),
        .done           (nb_done),
        .center_x       (center_x),
        .center_y       (center_y),
        .center_bank    (center_bank),
        .bank_count_0   (bank_count_0),
        .bank_count_1   (bank_count_1),
        .bank_count_2   (bank_count_2),
        .bank_row_0     (bank_row_0),
        .bank_row_1     (bank_row_1),
        .bank_row_2     (bank_row_2),
        .bank_state_0   (bank_state_0),
        .bank_state_1   (bank_state_1),
        .bank_state_2   (bank_state_2),
        .rd_en_0        (nb_rd_en_0),
        .rd_addr_0      (nb_rd_addr_0),
        .rd_data_0      (rd_data_0),
        .rd_en_1        (nb_rd_en_1),
        .rd_addr_1      (nb_rd_addr_1),
        .rd_data_1      (rd_data_1),
        .rd_en_2        (nb_rd_en_2),
        .rd_addr_2      (nb_rd_addr_2),
        .rd_data_2      (rd_data_2),
        .nb_score_0     (nb_score_0),
        .nb_score_1     (nb_score_1),
        .nb_score_2     (nb_score_2),
        .nb_score_3     (nb_score_3),
        .nb_score_4     (nb_score_4),
        .nb_score_5     (nb_score_5),
        .nb_score_6     (nb_score_6),
        .nb_score_7     (nb_score_7),
        .nb_valid       (nb_valid)
    );

    //------------------------------------------------------------------
    // Comparator (dùng center_score đã latch)
    //------------------------------------------------------------------
    nms_comparator u_comp (
        .center_score   (center_score),
        .nb_score_0     (nb_score_0),
        .nb_score_1     (nb_score_1),
        .nb_score_2     (nb_score_2),
        .nb_score_3     (nb_score_3),
        .nb_score_4     (nb_score_4),
        .nb_score_5     (nb_score_5),
        .nb_score_6     (nb_score_6),
        .nb_score_7     (nb_score_7),
        .nb_valid       (nb_valid),
        .keep           (keep)
    );

    //------------------------------------------------------------------
    // Controller
    //------------------------------------------------------------------
    nms_controller u_ctrl (
        .clk            (clk),
        .rst_n          (rst_n),
        .bank_ready     (bank_ready),
        .bank_count_0   (bank_count_0),
        .bank_count_1   (bank_count_1),
        .bank_count_2   (bank_count_2),
        .bank_row_0     (bank_row_0),
        .bank_row_1     (bank_row_1),
        .bank_row_2     (bank_row_2),
        .bank_state_0   (bank_state_0),
        .bank_state_1   (bank_state_1),
        .bank_state_2   (bank_state_2),
        .nb_start       (nb_start),
        .nb_busy        (nb_busy),
        .nb_done        (nb_done),
        .center_x       (center_x),
        .center_y       (center_y),
        .center_bank    (center_bank),
        .center_score   (center_score),
        .keep           (keep),
        .center_rd_en   (center_rd_en),
        .center_rd_addr (center_rd_addr),
        .center_rd_bank (center_rd_bank),
        .center_rd_data (center_rd_data),
        .kp_valid       (kp_valid),
        .kp_data        (kp_data),
        .kp_ready       (kp_ready),
        .nms_done       (nms_done),
        .nms_done_bank  (nms_done_bank),
        .nms_busy       (nms_busy)
    );

endmodule
