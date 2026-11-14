//==============================================================================
// nms_comparator.v
// So sánh score của candidate trung tâm với tối đa 8 neighbor.
// Điều kiện giữ: Sc > Sn  ∀n
// Khi bằng nhau → ưu tiên candidate xuất hiện sớm hơn (raster-scan:
// Y nhỏ hơn, rồi X nhỏ hơn). Vì center đang được xét theo thứ tự tăng dần,
// nếu Sn == Sc thì neighbor xuất hiện sau → loại center.
//==============================================================================

module nms_comparator (
    input  wire [7:0]   center_score,
    input  wire [7:0]   nb_score_0,
    input  wire [7:0]   nb_score_1,
    input  wire [7:0]   nb_score_2,
    input  wire [7:0]   nb_score_3,
    input  wire [7:0]   nb_score_4,
    input  wire [7:0]   nb_score_5,
    input  wire [7:0]   nb_score_6,
    input  wire [7:0]   nb_score_7,
    input  wire [7:0]   nb_valid,       // bit i = 1 nếu neighbor i tồn tại

    output wire         keep            // 1 = giữ keypoint
);

    // Giữ khi center_score > tất cả neighbor hợp lệ
    // (khi bằng → không giữ, vì neighbor xuất hiện sau hoặc cùng lúc)
    wire gt0 = !nb_valid[0] || (center_score > nb_score_0);
    wire gt1 = !nb_valid[1] || (center_score > nb_score_1);
    wire gt2 = !nb_valid[2] || (center_score > nb_score_2);
    wire gt3 = !nb_valid[3] || (center_score > nb_score_3);
    wire gt4 = !nb_valid[4] || (center_score > nb_score_4);
    wire gt5 = !nb_valid[5] || (center_score > nb_score_5);
    wire gt6 = !nb_valid[6] || (center_score > nb_score_6);
    wire gt7 = !nb_valid[7] || (center_score > nb_score_7);

    assign keep = gt0 & gt1 & gt2 & gt3 & gt4 & gt5 & gt6 & gt7;

endmodule
