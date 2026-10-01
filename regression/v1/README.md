# ORB — Pixel Stream → FAST-9 → FAST Score → NMS

## Cấu hình đã chốt

- DE2 thường; Verilog-2001; Quartus II 13.0sp1; clock mục tiêu 50 MHz.
- Pixel grayscale unsigned 8 bit, đọc trái sang phải rồi trên xuống dưới.
- Các tầng 640×480, 320×240, 160×120 xử lý lần lượt bằng cùng phần cứng.
- FAST-9, ngưỡng cấu hình 8 bit; đề xuất giá trị khởi đầu 20.
- Score là ngưỡng nguyên lớn nhất vẫn vượt FAST-9 với so sánh nghiêm ngặt.
- NMS 3×3 chỉ giữ score lớn hơn cả tám hàng xóm; điểm bằng score đứng cạnh nhau bị loại.
- Không padding. FAST bỏ ba pixel biên; NMS bỏ bốn pixel biên.
- Reset đồng bộ tích cực mức cao. Giao tiếp ready/valid, hỗ trợ backpressure.

Toàn bộ file trong `rtl/` không dùng vòng lặp, generate/genvar, biến integer hay
cú pháp riêng SystemVerilog. Các RAM, cây so sánh và thanh ghi được mô tả tường
minh. Vòng lặp chỉ được dùng trong testbench và phần mềm tạo vector tham chiếu.

## Top và các module

Top mới là **`orb_detector_frontend`**. Bốn module của Pixel Frontend v2 được
kèm trong gói, giữ nguyên chức năng. Không cần nối riêng ba khối mới nếu sử dụng
top này.

| File/module | Vai trò |
|---|---|
| `orb_detector_frontend.v` | Nối toàn chuỗi và điều khiển start/busy/done cho một tầng |
| `orb_pixel_frontend.v` | Tọa độ raster và điều khiển pixel stream |
| `orb_row_ram.v`, `orb_line_buffer.v` | Sáu RAM hàng pixel, đọc đồng bộ dữ liệu cũ |
| `orb_window_buffer.v` | Cửa sổ pixel 7×7 |
| `orb_fast9.v` | Lấy vòng tròn 16 pixel, kiểm tra 16 cung liên tiếp dài 9 |
| `orb_fast_score.v` | Pipeline ba tầng tính score chính xác |
| `orb_score_math.v` | Cây so sánh signed min9, max4 và max8 |
| `orb_nms_row_ram.v` | RAM hàng score 9 bit: `{is_corner, score}` |
| `orb_nms3x3.v` | Hai hàng score và cửa sổ NMS 3×3, xuất keypoint thưa |

DMA, tạo Image Pyramid, bộ điều khiển SDRAM, Keypoint Selection, Orientation và
rBRIEF chưa có trong gói. Đầu vào là pixel của tầng đã được chuẩn bị sẵn.

## Giao tiếp top

| Cổng | Ý nghĩa |
|---|---|
| `clk`, `reset` | Clock; reset đồng bộ mức cao |
| `start_valid`, `start_ready` | Chấp nhận bắt đầu tầng tại cạnh lên khi cả hai bằng 1 |
| `cfg_width[9:0]`, `cfg_height[8:0]` | Kích thước tầng (độ rộng mặc định) |
| `cfg_level[1:0]` | Chỉ số tầng 0, 1 hoặc 2 |
| `cfg_threshold[7:0]` | Ngưỡng FAST, được chốt khi bắt đầu tầng |
| `busy`, `done` | Đang xử lý; xung hoàn thành một chu kỳ |
| `pixel_data[7:0]`, `pixel_valid`, `pixel_ready` | Stream pixel đầu vào |
| `pixel_sof`, `pixel_eol` | Pixel đầu tầng; pixel cuối hàng |
| `keypoint_valid`, `keypoint_ready` | Stream các điểm sau NMS |
| `keypoint_x[9:0]`, `keypoint_y[8:0]` | Tọa độ trong tầng hiện tại, bắt đầu từ 0 |
| `keypoint_level[1:0]`, `keypoint_score[7:0]` | Tầng và score của keypoint |

Đặt cấu hình hợp lệ, giữ start_valid đến cạnh lên có start_ready. Sau đó gửi
đúng `width*height` pixel. Nguồn giữ valid/data/marker ổn định nếu valid=1 và
ready=0. Bên nhận keypoint áp dụng cùng quy tắc bắt tay.

Như frontend v2, tọa độ được đếm theo kích thước đã chốt. SOF/EOL thuộc hợp đồng
nguồn và được testbench kiểm tra; RTL không dùng chúng để khôi phục lỗi stream.
Không chèn pixel từ tầng khác hoặc bắt đầu tầng tiếp theo khi busy=1.

`done` của top mới khác `done` của Pixel Frontend: nó đợi FAST, mọi tầng score,
cột NMS và keypoint đang chờ được xử lý hết. Nếu keypoint cuối chưa được nhận,
busy vẫn bằng 1. Với ảnh không có keypoint, vẫn có done sau khi xử lý đủ dữ liệu.
Reset hủy tầng hiện tại; bắt đầu một tầng mới bằng giao tiếp start sau reset.

Với parameter mặc định, kích thước được chấp nhận là 7..640 × 7..480, level 0..2.
Ảnh rộng hoặc cao dưới 9 không có cửa sổ NMS đầy đủ và không sinh keypoint.
Các kích thước nhỏ dùng để kiểm tra; cấu hình project là ba tầng đã chốt.

## FAST-9 và thứ tự vòng tròn

Frontend xuất 49 byte theo hàng; byte 0 ở `[7:0]`, pixel tâm ở `[199:192]`.
Vòng tròn được đánh số từ pixel ngay trên tâm, đi theo chiều kim đồng hồ:

| Chỉ số | dx | dy | Chỉ số | dx | dy |
|---:|---:|---:|---:|---:|---:|
| 0 | 0 | -3 | 8 | 0 | 3 |
| 1 | 1 | -3 | 9 | -1 | 3 |
| 2 | 2 | -2 | 10 | -2 | 2 |
| 3 | 3 | -1 | 11 | -3 | 1 |
| 4 | 3 | 0 | 12 | -3 | 0 |
| 5 | 3 | 1 | 13 | -3 | -1 |
| 6 | 2 | 2 | 14 | -2 | -2 |
| 7 | 1 | 3 | 15 | -1 | -3 |

Với tâm C và vòng tròn P_i, tính signed 9 bit `d_i = P_i-C`, trong [-255,255].
Sáng khi `d_i > threshold`; tối khi `d_i < -threshold`. Phép so sánh bằng ngưỡng
không được tính là đạt. Mỗi cung có chín phần tử, được xét với chỉ số modulo 16;
cung đi qua 15→0 cũng hợp lệ. FAST xuất một packet cho mỗi tâm, kể cả non-corner.

## Score chính xác và pipeline

Cho mỗi điểm bắt đầu cung s:

```
bright_margin(s) = min(d_s, ..., d_(s+8))
dark_margin(s)   = min(-d_s, ..., -d_(s+8))
M = max của 16 bright_margin và 16 dark_margin
```

Một cung vượt ngưỡng T khi margin > T. Vì pixel/ngưỡng là số nguyên, score lớn
nhất là `M-1`. Chỉ corner ở ngưỡng cấu hình được xuất score này; non-corner có
score 0 và cờ corner=0. Dải score thực tế là 0..254. Threshold=255 không sinh
corner. Threshold=0 có thể có corner với score=0; cờ corner được giữ riêng.

Ví dụ tâm=100, chín pixel liên tiếp đều bằng 150: margin của cung là 50; điểm
vượt threshold=20 và có score=49. Nó không vượt threshold=50 vì so sánh là `>`.

Score được chia ba tầng thanh ghi:

1. Tính và giữ 32 minima của các cung.
2. Tính và giữ tám maxima, mỗi nhóm bốn minima.
3. Chọn maximum cuối, trừ 1 và xuất score/corner/tọa độ/level đã căn chỉnh.

Pipeline có thể nhận một packet mỗi chu kỳ khi không bị chặn. Khi đầu ra score
bị chặn, tất cả thanh ghi của pipeline này giữ nguyên, upstream thấy ready=0.
Tần số 50 MHz là mục tiêu; chưa được xác nhận bằng timing sau fitting.

## NMS trên bản đồ score đầy đủ

NMS phải nhận mọi vị trí FAST theo raster:
`x=3..W-4`, `y=3..H-4`. Không được lọc bỏ non-corner trước khi gửi sang NMS.
Tại cấu hình W=640, mỗi hàng có 634 score. Hai RAM lưu hai hàng trước, mỗi ô
9 bit gồm cờ corner và score. Dung lượng dữ liệu hai RAM: `2*634*9=11.412 bit`;
số M4K thực tế cần xem báo cáo Quartus.

Sau khi đã đọc score tại (x,y), cửa sổ 3×3 đầy đủ có tâm (x-1,y-1).
Giữ tâm khi cờ corner=1 và score lớn hơn cả tám score xung quanh. Score của
non-corner bằng 0. Không có quy tắc ưu tiên tọa độ khi bằng score: điểm bằng
score của bất kỳ hàng xóm nào đều bị loại, đúng cấu hình đã chốt.

Keypoint nằm trong `4 <= x <= W-5`, `4 <= y <= H-5`. Cửa sổ pixel đầu tiên
phục vụ FAST cần pixel (6,6); quyết định NMS đầu tiên ở tâm (4,4) cần dữ liệu
pixel đến (8,8), sau đó cộng độ trễ pipeline. Không cần đợi hết ảnh mới xử lý.

## Mô phỏng và kết quả đã kiểm tra

Giữ nguyên thư mục `sim/vectors/` khi giải nén. Dữ liệu tham chiếu đã được tạo
sẵn. Có thể tạo lại bằng `python make_vectors.py` trong `sim/` (cần NumPy).
Mô hình score tham chiếu dùng binary search trên phép thử FAST, không dùng cây
min/max giống RTL. Bản đồ NMS được tính bằng phần mềm trên đầy đủ score raster.

Trong ModelSim, chuyển đến thư mục sim rồi chạy:

```tcl
cd C:/orb_fast_nms_v1/sim
do run_modelsim.do
```

Script chạy lần lượt ba testbench. Dòng kết thúc phải là:

```
ALL FAST/SCORE TESTS PASSED
ALL NMS TESTS PASSED
ALL DETECTOR TESTS PASSED
```

Các dòng thực tế có thể kèm số lượng case/packet. Nếu thấy FAIL, bài test dừng
và cần sửa lỗi trước khi tiếp tục. Không chỉ dựa vào exit code vì `$finish` của
testbench cũng kết thúc mô phỏng khi phát hiện lỗi.

Với Icarus Verilog cài trên PATH: `bash run_iverilog.sh`. Log từng bài được lưu
trong `sim/build/`. Các log đã chạy được kèm riêng trong `sim/`.

Đã thực thi bằng Icarus Verilog 11.0, chế độ Verilog-2001:

- 1.551 case riêng FAST/score: cung vòng, 8/9 pixel, đúng bằng threshold,
  signed cực trị, threshold 0/254/255, hai cực tính và dữ liệu ngẫu nhiên.
- NMS: plateau bằng score, score 0, điểm ở biên, bản đồ ngẫu nhiên và backpressure.
- Toàn chuỗi: ảnh 7×7, 9×9, ảnh ngẫu nhiên nhỏ và cả ba kích thước pyramid thực;
  reset giữa tầng, đổi tầng không xóa RAM, valid gián đoạn, stall keypoint cuối,
  done và tốc độ một pixel/chu kỳ khi không bị chặn.
- Mọi FAST flag và score trước NMS, cũng như danh sách keypoint sau NMS, đều
  được đối chiếu với tham chiếu. Các ảnh lớn là ảnh kiểm thử tổng hợp có các
  điểm sáng/tối cô lập; chưa phải đánh giá chất lượng trên ảnh tự nhiên.

Chưa chạy ModelSim-ASE hoặc tổng hợp/fitting Quartus trong môi trường thực hiện.
Chưa xác nhận sử dụng M4K, LUT/LE, Fmax, throughput SDRAM hay 60 FPS toàn ORB.

## Đưa vào Quartus và bước tiếp theo

Thêm toàn bộ file `rtl/*.v`, chọn top `orb_detector_frontend`. Không thêm
testbench vào nguồn tổng hợp. `constraints/frontend.sdc` ràng buộc cổng clk
20 ns để kiểm tra riêng khối; ràng buộc I/O/pin và clock toàn SoC cần bổ sung
theo kết nối thực tế, không tạo clock trùng khi tích hợp.

Sau compilation, kiểm tra RAM inference/OLD DATA read-during-write, tài nguyên
M4K/LE và timing sau fitting. Đầu ra hiện tại chuyển sang Keypoint Selection;
Orientation/rBRIEF sẽ đọc lại patch từ các tầng ảnh lưu trong SDRAM.
