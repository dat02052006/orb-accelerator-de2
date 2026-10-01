# Calibration trước RTL

Coefficient bổ sung: **1**. Chia 2 trong border margin xuất phát từ hai hiệu intensity được cộng trong gx/gy, không phải hệ số fit tùy ý.

L/bright/dark corner có turn tại offset (1,1): ở toàn bộ contrast 1..255 đã thử, border margin=C và cả hai score=C-1 khi không noise. Flat, horizontal, vertical và diagonal ±45° đều border margin=0. Checker và isolated point có đáp ứng khác FAST: không thể ép phân bố của mọi loại hình học trùng nhau bằng một coefficient. Noise ±2 được ghi lại theo range/mean trong JSON.

| Pattern | Contrast | Noise | Mean FAST score | Mean Border score |
|---|---:|---:|---:|---:|
| flat | 20 | 0 | 0.00 | 0.00 |
| flat | 20 | 2 | 0.00 | 0.00 |
| flat | 80 | 0 | 0.00 | 0.00 |
| flat | 80 | 2 | 0.00 | 0.00 |
| flat | 200 | 0 | 0.00 | 0.00 |
| flat | 200 | 2 | 0.00 | 0.00 |
| horizontal | 20 | 0 | 0.00 | 0.00 |
| horizontal | 20 | 2 | 0.00 | 0.00 |
| horizontal | 80 | 0 | 0.00 | 0.00 |
| horizontal | 80 | 2 | 0.00 | 0.06 |
| horizontal | 200 | 0 | 0.00 | 0.00 |
| horizontal | 200 | 2 | 0.00 | 0.06 |
| vertical | 20 | 0 | 0.00 | 0.00 |
| vertical | 20 | 2 | 0.00 | 0.06 |
| vertical | 80 | 0 | 0.00 | 0.00 |
| vertical | 80 | 2 | 0.00 | 0.00 |
| vertical | 200 | 0 | 0.00 | 0.00 |
| vertical | 200 | 2 | 0.00 | 0.00 |
| diagonal_plus | 20 | 0 | 0.00 | 0.00 |
| diagonal_plus | 20 | 2 | 0.06 | 0.50 |
| diagonal_plus | 80 | 0 | 0.00 | 0.00 |
| diagonal_plus | 80 | 2 | 0.06 | 0.50 |
| diagonal_plus | 200 | 0 | 0.00 | 0.00 |
| diagonal_plus | 200 | 2 | 0.12 | 0.44 |
| diagonal_minus | 20 | 0 | 0.00 | 0.00 |
| diagonal_minus | 20 | 2 | 0.06 | 0.00 |
| diagonal_minus | 80 | 0 | 0.00 | 0.00 |
| diagonal_minus | 80 | 2 | 0.00 | 0.00 |
| diagonal_minus | 200 | 0 | 0.00 | 0.00 |
| diagonal_minus | 200 | 2 | 0.06 | 0.06 |
| L_corner | 20 | 0 | 19.00 | 19.00 |
| L_corner | 20 | 2 | 16.75 | 16.81 |
| L_corner | 80 | 0 | 79.00 | 79.00 |
| L_corner | 80 | 2 | 76.25 | 76.56 |
| L_corner | 200 | 0 | 199.00 | 199.00 |
| L_corner | 200 | 2 | 196.69 | 196.31 |
| bright_corner | 20 | 0 | 19.00 | 19.00 |
| bright_corner | 20 | 2 | 17.00 | 16.62 |
| bright_corner | 80 | 0 | 79.00 | 79.00 |
| bright_corner | 80 | 2 | 76.81 | 76.88 |
| bright_corner | 200 | 0 | 199.00 | 199.00 |
| bright_corner | 200 | 2 | 197.00 | 197.06 |
| dark_corner | 20 | 0 | 19.00 | 19.00 |
| dark_corner | 20 | 2 | 16.50 | 16.00 |
| dark_corner | 80 | 0 | 79.00 | 79.00 |
| dark_corner | 80 | 2 | 76.56 | 76.56 |
| dark_corner | 200 | 0 | 199.00 | 199.00 |
| dark_corner | 200 | 2 | 196.38 | 196.69 |
| checker | 20 | 0 | 0.00 | 19.00 |
| checker | 20 | 2 | 0.00 | 16.94 |
| checker | 80 | 0 | 0.00 | 79.00 |
| checker | 80 | 2 | 0.00 | 76.56 |
| checker | 200 | 0 | 0.00 | 199.00 |
| checker | 200 | 2 | 0.00 | 197.12 |
| isolated_bright | 20 | 0 | 19.00 | 0.00 |
| isolated_bright | 20 | 2 | 17.88 | 0.00 |
| isolated_bright | 80 | 0 | 79.00 | 0.00 |
| isolated_bright | 80 | 2 | 78.31 | 0.06 |
| isolated_bright | 200 | 0 | 199.00 | 0.00 |
| isolated_bright | 200 | 2 | 197.81 | 0.06 |
| isolated_dark | 20 | 0 | 19.00 | 0.00 |
| isolated_dark | 20 | 2 | 16.50 | 0.50 |
| isolated_dark | 80 | 0 | 79.00 | 0.00 |
| isolated_dark | 80 | 2 | 77.00 | 0.06 |
| isolated_dark | 200 | 0 | 199.00 | 0.00 |
| isolated_dark | 200 | 2 | 196.75 | 0.50 |

Đây là paired calibration trên các patch chuyển từ interior đến góc ảnh. Neighborhood một phía có thể lệch vị trí tới hai pixel và bỏ isolated point; không tuyên bố tương đương FAST hay false-positive bằng 0 ở mọi góc/ảnh tự nhiên.

## Kiểm tra thêm bốn edge và bốn corner

Đã chạy toàn bộ 11 pattern, 10 contrast level và hai mức noise trên tám vị trí. Các L-corner được đặt turn ở pixel nhìn thấy trong neighborhood thật; clean L/bright/dark score=C-1 tại mọi vị trí. Kết quả từng distribution nằm trong placement_results.json. Không fit coefficient sau khi quan sát một case đơn lẻ.

36 ideal single-edge orientations (0..175 degrees, step 5), contrast 180, threshold 20: zero border candidates in this synthetic experiment. Not a guarantee on textured/noisy real images.
