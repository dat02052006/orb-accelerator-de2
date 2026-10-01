#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
IVERILOG_BIN="${IVERILOG_BIN:-iverilog}"
VVP_BIN="${VVP_BIN:-vvp}"
export IVERILOG_BIN VVP_BIN
mkdir -p build
python verify_sources.py | tee build/source_checks.log
python calibrate.py | tee build/calibration.log
python make_vectors.py
# Run the complete original suite in its copied snapshot, without touching V1.
(cd ../regression/v1/sim && bash run_iverilog.sh) > build/v1_regression.log
cat build/v1_regression.log
grep -q 'ALL FAST/SCORE TESTS PASSED' build/v1_regression.log
grep -q 'ALL DETECTOR TESTS PASSED' build/v1_regression.log
if grep -q 'FAIL' build/v1_regression.log; then exit 1; fi
echo 'ALL FAST REGRESSION TESTS PASSED'
for top in tb_border tb_hybrid tb_nms_v2 tb_detector_v2; do
 if [ "$top" = tb_detector_v2 ]; then
  "$IVERILOG_BIN" -g2001 -Wall -s "$top" -o "build/$top.vvp" ../rtl/*.v ../regression/v1/rtl/orb_pixel_frontend.v ../regression/v1/rtl/orb_window_buffer.v ../regression/v1/rtl/orb_nms3x3.v v1_compare_top.v "$top.v"
 else
  "$IVERILOG_BIN" -g2001 -Wall -s "$top" -o "build/$top.vvp" ../rtl/*.v "$top.v"
 fi
 "$VVP_BIN" "build/$top.vvp" > "build/$top.log"
 cat "build/$top.log"
 if ! grep -q 'ALL .* TESTS PASSED' "build/$top.log" || grep -q 'FAIL' "build/$top.log"; then exit 1; fi
done
python verify_sources.py
