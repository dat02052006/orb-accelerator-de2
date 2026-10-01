#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
IVERILOG_BIN="${IVERILOG_BIN:-iverilog}"
VVP_BIN="${VVP_BIN:-vvp}"
mkdir -p build
for top in tb_fast_score tb_nms3x3 tb_orb_detector; do
    "$IVERILOG_BIN" -g2001 -Wall -s "$top" -o "build/$top.vvp" ../rtl/*.v "$top.v"
    "$VVP_BIN" "build/$top.vvp" > "build/$top.log"
    cat "build/$top.log"
    if ! grep -q 'ALL .* TESTS PASSED' "build/$top.log" || grep -q 'FAIL' "build/$top.log"; then
        exit 1
    fi
done
