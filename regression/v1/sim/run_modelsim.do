transcript file modelsim_transcript.log
if {![file exists work]} {vlib work}
vlog ../rtl/*.v tb_fast_score.v tb_nms3x3.v tb_orb_detector.v
vsim work.tb_fast_score
run -all
quit -sim
vsim work.tb_nms3x3
run -all
quit -sim
vsim work.tb_orb_detector
add wave sim:/tb_orb_detector/clk sim:/tb_orb_detector/reset
add wave sim:/tb_orb_detector/busy sim:/tb_orb_detector/done
add wave sim:/tb_orb_detector/pixel_valid sim:/tb_orb_detector/pixel_ready
add wave sim:/tb_orb_detector/dut/fast_valid sim:/tb_orb_detector/dut/fast_ready
add wave sim:/tb_orb_detector/dut/score_valid sim:/tb_orb_detector/dut/score_ready
add wave sim:/tb_orb_detector/dut/score sim:/tb_orb_detector/dut/score_corner
add wave sim:/tb_orb_detector/keypoint_valid sim:/tb_orb_detector/keypoint_ready
add wave sim:/tb_orb_detector/keypoint_x sim:/tb_orb_detector/keypoint_y
add wave sim:/tb_orb_detector/keypoint_score sim:/tb_orb_detector/keypoint_level
run -all
