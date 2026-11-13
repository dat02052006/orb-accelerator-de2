transcript file modelsim_v2.log
if {![file exists v1_work]} {vlib v1_work}
if {![file exists v2_work]} {vlib v2_work}
vmap v1_work [file normalize v1_work]
vmap v2_work [file normalize v2_work]
vlog -work v1_work ../regression/v1/rtl/*.v ../regression/v1/sim/tb_fast_score.v ../regression/v1/sim/tb_nms3x3.v ../regression/v1/sim/tb_orb_detector.v
vlog -work v2_work ../rtl/*.v ../regression/v1/rtl/orb_pixel_frontend.v ../regression/v1/rtl/orb_window_buffer.v ../regression/v1/rtl/orb_nms3x3.v v1_compare_top.v tb_border.v tb_hybrid.v tb_nms_v2.v tb_detector_v2.v
proc checked {lib top checks} {
    vsim -onfinish stop ${lib}.${top}
    run -all
    foreach pair $checks {
        set signal [lindex $pair 0]
        set wanted [lindex $pair 1]
        set actual [string trim [examine -radix decimal sim:/${top}/${signal}]]
        if {$actual != $wanted} {error "FAILED ${top}: ${signal}=${actual}, expected ${wanted}"}
    }
    quit -sim
}
# Original V1 fixtures are read relative to their snapshot directory.
set saved_dir [pwd]
cd ../regression/v1/sim
checked v1_work tb_fast_score {{received 1551} {detected 1551} {sent 1551} {empty 1} {fast_valid 0} {final_hold 19}}
checked v1_work tb_nms3x3 {{map 3}}
checked v1_work tb_orb_detector {{f 8} {busy 0} {done 0} {keypoint_valid 0}}
cd $saved_dir
puts "ALL FAST REGRESSION TESTS PASSED"
checked v2_work tb_border {{received 1080} {empty 1}}
checked v2_work tb_hybrid {{received 437} {empty 1} {i 9}}
checked v2_work tb_nms_v2 {{map 3}}
checked v2_work tb_detector_v2 {{f 16} {busy 0} {done 0} {keypoint_valid 0}}
puts "ALL MODELSIM RTL TEST COMPLETION CHECKS PASSED"
# Calibration/reference regeneration is a Python step; see run_all.sh / README.
