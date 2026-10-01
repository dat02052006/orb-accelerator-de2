transcript file modelsim_transcript.log
if {![file exists work]} {vlib work}
vlog ../rtl/orb_row_ram.v ../rtl/orb_line_buffer.v ../rtl/orb_window_buffer.v ../rtl/orb_pixel_frontend.v tb_orb_pixel_frontend.v
vsim work.tb_orb_pixel_frontend
add wave sim:/tb_orb_pixel_frontend/clk sim:/tb_orb_pixel_frontend/reset
add wave sim:/tb_orb_pixel_frontend/start_valid sim:/tb_orb_pixel_frontend/start_ready
add wave sim:/tb_orb_pixel_frontend/busy sim:/tb_orb_pixel_frontend/done
add wave sim:/tb_orb_pixel_frontend/pixel_valid sim:/tb_orb_pixel_frontend/pixel_ready
add wave sim:/tb_orb_pixel_frontend/pixel_data sim:/tb_orb_pixel_frontend/pixel_sof sim:/tb_orb_pixel_frontend/pixel_eol
add wave sim:/tb_orb_pixel_frontend/window_valid sim:/tb_orb_pixel_frontend/window_ready
add wave sim:/tb_orb_pixel_frontend/center_x sim:/tb_orb_pixel_frontend/center_y sim:/tb_orb_pixel_frontend/level_id
run -all
