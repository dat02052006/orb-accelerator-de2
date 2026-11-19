# ============================================================================
# orb_top.sdc
# Rang buoc timing cho orb_accelerator (Cyclone II / DE2), clock 50 MHz
# ============================================================================

# Clock he thong 50 MHz (chu ky 20 ns), cong clk cua orb_top
create_clock -name clk -period 20.000 [get_ports {clk}]


