# Standalone frontend check: top-level clock port is clk.
# Integrated SoC may already constrain this clock; do not duplicate constraints.
create_clock -name frontend_clk -period 20.000 [get_ports {clk}]
derive_clock_uncertainty
# External I/O delays belong to the eventual system-level integration.
