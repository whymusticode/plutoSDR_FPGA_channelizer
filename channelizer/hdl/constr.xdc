# The AD9361 receive clock is recovered from the RF transceiver and has no
# phase relationship to PS FCLK1. All RX/PFB crossings use XPM async FIFOs.
set_clock_groups -asynchronous \
  -group [get_clocks rx_clk] \
  -group [get_clocks clk_fpga_1]
