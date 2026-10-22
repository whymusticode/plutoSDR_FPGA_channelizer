# Helpers for out-of-tree Pluto projects, sourced from a project's hdl/bd.tcl.
# The stock v0.39 block design is built first; a project then edits it.

# Detach cpack's inputs so the project can drive the RX DMA stream itself.
# Only cpack's side is disconnected: the stock sources (RX FIR decimator, RX2
# ADC lanes) stay instantiated and still reachable by the project, and Vivado
# trims whatever no longer reaches an output. Every stock AXI peripheral stays
# on the interconnect, so the stock device trees remain valid.
proc pluto_detach_rx_dma {} {
  foreach pin {enable_0 enable_1 enable_2 enable_3
               fifo_wr_data_0 fifo_wr_data_1 fifo_wr_data_2 fifo_wr_data_3
               fifo_wr_en} {
    set p [get_bd_pins cpack/$pin]
    set net [get_bd_nets -quiet -of_objects $p]
    if {$net ne ""} { disconnect_bd_net $net $p }
  }
}

# Drive the RX DMA with a project stream: four 16-bit lanes
# {rx0_i, rx0_q, rx1_i, rx1_q} and one write strobe. cpack's enables must be the
# static IIO channel enables: cpack resets its data path whenever `enable`
# changes or is all zero, so tying them to a strobe drops every sample.
proc pluto_connect_rx_dma {lanes wr_en} {
  set enables {adc_enable_i0 adc_enable_q0 adc_enable_i1 adc_enable_q1}
  foreach k {0 1 2 3} {
    ad_connect [lindex $lanes $k] cpack/fifo_wr_data_$k
    ad_connect axi_ad9361/[lindex $enables $k] cpack/enable_$k
  }
  ad_connect $wr_en cpack/fifo_wr_en
}

# Add project RTL by path, marking .mem files as memory initialisation files.
proc pluto_add_sources {files} {
  add_files -norecurse $files
  set mems [lsearch -all -inline -glob $files *.mem]
  if {[llength $mems]} { set_property file_type {Memory File} [get_files $mems] }
  update_compile_order -fileset sources_1
}
