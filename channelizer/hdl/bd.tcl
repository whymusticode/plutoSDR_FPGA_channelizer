# Dual-RX PFB (runtime size 2..256 channels) in place of the stock RX path
# into cpack/DMA, configured through pfb_ctrl at 0x41210000. pfb_rx_mux can
# hand the DMA the raw receivers instead (CONTROL[1]).
# Sourced at the end of the stock v0.39 system_bd.tcl (scripts/pluto-fw).
source $::env(PLUTO_FW_SCRIPTS)/pluto_bd_helpers.tcl
set here $::env(PLUTO_PROJECT_DIR)

pluto_add_sources [list \
  "$here/hdl/pfb_async_fifo64.v" \
  "$here/hdl/pfb_dual_ingress.v" \
  "$here/hdl/pfb_dual_bin_selector.v" \
  "$here/hdl/pfb_ctrl.v" \
  "$here/hdl/pfb_rx_mux.v" \
  "$here/hdl/pfb_channelizer_wrapper.v" \
  "$here/rtl/pfb_channelizer.sv" \
  "$here/rtl/pfb_coeffs.mem" \
  "$here/rtl/twiddle.mem"]

pluto_detach_rx_dma

# The four TX DDS tone generators are ~4.1k LUTs; without them the stock
# peripherals (axi_tdd included) and the PFB fit. TX still works from DMA
# buffers, and cf_axi_dds omits the altvoltage DDS channels when the core
# reports DDS disabled.
ad_ip_parameter axi_ad9361 CONFIG.DAC_DDS_DISABLE 1

create_bd_cell -type module -reference pfb_dual_ingress pfb_ingress
create_bd_cell -type module -reference pfb_channelizer_wrapper pfb_channelizer
create_bd_cell -type module -reference pfb_dual_bin_selector pfb_bin_selector
create_bd_cell -type module -reference pfb_async_fifo64 pfb_output_fifo
create_bd_cell -type module -reference pfb_ctrl pfb_ctrl
create_bd_cell -type module -reference pfb_rx_mux pfb_rx_mux
ad_ip_instance xlconstant pfb_one
ad_ip_parameter pfb_one CONFIG.CONST_WIDTH 1
ad_ip_parameter pfb_one CONFIG.CONST_VAL 1
ad_ip_instance util_vector_logic pfb_ingress_valid [list C_OPERATION {not} C_SIZE 1]
ad_ip_instance util_vector_logic pfb_output_valid [list C_OPERATION {not} C_SIZE 1]
# The output FIFO discards a beat when full; pfb_ctrl reports it.
ad_ip_instance util_vector_logic pfb_output_drop [list C_OPERATION {and} C_SIZE 1]

foreach {k from to} {0 15 0  1 31 16  2 47 32  3 63 48} {
  ad_ip_instance xlslice pfb_out$k
  ad_ip_parameter pfb_out$k CONFIG.DIN_WIDTH 64
  ad_ip_parameter pfb_out$k CONFIG.DIN_FROM $from
  ad_ip_parameter pfb_out$k CONFIG.DIN_TO $to
}

# Pack both RX lanes before the CDC. Independent FIFOs plus a shared FFT
# scheduler were able to mis-pair two otherwise coherent receivers.
ad_connect axi_ad9361/rst pfb_ingress/rst
ad_connect axi_ad9361/l_clk pfb_ingress/rx_clk
ad_connect axi_ad9361/adc_valid_i0 pfb_ingress/rx0_valid
ad_connect axi_ad9361/adc_data_i0 pfb_ingress/rx0_i
ad_connect axi_ad9361/adc_data_q0 pfb_ingress/rx0_q
ad_connect axi_ad9361/adc_valid_i1 pfb_ingress/rx1_valid
ad_connect axi_ad9361/adc_data_i1 pfb_ingress/rx1_i
ad_connect axi_ad9361/adc_data_q1 pfb_ingress/rx1_q
ad_connect sys_200m_clk pfb_ingress/fabric_clk
ad_connect pfb_ingress/empty pfb_ingress_valid/Op1

# pfb_ctrl holds the channelizer and selector in reset while the CPU is in
# reset or reconfiguring, and hands them the size, taps and mask.
ad_connect sys_200m_clk pfb_ctrl/pfb_clk
ad_connect axi_ad9361/l_clk pfb_ctrl/in_clk
ad_connect pfb_ingress/drop pfb_ctrl/in_drop
ad_connect pfb_output_drop/Res pfb_ctrl/out_drop

ad_connect sys_200m_clk pfb_channelizer/aclk
ad_connect pfb_ctrl/pfb_resetn pfb_channelizer/aresetn
ad_connect pfb_ctrl/pfb_logn pfb_channelizer/cfg_logn
ad_connect pfb_ctrl/coef_we pfb_channelizer/coef_we
ad_connect pfb_ctrl/coef_wphase pfb_channelizer/coef_wphase
ad_connect pfb_ctrl/coef_wtap pfb_channelizer/coef_wtap
ad_connect pfb_ctrl/coef_wdata pfb_channelizer/coef_wdata
ad_connect pfb_ingress_valid/Res pfb_channelizer/s_axis_tvalid
ad_connect pfb_ingress/rd_data pfb_channelizer/s_axis_tdata
ad_connect GND pfb_channelizer/s_axis_tlast
ad_connect pfb_channelizer/s_axis_tready pfb_ingress/rd_en

ad_connect sys_200m_clk pfb_bin_selector/clk
ad_connect pfb_ctrl/pfb_reset pfb_bin_selector/rst
ad_connect pfb_ctrl/pfb_logn pfb_bin_selector/logn
ad_connect pfb_ctrl/channel_mask pfb_bin_selector/channel_mask
ad_connect pfb_channelizer/m_axis_tvalid pfb_bin_selector/s_valid
ad_connect pfb_channelizer/m_axis_tlast pfb_bin_selector/s_last
ad_connect pfb_channelizer/m_axis_tdata pfb_bin_selector/s_data
ad_connect pfb_one/dout pfb_channelizer/m_axis_tready

# Paired selected bins cross back to the AD9361/DMA clock as one 64-bit word.
ad_connect axi_ad9361/rst pfb_output_fifo/rst
ad_connect sys_200m_clk pfb_output_fifo/wr_clk
ad_connect pfb_bin_selector/m_data pfb_output_fifo/wr_data
ad_connect pfb_bin_selector/m_valid pfb_output_fifo/wr_en
ad_connect pfb_bin_selector/m_valid pfb_output_drop/Op1
ad_connect pfb_output_fifo/full pfb_output_drop/Op2
ad_connect axi_ad9361/l_clk pfb_output_fifo/rd_clk
ad_connect pfb_output_fifo/empty pfb_output_valid/Op1
ad_connect pfb_output_valid/Res pfb_output_fifo/rd_en

# The DMA takes the selected channels, or the raw receivers in bypass.
ad_connect axi_ad9361/l_clk pfb_rx_mux/clk
ad_connect pfb_ctrl/rx_bypass pfb_rx_mux/bypass
ad_connect axi_ad9361/adc_valid_i0 pfb_rx_mux/rx0_valid
ad_connect axi_ad9361/adc_data_i0 pfb_rx_mux/rx0_i
ad_connect axi_ad9361/adc_data_q0 pfb_rx_mux/rx0_q
ad_connect axi_ad9361/adc_valid_i1 pfb_rx_mux/rx1_valid
ad_connect axi_ad9361/adc_data_i1 pfb_rx_mux/rx1_i
ad_connect axi_ad9361/adc_data_q1 pfb_rx_mux/rx1_q
ad_connect pfb_output_valid/Res pfb_rx_mux/pfb_valid
ad_connect pfb_output_fifo/rd_data pfb_rx_mux/pfb_data
foreach k {0 1 2 3} { ad_connect pfb_rx_mux/m_data pfb_out$k/Din }
pluto_connect_rx_dma {pfb_out0/Dout pfb_out1/Dout pfb_out2/Dout pfb_out3/Dout} \
  pfb_rx_mux/m_valid

ad_cpu_interconnect 0x41210000 pfb_ctrl
