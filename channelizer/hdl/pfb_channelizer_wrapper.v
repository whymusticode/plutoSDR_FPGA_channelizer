// Keep the block-design module reference in Verilog while the channelizer
// datapath remains SystemVerilog. While reset is held the input is drained
// and discarded, so the ingress FIFO never fills during a reconfiguration.
module pfb_channelizer_wrapper (
    input wire aclk,
    input wire aresetn,
    input wire [3:0] cfg_logn,
    input wire coef_we,
    input wire [7:0] coef_wphase,
    input wire [2:0] coef_wtap,
    input wire [17:0] coef_wdata,
    input wire s_axis_tvalid,
    output wire s_axis_tready,
    input wire signed [63:0] s_axis_tdata,
    input wire s_axis_tlast,
    output wire m_axis_tvalid,
    input wire m_axis_tready,
    output wire signed [63:0] m_axis_tdata,
    output wire m_axis_tlast
);
    wire ready;
    assign s_axis_tready = ready || !aresetn;
    pfb_channelizer #(
        .WIDTH(16), .FRAC(15), .N_MAX(256), .IN_LANES(4), .OUT_LANES(4),
        // 12-bit ADC codes sit in the low bits of the Q15 input; three more
        // output fractional bits keep the sub-LSB channel noise.
        .OUT_FRAC(18), .TAPS(8), .MULT_TAPS(4)
    ) impl (
        .aclk(aclk), .aresetn(aresetn), .cfg_logn(cfg_logn),
        .coef_we(coef_we), .coef_wphase(coef_wphase), .coef_wtap(coef_wtap),
        .coef_wdata(coef_wdata),
        .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(ready),
        .s_axis_tdata(s_axis_tdata), .s_axis_tlast(s_axis_tlast),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready),
        .m_axis_tdata(m_axis_tdata), .m_axis_tlast(m_axis_tlast)
    );
endmodule
