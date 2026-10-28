// pfb_ctrl + pfb_channelizer_wrapper + pfb_dual_bin_selector, driven over
// AXI-Lite exactly as the host does: hold reset at size 2**LOGN, write the
// taps in coef.hex (one phase per line), write and commit mask0.hex (one
// 32-bit word per line), release, stream in.hex every GAP+1 fabric cycles,
// and once SWITCH_AT samples have gone in commit mask1.hex. Selected beats
// go to sel.hex; register readbacks go to regs.txt. Finally CONTROL[1] switches
// pfb_rx_mux to a fixed raw sample, which goes to regs.txt too.
`timescale 1ns/1ps
module tb_system;
  parameter integer NSAMP = 0;
  parameter integer LOGN = 6;
  parameter integer GAP = 7;
  parameter integer SWITCH_AT = 0;
  localparam integer N_MAX = 256, TAPS = 8;

  reg aclk = 0, pclk = 0, aresetn = 0;
  always #5 aclk = ~aclk;            // 100 MHz CPU interconnect
  always #2.5 pclk = ~pclk;          // 200 MHz fabric

  reg  [11:0] awaddr, araddr;
  reg  awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
  reg  [31:0] wdata;
  wire awready, wready, bvalid, arready, rvalid;
  wire [31:0] rdata;
  wire [1:0] bresp, rresp;

  wire presetn, preset, coef_we;
  wire [3:0] plogn;
  wire [7:0] coef_wphase;
  wire [2:0] coef_wtap;
  wire [17:0] coef_wdata;
  wire [N_MAX-1:0] mask;
  wire bypass;

  pfb_ctrl ctrl (
    .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
    .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(4'hf), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
    .pfb_clk(pclk), .pfb_resetn(presetn), .pfb_reset(preset), .pfb_logn(plogn),
    .coef_we(coef_we), .coef_wphase(coef_wphase), .coef_wtap(coef_wtap), .coef_wdata(coef_wdata),
    .channel_mask(mask), .in_clk(pclk), .in_drop(1'b0), .out_drop(1'b0), .rx_bypass(bypass));

  reg  vin = 0;
  reg  [63:0] din;
  wire tready, cvalid, clast;
  wire [63:0] cdata;
  pfb_channelizer_wrapper core (
    .aclk(pclk), .aresetn(presetn), .cfg_logn(plogn),
    .coef_we(coef_we), .coef_wphase(coef_wphase), .coef_wtap(coef_wtap), .coef_wdata(coef_wdata),
    .s_axis_tvalid(vin), .s_axis_tready(tready), .s_axis_tdata(din), .s_axis_tlast(1'b0),
    .m_axis_tvalid(cvalid), .m_axis_tready(1'b1), .m_axis_tdata(cdata), .m_axis_tlast(clast));

  wire svalid;
  wire [63:0] sdata;
  pfb_dual_bin_selector sel (
    .clk(pclk), .rst(preset), .logn(plogn), .channel_mask(mask),
    .s_valid(cvalid), .s_last(clast), .s_data(cdata), .m_valid(svalid), .m_data(sdata));

  reg  rawv = 0;
  wire mvalid;
  wire [63:0] mdata;
  pfb_rx_mux mux (
    .clk(pclk), .bypass(bypass),
    .rx0_valid(rawv), .rx0_i(16'hf801), .rx0_q(16'h07ff),
    .rx1_valid(rawv), .rx1_i(16'h0001), .rx1_q(16'hffff),
    .pfb_valid(svalid), .pfb_data(sdata), .m_valid(mvalid), .m_data(mdata));
  integer nmux = 0, nraw = 0;
  reg [63:0] raw_seen = 0, pfb_mismatch = 0;
  reg [63:0] sdata_q;
  reg svalid_q = 0;
  always @(posedge pclk) begin
    svalid_q <= svalid; sdata_q <= sdata;
    if (mvalid && !bypass) begin
      nmux = nmux + 1;
      if (!svalid_q || mdata != sdata_q) pfb_mismatch = pfb_mismatch + 1;
    end
    if (mvalid && bypass) begin nraw = nraw + 1; raw_seen = mdata; end
  end

  task axi_write(input [11:0] a, input [31:0] d);
    begin
      @(posedge aclk);
      awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
      @(posedge aclk);
      while (!(awready && wready)) @(posedge aclk);
      awvalid <= 0; wvalid <= 0; bready <= 1;
      @(posedge aclk);
      while (!bvalid) @(posedge aclk);
      bready <= 0;
    end
  endtask

  task axi_read(input [11:0] a, output [31:0] d);
    begin
      @(posedge aclk);
      araddr <= a; arvalid <= 1;
      @(posedge aclk);
      while (!arready) @(posedge aclk);
      arvalid <= 0; rready <= 1;
      while (!rvalid) @(posedge aclk);
      d = rdata;
      @(posedge aclk);
      rready <= 0;
    end
  endtask

  reg [63:0] mem [0:NSAMP-1];
  reg [TAPS*18-1:0] coef [0:N_MAX-1];
  reg [31:0] mask0 [0:7], mask1 [0:7];
  reg [31:0] r;
  integer fd, fr, i = 0, gap = 0, p, t, nout = 0, streaming = 0;

  always @(posedge pclk) begin
    if (svalid) begin $fwrite(fd, "%h\n", sdata); nout = nout + 1; end
    if (streaming) begin
      if (vin && tready) begin vin <= 0; i = i + 1; gap = GAP; end
      else if (!vin) begin
        if (gap > 0) gap = gap - 1;
        else if (i < NSAMP) begin vin <= 1; din <= mem[i]; end
      end
    end
  end

  initial begin
    $readmemh("in.hex", mem);
    $readmemh("coef.hex", coef);
    $readmemh("mask0.hex", mask0);
    $readmemh("mask1.hex", mask1);
    fd = $fopen("sel.hex", "w");
    fr = $fopen("regs.txt", "w");
    repeat (10) @(posedge aclk);
    aresetn <= 1;
    repeat (4) @(posedge aclk);
    axi_read(12'h000, r); $fwrite(fr, "id %h\n", r);
    axi_read(12'h004, r); $fwrite(fr, "build %h\n", r);
    axi_read(12'h008, r); $fwrite(fr, "control_boot %h\n", r);
    axi_write(12'h008, 32'h1 | (LOGN << 8));
    axi_write(12'h010, 0);
    for (p = 0; p < (1 << LOGN); p = p + 1)
      for (t = 0; t < TAPS; t = t + 1)
        axi_write(12'h014, (coef[p] >> (t * 18)) & 18'h3ffff);
    axi_read(12'h010, r); $fwrite(fr, "coef_idx %h\n", r);
    for (p = 0; p < 8; p = p + 1) axi_write(12'h040 + 4 * p, mask0[p]);
    axi_write(12'h018, 0);
    axi_read(12'h00c, r); $fwrite(fr, "status_held %h\n", r);
    axi_write(12'h008, LOGN << 8);
    axi_read(12'h008, r); $fwrite(fr, "control %h\n", r);
    axi_read(12'h044, r); $fwrite(fr, "mask1 %h\n", r);
    repeat (300) @(posedge pclk);     // the core clears its history first
    streaming = 1;
    if (SWITCH_AT) begin
      wait (i >= SWITCH_AT);
      for (p = 0; p < 8; p = p + 1) axi_write(12'h040 + 4 * p, mask1[p]);
      axi_write(12'h018, 0);
    end
    wait (i >= NSAMP);
    repeat (4000) @(posedge pclk);
    axi_read(12'h00c, r); $fwrite(fr, "status %h\n", r);
    $fwrite(fr, "mux_beats %0d\n", nmux - nout);
    $fwrite(fr, "mux_mismatch %0d\n", pfb_mismatch);
    axi_write(12'h008, (LOGN << 8) | 2);
    axi_read(12'h008, r); $fwrite(fr, "control_bypass %h\n", r);
    repeat (20) @(posedge pclk);
    rawv <= 1; repeat (5) @(posedge pclk); rawv <= 0;
    repeat (4) @(posedge pclk);
    $fwrite(fr, "raw_beats %0d\n", nraw);
    $fwrite(fr, "raw %h\n", raw_seen);
    $fclose(fd); $fclose(fr);
    $finish;
  end
endmodule
