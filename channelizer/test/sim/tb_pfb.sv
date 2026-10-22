// Feed segments of in<k>.hex (one 64-bit beat per line) through the
// channelizer at one sample every GAP+1 cycles, with random output
// back-pressure if STALL, and write every output beat to out.hex.
//
// Segment k runs at 2**LOGN<k> channels. While reset is held before it, the
// taps in coef<k>.hex (one phase per line, packed as in pfb_coeffs.mem) are
// written through the coefficient port, unless LOAD<k> is 0, in which case
// the bank runs on whatever the RAM holds (segment 0: the boot image). The
// cycles between a segment's first and last accepted beat go to cycles.txt.
`timescale 1ns/1ps
module tb_pfb;
  parameter integer NSAMP0 = 0, NSAMP1 = 0;
  parameter integer LOGN0 = 6, LOGN1 = 6;
  parameter integer LOAD0 = 0, LOAD1 = 1;
  parameter integer GAP = 7;
  parameter integer STALL = 0;
  parameter integer OUT_FRAC = 18;
  parameter integer MULT_TAPS = 4;
  parameter integer STAGE_GAP_BASE = 10;
  localparam integer N_MAX = 256, TAPS = 8;
  reg clk = 0, rstn = 0;
  always #2.5 clk = ~clk;
  reg [63:0] mem [0:NSAMP0+NSAMP1-1];
  reg [TAPS*18-1:0] coef0 [0:N_MAX-1], coef1 [0:N_MAX-1];
  reg [63:0] din;
  reg vin = 0, rdy = 1;
  reg [3:0] logn;
  reg cwe = 0;
  reg [7:0] cphase;
  reg [2:0] ctap;
  reg [17:0] cdata;
  wire tready, vout, last;
  wire [63:0] dout;
  integer i, gap, fd, fc, nout, seg, p, t, first, lastc, cycle = 0;
  pfb_channelizer #(.WIDTH(16), .FRAC(15), .N_MAX(N_MAX), .IN_LANES(4), .OUT_LANES(4),
    .OUT_FRAC(OUT_FRAC), .TAPS(TAPS), .MULT_TAPS(MULT_TAPS),
    .STAGE_GAP_BASE(STAGE_GAP_BASE)) dut (
    .aclk(clk), .aresetn(rstn), .cfg_logn(logn),
    .coef_we(cwe), .coef_wphase(cphase), .coef_wtap(ctap), .coef_wdata(cdata),
    .s_axis_tvalid(vin), .s_axis_tready(tready),
    .s_axis_tdata(din), .s_axis_tlast(1'b0), .m_axis_tvalid(vout),
    .m_axis_tready(rdy), .m_axis_tdata(dout), .m_axis_tlast(last));

  always @(posedge clk) cycle <= cycle + 1;

  task run_segment(input integer k, input integer lg, input integer load,
                   input integer base, input integer nsamp);
    integer want;
    begin
      rstn <= 0;
      logn <= lg;
      repeat (4) @(posedge clk);
      if (load)
        for (p = 0; p < (1 << lg); p = p + 1)
          for (t = 0; t < TAPS; t = t + 1) begin
            cwe <= 1; cphase <= p; ctap <= t; cdata <= (k ? coef1[p] : coef0[p]) >> (t*18);
            @(posedge clk);
          end
      cwe <= 0;
      repeat (4) @(posedge clk);
      rstn <= 1;
      want = nout + (nsamp >> lg << lg);
      i = base; gap = 0; first = -1;
      while (nout < want) begin
        @(posedge clk);
        if (vin && tready) begin
          if (first < 0) first = cycle;
          lastc = cycle;
          i = i + 1;
          if (GAP == 0 && i < base + nsamp) din <= mem[i];
          else begin vin <= 0; gap = GAP; end
        end else if (!vin) begin
          if (gap > 0) gap = gap - 1;
          else if (i < base + nsamp) begin vin <= 1; din <= mem[i]; end
        end
      end
      $fwrite(fc, "%0d %0d %0d\n", lg, nsamp, lastc - first);
    end
  endtask

  always @(posedge clk) if (rstn) begin
    if (STALL) rdy <= ($random & 3) != 0;
    if (vout && rdy) begin $fwrite(fd, "%h\n", dout); nout = nout + 1; end
  end

  initial begin
    $readmemh("in.hex", mem);
    if (LOAD0) $readmemh("coef0.hex", coef0);
    if (NSAMP1 && LOAD1) $readmemh("coef1.hex", coef1);
    fd = $fopen("out.hex", "w");
    fc = $fopen("cycles.txt", "w");
    nout = 0;
    run_segment(0, LOGN0, LOAD0, 0, NSAMP0);
    if (NSAMP1) run_segment(1, LOGN1, LOAD1, NSAMP0, NSAMP1);
    $fclose(fd); $fclose(fc);
    $finish;
  end
endmodule
