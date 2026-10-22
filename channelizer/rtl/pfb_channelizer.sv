// Dual-receiver polyphase channelizer for the ADALM-Pluto, with a runtime
// channel count: any power of two n = 2**cfg_logn from 2 to N_MAX.
//
// One AXI4-Stream beat is one time sample carrying both receivers:
//   tdata = {rx1.imag, rx1.real, rx0.imag, rx0.real}, lane 0 in the LSBs.
// One frame of n beats produces one frame of n bins in the same lane order,
// permanently bit-reversed - which is what a decimation-in-frequency FFT
// emits from naturally ordered input, so no reordering network is needed.
//
// Three engines run concurrently on a two-frame ping-pong buffer, which is
// what lets input acceptance, transform and output drain overlap:
//   * the polyphase FIR consumes each beat as it arrives, TAPS taps deep,
//   * the FFT transforms the frame the FIR has just finished, in place,
//   * the output stage drains that same frame while the FIR fills the other.
//
// Configuration.  cfg_logn is taken while aresetn is low and held until the
// next reset, so a size change is: hold reset, write the coefficients for the
// new size through the coef_* port (which works at any time), release.  Every
// size uses the low n entries of memories sized for N_MAX, and the FFT strides
// through one N_MAX twiddle table.
//
// Fixed point.  Input samples are Q(WIDTH-1-FRAC).FRAC; output samples carry
// OUT_FRAC fractional bits, so OUT_FRAC > FRAC keeps resolution a narrow
// channel needs when the input only uses its low bits (Pluto: 12-bit ADC
// codes in a Q15 word, channel noise below one input LSB).
// Coefficients are Q(COEF_FRAC_BASE + logn): a unity-DC prototype's largest
// tap is about 1/n, so this keeps it near the top of COEF_W for every n, and
// the FIR's output shift grows with logn to match.  Internally every complex
// value is DATA_W bits at DATA_FRAC fractional bits, sized so the FFT's
// worst-case gain of n cannot overflow: n times the largest per-phase |tap|
// sum is about 1.6-1.8 for the 8-tap equiripple prototypes at every size, so a
// full-scale input reaches ~7.3e6 against the 1.6e7 a 25-bit word holds (the
// host checks each prototype against this).  The datapath is 25x18
// throughout, which is exactly one DSP48E1 multiplier.
module pfb_channelizer #(
  parameter integer WIDTH = 16,
  parameter integer FRAC = 15,
  parameter integer N_MAX = 256,       // largest channel count; a power of two
  parameter integer IN_LANES = 4,
  parameter integer OUT_LANES = 4,
  parameter integer OUT_FRAC = FRAC,   // output fractional bits, <= DATA_FRAC - 1
  parameter integer TAPS = 8,          // prototype taps per phase
  parameter integer MULT_TAPS = 4,     // taps multiplied per cycle; divides TAPS
  // Idle cycles between FFT stages are STAGE_GAP_BASE - n/4 when positive, see
  // FFT_GAP.  Simulation fails at 8 and passes at 9 = WRITE_LATENCY.
  parameter integer STAGE_GAP_BASE = 10
) (
  input  wire aclk,
  input  wire aresetn,
  input  wire [3:0] cfg_logn,          // log2 of the channel count, 1..log2(N_MAX)
  input  wire coef_we,
  input  wire [$clog2(N_MAX)-1:0] coef_wphase,
  input  wire [(TAPS > 1 ? $clog2(TAPS) : 1)-1:0] coef_wtap,
  input  wire [17:0] coef_wdata,
  input  wire s_axis_tvalid,
  output wire s_axis_tready,
  input  wire signed [IN_LANES*WIDTH-1:0] s_axis_tdata,
  input  wire s_axis_tlast,
  output wire m_axis_tvalid,
  input  wire m_axis_tready,
  output wire signed [OUT_LANES*WIDTH-1:0] m_axis_tdata,
  output wire m_axis_tlast
);
  localparam integer LANES     = IN_LANES;          // real lanes per beat
  localparam integer LOGN      = $clog2(N_MAX);       // address bits for N_MAX
  localparam integer COEF_W    = 18;
  localparam integer COEF_FRAC_BASE = 16;              // + logn: the taps' Q format
  localparam integer TWID_W    = 18;
  localparam integer TWID_FRAC = 17;
  localparam integer DATA_W    = 25;
  localparam integer DATA_FRAC = 22;
  localparam integer ACC_W     = WIDTH + COEF_W + $clog2(TAPS);  // TAPS products
  localparam integer FOLD      = TAPS / MULT_TAPS;       // passes per sample
  localparam integer FOLD_W    = FOLD > 1 ? $clog2(FOLD) : 1;
  localparam integer LOG_MT    = $clog2(MULT_TAPS);
  localparam integer PROD_W    = DATA_W + TWID_W;
  localparam integer FIR_SHIFT_BASE = FRAC + COEF_FRAC_BASE - DATA_FRAC;  // + logn
  localparam integer FIR_SHIFT_MIN  = FIR_SHIFT_BASE + 1;
  localparam integer OUT_SHIFT = DATA_FRAC - OUT_FRAC;
  localparam integer MEM_W     = LANES * DATA_W;          // one bin, both rx
  localparam integer HIST_W    = LANES * WIDTH;           // one sample, both rx
  localparam integer HALF      = N_MAX / 2;
  localparam integer ADDR_W    = LOGN - 1;                // index within a bank

  integer lane, slot;

  // Handshakes between the three engines, declared here so each section below
  // can be read in dataflow order.
  localparam [2:0] FFT_IDLE = 3'd0, FFT_RUN = 3'd1, FFT_DRAIN = 3'd2,
                   FFT_OUT = 3'd3, FFT_GAP = 3'd4;
  reg [2:0] fft_state;
  wire fft_hold;                    // the FIR owns the memory write ports
  reg  p5_valid;                    // a butterfly result is waiting to be written
  wire fft_start;                   // the FFT is taking the finished frame
  reg  out_done;                    // the last output beat has been presented
  reg  out_read_bank;               // bank the in-flight output read came from
  reg  frame_ready;                 // a complete frame is waiting for the FFT
  reg  fir_buf;                     // ping-pong half the FIR is filling

  // ------------------------------------------------------------------
  // Configuration, fixed from one reset to the next
  // ------------------------------------------------------------------
  reg [3:0]      logn;
  reg [LOGN-1:0] n_last;            // n - 1
  reg [LOGN:0]   n_count;           // n, for the output counter that reaches it
  reg [LOGN-2:0] half_last;         // n/2 - 1, the last butterfly of a stage
  reg [2:0]      last_stage;        // logn - 1
  reg [3:0]      tw_shift;          // LOGN - logn: twiddle stride exponent
  reg [4:0]      stage_gap;         // idle cycles between FFT stages, see FFT_GAP
  reg [2:0]      fir_shift_off;     // logn - 1: FIR shift beyond FIR_SHIFT_MIN
  reg signed [ACC_W-1:0] fir_round; // half an LSB after fir_shift
  always @(posedge aclk)
    if (!aresetn) begin
      logn       <= cfg_logn;
      n_last     <= (1 << cfg_logn) - 1;
      n_count    <= 1 << cfg_logn;
      half_last  <= (1 << (cfg_logn - 1)) - 1;
      last_stage <= cfg_logn - 1;
      tw_shift   <= LOGN - cfg_logn;
      stage_gap  <= ((1 << cfg_logn) / 4 >= STAGE_GAP_BASE) ? 4'd0
                    : STAGE_GAP_BASE - (1 << cfg_logn) / 4;
      fir_shift_off <= cfg_logn - 1;
      fir_round  <= {{(ACC_W-1){1'b0}}, 1'b1} << (FIR_SHIFT_BASE + cfg_logn - 1);
    end

  // ------------------------------------------------------------------
  // Coefficient RAM and twiddle ROM
  // ------------------------------------------------------------------
  // One RAM per tap, so the host can write a tap at a time; together they
  // read a whole phase per cycle.  Block RAM: a distributed read costs 2.5 ns
  // of clock-to-output feeding the multipliers.  They boot holding the image
  // in pfb_coeffs.mem, so the bitstream starts as a 64-channel bank.
  localparam integer TAP_W = TAPS > 1 ? $clog2(TAPS) : 1;
  reg signed [2*TWID_W-1:0] twiddle_rom [0:HALF-1];
  initial $readmemh("twiddle.mem", twiddle_rom);
  wire [TAPS*COEF_W-1:0] coef_q;
  reg [LOGN-1:0]         coef_addr;

  genvar gc;
  generate
    for (gc = 0; gc < TAPS; gc = gc + 1) begin : g_coef
      (* ram_style = "block" *) reg [COEF_W-1:0] mem [0:N_MAX-1];
      reg [TAPS*COEF_W-1:0] boot [0:N_MAX-1];
      reg [COEF_W-1:0] q;
      integer p;
      initial begin
        for (p = 0; p < N_MAX; p = p + 1) boot[p] = 0;
        $readmemh("pfb_coeffs.mem", boot);
        for (p = 0; p < N_MAX; p = p + 1) mem[p] = boot[p][gc*COEF_W +: COEF_W];
      end
      always @(posedge aclk) begin
        if (coef_we && coef_wtap == gc) mem[coef_wphase] <= coef_wdata;
        q <= mem[coef_addr];
      end
      assign coef_q[gc*COEF_W +: COEF_W] = q;
    end
  endgenerate
  reg signed [2*TWID_W-1:0] twiddle_q;

  // ------------------------------------------------------------------
  // Frame buffers.  Two banks selected by the parity of the bin index: a
  // radix-2 butterfly pair differs in exactly one address bit, so its two
  // operands always have opposite parity and therefore always sit in
  // different banks.  That is what allows both to be read, and both written,
  // in the same cycle.  The buffer bit above them is the ping-pong half.
  // ------------------------------------------------------------------
  reg [MEM_W-1:0] bank0 [0:2*HALF-1];
  reg [MEM_W-1:0] bank1 [0:2*HALF-1];
  reg [MEM_W-1:0] bank0_q, bank1_q;
  reg             bank0_we, bank1_we;
  reg [ADDR_W:0]  bank0_waddr, bank1_waddr, bank0_raddr, bank1_raddr;
  reg [MEM_W-1:0] bank0_wdata, bank1_wdata;

  // The memory read is itself a pipeline stage, so it has to freeze with the
  // rest of the butterfly pipeline: the address generator has already moved
  // on to the next pair, and letting its read land would hand those operands
  // to the butterfly that is still waiting.  Draining does not use the
  // butterfly pipeline, so reads stay enabled there.
  wire bank_re = (fft_state == FFT_OUT) || !fft_hold;

  always @(posedge aclk) begin
    if (bank0_we) bank0[bank0_waddr] <= bank0_wdata;
    if (bank_re) bank0_q <= bank0[bank0_raddr];
  end
  always @(posedge aclk) begin
    if (bank1_we) bank1[bank1_waddr] <= bank1_wdata;
    if (bank_re) bank1_q <= bank1[bank1_raddr];
  end

  function automatic parity;
    input [LOGN-1:0] index;
    begin
      parity = ^index;
    end
  endfunction

  // ------------------------------------------------------------------
  // Scaling helpers.  Each one shifts by a fixed amount and clamps, at the
  // exact width of its operand: written against a 64-bit temporary instead,
  // these synthesise into 64-bit adders and comparators and cost more than
  // the butterfly they serve.  The round-half-up constant is folded into
  // whatever produced the value, which on this device is a free DSP
  // post-addition rather than a second carry chain in the timing path.
  //
  // Clamping is a comparison of the bits above the result's sign bit against
  // that sign bit, so it costs one lookup table rather than two magnitude
  // comparisons.
  // ------------------------------------------------------------------
  localparam integer FIR_OUT_W  = ACC_W - FIR_SHIFT_MIN;   // widest shifted branch
  localparam integer TWID_OUT_W = PROD_W + 2 - TWID_FRAC;
  localparam integer SAMP_OUT_W = DATA_W - OUT_SHIFT;
  localparam signed [PROD_W:0]   TWID_ROUND = {{(PROD_W+1-TWID_FRAC){1'b0}}, 1'b1,
                                               {(TWID_FRAC-1){1'b0}}};
  localparam signed [DATA_W-1:0] SAMP_ROUND = {{(DATA_W-OUT_SHIFT){1'b0}}, 1'b1,
                                               {(OUT_SHIFT-1){1'b0}}};

  // A branch value is shifted by the runtime fir_shift in one cycle and
  // clamped to the datapath in the next.  Only the smallest banks can need
  // the clamp, and only for inputs beyond the 12-bit ADC's range.
  function automatic signed [DATA_W-1:0] fir_clamp;
    input signed [FIR_OUT_W-1:0] value;
    begin
      if (FIR_OUT_W <= DATA_W)
        fir_clamp = value;
      else if (value[FIR_OUT_W-1:DATA_W-1] == {(FIR_OUT_W-DATA_W+1){value[FIR_OUT_W-1]}})
        fir_clamp = value[DATA_W-1:0];
      else if (value[FIR_OUT_W-1])
        fir_clamp = {1'b1, {(DATA_W-1){1'b0}}};
      else
        fir_clamp = {1'b0, {(DATA_W-1){1'b1}}};
    end
  endfunction

  function automatic signed [DATA_W-1:0] twiddle_scale;
    input signed [PROD_W+1:0] value;
    reg signed [TWID_OUT_W-1:0] scaled;
    begin
      scaled = value[PROD_W+1:TWID_FRAC];
      if (scaled[TWID_OUT_W-1:DATA_W-1] ==
          {(TWID_OUT_W-DATA_W+1){scaled[TWID_OUT_W-1]}})
        twiddle_scale = scaled[DATA_W-1:0];
      else if (scaled[TWID_OUT_W-1])
        twiddle_scale = {1'b1, {(DATA_W-1){1'b0}}};
      else
        twiddle_scale = {1'b0, {(DATA_W-1){1'b1}}};
    end
  endfunction

  function automatic signed [WIDTH-1:0] to_sample;
    input signed [DATA_W-1:0] value;   // the rounding constant is already in
    reg signed [SAMP_OUT_W-1:0] scaled;
    begin
      scaled = value[DATA_W-1:OUT_SHIFT];
      if (scaled[SAMP_OUT_W-1:WIDTH-1] ==
          {(SAMP_OUT_W-WIDTH+1){scaled[SAMP_OUT_W-1]}})
        to_sample = scaled[WIDTH-1:0];
      else if (scaled[SAMP_OUT_W-1])
        to_sample = {1'b1, {(WIDTH-1){1'b0}}};
      else
        to_sample = {1'b0, {(WIDTH-1){1'b1}}};
    end
  endfunction

  function automatic signed [DATA_W-1:0] clamp;
    input signed [DATA_W:0] value;
    begin
      if (value[DATA_W] == value[DATA_W-1]) clamp = value[DATA_W-1:0];
      else if (value[DATA_W])               clamp = {1'b1, {(DATA_W-1){1'b0}}};
      else                                  clamp = {1'b0, {(DATA_W-1){1'b1}}};
    end
  endfunction

  // ------------------------------------------------------------------
  // Polyphase FIR.  TAPS-1 history memories hold the frames the prototype
  // still reaches back to; each arriving sample reads them all at its own
  // phase and shifts itself in.  The multipliers cover MULT_TAPS taps, so a
  // sample's TAPS taps are folded over FOLD = TAPS / MULT_TAPS consecutive
  // passes and accumulated: TAPS buys stopband at the cost of cycles per
  // sample, not DSPs.  At 25.6 MS/s into a 200 MHz fabric there are ~7.8.
  // A frame is the n phases 0..n_last; the memories are sized for N_MAX.
  // ------------------------------------------------------------------
  wire [HIST_W-1:0] hist_q [1:TAPS-1];   // hist_q[k]: k frames back
  reg  [HIST_W-1:0] hist_d [1:TAPS-1];
  reg               hist_we;
  reg  [LOGN-1:0]   hist_raddr, hist_waddr;
  reg               clearing;       // zeroing the delay line after reset

  // Independent read and write addressing.  Sharing one address register
  // serialised the whole FIR onto one sample at a time; a sample's write goes
  // to the phase it came from while later samples are already reading theirs,
  // and the next frame does not revisit a phase for another n accepts.
  genvar gk;
  generate
    for (gk = 1; gk < TAPS; gk = gk + 1) begin : g_hist
      (* ram_style = "block" *) reg [HIST_W-1:0] mem [0:N_MAX-1];
      reg [HIST_W-1:0] q;
      // Zeroing the output register while clearing uses the block RAM's
      // own output reset, so the delay line's write data needs no mux: a
      // BRAM-to-BRAM route through one was the critical path at 256 deep.
      always @(posedge aclk) begin
        if (hist_we) mem[hist_waddr] <= hist_d[gk];
        if (clearing) q <= 0;
        else q <= mem[hist_raddr];
      end
      assign hist_q[gk] = q;
    end
  endgenerate

  // Stages: issue the reads, wait for the memories, register their outputs
  // and shift the delay line, hold them for the multipliers, then FOLD
  // multiply passes, an adder tree one level per cycle, and the pass
  // accumulator.  A valid bit and the phase travel alongside.  Stages 3 and 4
  // only load on a valid sample, so a sample's operands stay put for all of
  // its passes; accepts are FOLD cycles apart, which is exactly long enough.
  reg [LOGN-1:0]   fir_phase;      // phase of the next beat to accept
  reg [LOGN-1:0]   clear_addr;
  reg              frame_pending;  // a whole frame is accepted, awaiting the FFT
  reg [FOLD_W-1:0] accept_gap;     // cycles until the next beat may be accepted

  reg [MEM_W-1:0]  fir_result;
  reg [LOGN-1:0]   fir_result_index;
  reg              fir_result_valid, fir_result_buf;

  reg              v1, v2, v3, v4;
  reg [LOGN-1:0]   i1, i2, i3, i4;
  reg [HIST_W-1:0] s1, s2, s3, s4;
  reg [HIST_W-1:0] hist_r [1:TAPS-1];
  reg [HIST_W-1:0] hist_r2 [1:TAPS-1];
  reg [TAPS*COEF_W-1:0] coef_r, coef_r2;

  // Pass issue, then the product/tree/accumulator pipeline.
  reg [FOLD_W-1:0] pass_next;      // next pass of the sample in stage 4
  wire             pass_issue = v4 || (pass_next != 0);
  wire [FOLD_W-1:0] pass_sel = v4 ? {FOLD_W{1'b0}} : pass_next;
  // This pass's operands, registered before the multiply: selecting them in
  // the multiply's own cycle put the multiplexer behind the DSPs instead of
  // in front, which duplicated multipliers and missed 200 MHz by 1.8 ns.
  reg              sv;
  reg [FOLD_W-1:0] sp;
  reg [LOGN-1:0]   si;
  reg signed [WIDTH-1:0]  op_d [0:MULT_TAPS*LANES-1];
  reg signed [COEF_W-1:0] op_c [0:MULT_TAPS-1];
  reg              tv [0:LOG_MT];  // valid per tree level, 0 = products
  reg [FOLD_W-1:0] tp [0:LOG_MT];  // pass per tree level
  reg [LOGN-1:0]   ti [0:LOG_MT];  // phase per tree level
  reg signed [ACC_W-1:0] tree [0:LOG_MT][0:MULT_TAPS*LANES-1];
  reg signed [ACC_W-1:0] fir_acc [0:LANES-1];
  reg              acc_valid;
  reg [LOGN-1:0]   acc_index;

  wire fir_accept = s_axis_tvalid && s_axis_tready;
  assign s_axis_tready = !clearing && !frame_pending && accept_gap == 0;

  // Tap 0 meets the oldest frame, exactly as the reference orders its window.
  reg [HIST_W-1:0] fir_tapdata [0:TAPS-1];
  integer tk;
  always @* begin
    for (tk = 0; tk < TAPS - 1; tk = tk + 1) fir_tapdata[tk] = hist_r2[TAPS - 1 - tk];
    fir_tapdata[TAPS - 1] = s4;
  end

  // The delay line shifts as soon as its old contents have been read, which
  // is the same cycle they are registered for the multipliers.
  always @* begin
    hist_we    = clearing || v2;
    hist_waddr = clearing ? clear_addr : i2;
    hist_d[1]  = clearing ? {HIST_W{1'b0}} : s2;
    for (tk = 2; tk < TAPS; tk = tk + 1)
      hist_d[tk] = hist_q[tk - 1];   // zero while clearing, see g_hist
  end

  integer tap, lvl, j;
  always @(posedge aclk) begin
    if (!aresetn) begin
      // Clear the delay line so a reset really does start from silence.
      clearing <= 1'b1;
      clear_addr <= 0;
      fir_phase <= 0;
      frame_pending <= 1'b0;
      frame_ready <= 1'b0;
      fir_buf <= 1'b0;
      accept_gap <= 0;
      pass_next <= 0;
      acc_valid <= 1'b0;
      v1 <= 1'b0; v2 <= 1'b0; v3 <= 1'b0; v4 <= 1'b0; sv <= 1'b0;
      for (lvl = 0; lvl <= LOG_MT; lvl = lvl + 1) tv[lvl] <= 1'b0;
    end else begin
      if (clearing) begin
        clear_addr <= clear_addr + 1'b1;
        if (clear_addr == N_MAX - 1) clearing <= 1'b0;
      end

      // Stage 1: issue the delay-line and coefficient reads for this phase.
      v1 <= fir_accept;
      if (fir_accept) begin
        hist_raddr <= fir_phase;
        coef_addr <= fir_phase;
        s1 <= s_axis_tdata;
        i1 <= fir_phase;
        if (fir_phase == n_last) begin
          fir_phase <= 0;
          frame_pending <= 1'b1;
        end else
          fir_phase <= fir_phase + 1'b1;
        accept_gap <= FOLD - 1;
      end else if (accept_gap != 0)
        accept_gap <= accept_gap - 1'b1;

      // Stage 2: the memories are reading.
      v2 <= v1;
      if (v1) begin s2 <= s1; i2 <= i1; end

      // Stage 3: register what they returned.  The delay line is shifted
      // combinationally this cycle, above.
      v3 <= v2;
      if (v2) begin
        s3 <= s2; i3 <= i2;
        for (tk = 1; tk < TAPS; tk = tk + 1) hist_r[tk] <= hist_q[tk];
        coef_r <= coef_q;
      end

      // Stage 4: hold the memory outputs one more cycle.  Vivado packs the
      // last register before a multiplier into the DSP itself, so without
      // this one the coefficient memory's output would have to route all the
      // way to the DSPs - and a distributed read is already 2.5 ns of
      // clock-to-output before that route even starts.
      v4 <= v3;
      if (v3) begin
        s4 <= s3; i4 <= i3;
        for (tk = 1; tk < TAPS; tk = tk + 1) hist_r2[tk] <= hist_r[tk];
        coef_r2 <= coef_r;
      end

      // Stage 5: select one pass's operands, MULT_TAPS taps of every lane.
      if (pass_issue)
        pass_next <= (pass_sel == FOLD - 1) ? {FOLD_W{1'b0}} : pass_sel + 1'b1;
      sv <= pass_issue; sp <= pass_sel; si <= i4;
      for (j = 0; j < MULT_TAPS; j = j + 1) begin
        op_c[j] <= coef_r2[(pass_sel*MULT_TAPS + j)*COEF_W +: COEF_W];
        for (lane = 0; lane < LANES; lane = lane + 1)
          op_d[j*LANES + lane] <= fir_tapdata[pass_sel*MULT_TAPS + j][lane*WIDTH +: WIDTH];
      end

      // Stage 6: multiply them.
      tv[0] <= sv; tp[0] <= sp; ti[0] <= si;
      // The rounding constant rides in the first tap's DSP post-adder, so
      // the adder tree below is two-input at every level and can chain
      // through the DSPs instead of falling back into fabric carry logic.
      for (j = 0; j < MULT_TAPS; j = j + 1)
        for (lane = 0; lane < LANES; lane = lane + 1)
          // Written as a branch, not a ternary with an untyped zero: an
          // unsigned operand anywhere in the sum would make the whole
          // expression unsigned and drop every product's sign.
          if (j == 0 && sp == 0)
            tree[0][j*LANES + lane] <= op_d[j*LANES + lane] * op_c[j] + fir_round;
          else
            tree[0][j*LANES + lane] <= op_d[j*LANES + lane] * op_c[j];

      // Adder tree, one level per cycle: level l holds MULT_TAPS >> l sums
      // per lane, element t*LANES + lane.
      for (lvl = 1; lvl <= LOG_MT; lvl = lvl + 1) begin
        tv[lvl] <= tv[lvl - 1]; tp[lvl] <= tp[lvl - 1]; ti[lvl] <= ti[lvl - 1];
        for (j = 0; j < (MULT_TAPS >> lvl); j = j + 1)
          for (lane = 0; lane < LANES; lane = lane + 1)
            tree[lvl][j*LANES + lane] <= tree[lvl - 1][(2*j)*LANES + lane]
                                       + tree[lvl - 1][(2*j + 1)*LANES + lane];
      end

      // Pass accumulator: the last pass completes the branch value.
      acc_valid <= tv[LOG_MT] && tp[LOG_MT] == FOLD - 1;
      if (tv[LOG_MT]) begin
        acc_index <= ti[LOG_MT];
        for (lane = 0; lane < LANES; lane = lane + 1)
          fir_acc[lane] <= (tp[LOG_MT] == 0) ? tree[LOG_MT][lane]
                                              : fir_acc[lane] + tree[LOG_MT][lane];
      end

      // The frame is only complete once its last branch value has reached the
      // buffer, which is a cycle behind the accumulator.
      if (fir_result_valid && fir_result_index == n_last) frame_ready <= 1'b1;
      if (frame_ready && fft_start) begin
        frame_ready <= 1'b0;
        frame_pending <= 1'b0;
        fir_buf <= ~fir_buf;
      end
    end
  end

  // The branch value is shifted a cycle behind the accumulator and clamped
  // into the datapath a cycle after that.  fir_buf is captured with the shift:
  // it only flips once the frame's last result has been written.
  reg              sh_valid, sh_buf;
  reg [LOGN-1:0]   sh_index;
  reg signed [FIR_OUT_W-1:0] fir_shifted [0:LANES-1];
  always @(posedge aclk) begin
    if (!aresetn) begin
      sh_valid <= 1'b0;
      fir_result_valid <= 1'b0;
    end else begin
      sh_valid <= acc_valid;
      sh_index <= acc_index;
      sh_buf <= fir_buf;
      for (lane = 0; lane < LANES; lane = lane + 1)
        fir_shifted[lane] <= (fir_acc[lane] >>> FIR_SHIFT_MIN) >>> fir_shift_off;
      fir_result_valid <= sh_valid;
      fir_result_index <= sh_index;
      fir_result_buf <= sh_buf;
      for (lane = 0; lane < LANES; lane = lane + 1)
        fir_result[lane*DATA_W +: DATA_W] <= fir_clamp(fir_shifted[lane]);
    end
  end

  // ------------------------------------------------------------------
  // Radix-2 decimation-in-frequency FFT, one butterfly per cycle, in place.
  // ------------------------------------------------------------------
  reg [2:0]        fft_stage;
  // Per-stage addressing, stepped with fft_stage rather than derived from it:
  // span_log = logn - 1 - stage, span_mask = 2**span_log - 1 and
  // tw_exp = stage + tw_shift.  Deriving them put an adder in front of two
  // variable shifts.
  reg [2:0]        span_log;
  reg [LOGN-2:0]   span_mask;
  reg [3:0]        tw_exp;
  reg [ADDR_W-1:0] fft_count;        // butterfly within the stage
  reg              fft_buf;
  reg [4:0]        fft_flush;
  // Cycles from a butterfly issuing its address to its result reaching the
  // frame memory: address register, memory, p1/ps/p2/pm/p3/p4/p5, then the
  // registered write port.
  localparam integer WRITE_LATENCY = 9;
  localparam [4:0]  FINAL_DRAIN = WRITE_LATENCY + 4;
  assign           fft_start = (fft_state == FFT_IDLE) && frame_ready;
  // The FIR owns the write ports whenever it has a result, so the butterfly
  // pipeline holds for that cycle - but only when there is actually a
  // butterfly result to write.  The two contend for the write ports, not for
  // the memory, so a FIR result arriving while the pipeline is still filling
  // costs nothing.
  assign           fft_hold = fir_result_valid && p5_valid;

  reg [LOGN-1:0]   ia, ib;
  reg [LOGN-2:0]   tw_index;
  // The address generator's variable shifts are four lookup tables deep, so
  // its result is registered before it drives a memory: a block RAM address
  // arriving through logic cannot also absorb the routing to get there.
  reg [LOGN-1:0]   a_ia, a_ib;
  reg [LOGN-2:0]   a_tw;
  reg              a_valid;
  always @* begin : address_generation
    integer j, group;
    j = fft_count & span_mask;
    group = fft_count >> span_log;
    ia = (group << (span_log + 1)) | j;
    ib = ia | (span_mask + 1);
    tw_index = j << tw_exp;
  end

  // Pipeline: issue -> memory -> add/subtract -> multiply -> combine ->
  // round -> write.  Combining and rounding are separate stages because
  // together they are two chained 44-bit carry chains, which is what held
  // the first version of this design to half the target clock.
  reg              p1_valid, ps_valid, p2_valid, pm_valid, p3_valid, p4_valid;
  reg [LOGN-1:0]   p1_ia, ps_ia, p2_ia, pm_ia, p3_ia, p4_ia, p5_ia;
  reg [LOGN-1:0]   p1_ib, ps_ib, p2_ib, pm_ib, p3_ib, p4_ib, p5_ib;
  // One operand register pair per multiplier rather than per butterfly.  A
  // register shared between two multipliers has to stay in the fabric, and
  // the route from there to a DSP's A pin - whose setup time is the whole
  // multiplier - is what the last 24ps of the clock period were going into.
  reg signed [DATA_W-1:0] m_a [0:4*2-1];
  reg signed [TWID_W-1:0] m_b [0:4*2-1];
  reg              p1_swap;          // operand a came out of bank1
  // Selecting the operands out of the two banks is its own stage: leaving
  // that multiplexer in front of the adder put it and a 26-bit carry chain
  // and the clamp all in one clock.
  reg [MEM_W-1:0]  ps_a, ps_b;
  reg signed [TWID_W-1:0] ps_wr, ps_wi;
  reg [MEM_W-1:0]  pm_sum, p3_sum, p4_sum, p5_sum;
  // The butterfly's add and subtract keep their carry-out, and the clamp back
  // to the datapath width happens in the next stage: chaining a comparison of
  // the top bits onto the end of a 26-bit carry chain made this the critical
  // path, and the stage after it only shuffles registers.
  reg signed [DATA_W:0] p2_sum_wide [0:LANES-1];
  reg signed [DATA_W:0] p2_dif_wide [0:LANES-1];
  reg signed [TWID_W-1:0] p2_wr, p2_wi;
  reg signed [PROD_W:0] p3_rr [0:1], p3_ii [0:1], p3_ri [0:1], p3_ir [0:1];
  reg signed [PROD_W+1:0] p4_real [0:1], p4_imag [0:1];
  reg [MEM_W-1:0]  p5_product;

  wire [MEM_W-1:0] operand_a = p1_swap ? bank1_q : bank0_q;
  wire [MEM_W-1:0] operand_b = p1_swap ? bank0_q : bank1_q;

  always @(posedge aclk) begin
    if (!aresetn) begin
      fft_state <= FFT_IDLE;
      fft_stage <= 0;
      fft_count <= 0;
      fft_buf <= 1'b0;
      fft_flush <= 0;
      a_valid <= 1'b0; p1_valid <= 1'b0; ps_valid <= 1'b0; p2_valid <= 1'b0;
      pm_valid <= 1'b0; p3_valid <= 1'b0; p4_valid <= 1'b0; p5_valid <= 1'b0;
    end else begin
      case (fft_state)
        FFT_IDLE:
          if (frame_ready) begin
            fft_buf <= fir_buf;
            fft_stage <= 0;
            span_log <= last_stage;
            span_mask <= half_last;
            tw_exp <= tw_shift;
            fft_count <= 0;
            fft_state <= FFT_RUN;
          end
        // Stages run back to back with no gap between them.  A stage reads
        // exactly what the stage before it wrote, but not in the order it
        // wrote it: butterfly k' of a stage needs indices produced by
        // butterfly k of the previous one where k - k' <= n/4, so with the
        // write landing WRITE_LATENCY cycles behind its own issue, the
        // earliest unsafe read would be n/2 - n/4 cycles away.  Holding for
        // the FIR freezes issue and pipeline together, so that margin is in
        // cycles actually taken and survives any number of holds.  Small
        // banks lack that margin, so FFT_GAP idles stage_gap cycles first.
        FFT_RUN:
          if (!fft_hold) begin
            if (fft_count == half_last) begin
              fft_count <= 0;
              if (fft_stage == last_stage) begin
                fft_flush <= FINAL_DRAIN;
                fft_state <= FFT_DRAIN;
              end else begin
                fft_stage <= fft_stage + 1'b1;
                span_log <= span_log - 1'b1;
                span_mask <= span_mask >> 1;
                tw_exp <= tw_exp + 1'b1;
                if (stage_gap != 0) begin
                  fft_flush <= stage_gap;
                  fft_state <= FFT_GAP;
                end
              end
            end else
              fft_count <= fft_count + 1'b1;
          end
        FFT_GAP:
          if (!fft_hold) begin
            if (fft_flush == 1) fft_state <= FFT_RUN;
            fft_flush <= fft_flush - 1'b1;
          end
        // The last stage still has to reach memory before the output reads it.
        FFT_DRAIN:
          if (fft_flush != 0) begin
            if (!fft_hold) fft_flush <= fft_flush - 1'b1;
          end else begin
            fft_stage <= 0;
            fft_state <= FFT_OUT;
          end
        FFT_OUT:
          if (out_done) fft_state <= FFT_IDLE;
        default: fft_state <= FFT_IDLE;
      endcase

      if (fft_hold) begin
        // Freeze the whole pipeline; the memories are busy this cycle.
      end else begin
        a_valid <= (fft_state == FFT_RUN);
        a_ia <= ia; a_ib <= ib; a_tw <= tw_index;

        p1_valid <= a_valid;
        p1_ia <= a_ia; p1_ib <= a_ib;
        p1_swap <= parity(a_ia);
        twiddle_q <= twiddle_rom[a_tw];

        ps_valid <= p1_valid;
        ps_ia <= p1_ia; ps_ib <= p1_ib;
        ps_a <= operand_a; ps_b <= operand_b;
        ps_wr <= twiddle_q[0 +: TWID_W];
        ps_wi <= twiddle_q[TWID_W +: TWID_W];

        p2_valid <= ps_valid;
        p2_ia <= ps_ia; p2_ib <= ps_ib;
        p2_wr <= ps_wr; p2_wi <= ps_wi;
        for (lane = 0; lane < LANES; lane = lane + 1) begin
          p2_sum_wide[lane] <= $signed(ps_a[lane*DATA_W +: DATA_W])
                             + $signed(ps_b[lane*DATA_W +: DATA_W]);
          p2_dif_wide[lane] <= $signed(ps_a[lane*DATA_W +: DATA_W])
                             - $signed(ps_b[lane*DATA_W +: DATA_W]);
        end

        pm_valid <= p2_valid;
        pm_ia <= p2_ia; pm_ib <= p2_ib;
        for (lane = 0; lane < LANES; lane = lane + 1)
          pm_sum[lane*DATA_W +: DATA_W] <= clamp(p2_sum_wide[lane]);
        for (slot = 0; slot < 2; slot = slot + 1) begin
          m_a[4*slot + 0] <= clamp(p2_dif_wide[2*slot]);
          m_a[4*slot + 1] <= clamp(p2_dif_wide[2*slot + 1]);
          m_a[4*slot + 2] <= clamp(p2_dif_wide[2*slot]);
          m_a[4*slot + 3] <= clamp(p2_dif_wide[2*slot + 1]);
          m_b[4*slot + 0] <= p2_wr;  m_b[4*slot + 1] <= p2_wi;
          m_b[4*slot + 2] <= p2_wi;  m_b[4*slot + 3] <= p2_wr;
        end

        p3_valid <= pm_valid;
        p3_ia <= pm_ia; p3_ib <= pm_ib;
        p3_sum <= pm_sum;
        for (slot = 0; slot < 2; slot = slot + 1) begin
          // The rounding constant rides along in the DSP's post-adder.
          p3_rr[slot] <= m_a[4*slot + 0] * m_b[4*slot + 0] + TWID_ROUND;
          p3_ii[slot] <= m_a[4*slot + 1] * m_b[4*slot + 1];
          p3_ri[slot] <= m_a[4*slot + 2] * m_b[4*slot + 2] + TWID_ROUND;
          p3_ir[slot] <= m_a[4*slot + 3] * m_b[4*slot + 3];
        end

        p4_valid <= p3_valid;
        p4_ia <= p3_ia; p4_ib <= p3_ib;
        p4_sum <= p3_sum;
        for (slot = 0; slot < 2; slot = slot + 1) begin
          p4_real[slot] <= p3_rr[slot] - p3_ii[slot];
          p4_imag[slot] <= p3_ri[slot] + p3_ir[slot];
        end

        p5_valid <= p4_valid;
        p5_ia <= p4_ia; p5_ib <= p4_ib;
        p5_sum <= p4_sum;
        for (slot = 0; slot < 2; slot = slot + 1) begin
          p5_product[(2*slot)*DATA_W +: DATA_W] <= twiddle_scale(p4_real[slot]);
          p5_product[(2*slot + 1)*DATA_W +: DATA_W] <= twiddle_scale(p4_imag[slot]);
        end
      end
    end
  end

  // ------------------------------------------------------------------
  // Output stage.  Reading a bin, selecting its bank and rounding it to the
  // sample format are three separate cycles, so a four-entry skid and a
  // credit count cover everything in flight.  Beats leave one per cycle
  // while the sink keeps up, and an offered beat is never withdrawn or
  // altered when it does not.
  // ------------------------------------------------------------------
  localparam integer SKID = 8;
  reg [OUT_LANES*WIDTH-1:0] skid [0:SKID-1];
  reg              skid_last [0:SKID-1];
  reg [2:0]        wptr, rptr;
  reg [3:0]        filled;         // beats sitting in the skid
  reg [3:0]        pending;        // beats issued but not yet accepted
  reg [LOGN:0]     out_index;      // counts reads issued, so it reaches n
  reg              rd1, rd1_last, rd2, rd2_last, rd3, rd3_last;
  reg [MEM_W-1:0]  out_sel, out_word;
  reg [OUT_LANES*WIDTH-1:0] landed;

  wire [MEM_W-1:0] out_source = out_read_bank ? bank1_q : bank0_q;
  wire             pop = m_axis_tvalid && m_axis_tready;
  wire             issue = (fft_state == FFT_OUT) && (pending < SKID[3:0])
                           && (out_index != n_count);

  assign m_axis_tvalid = (filled != 4'd0);
  assign m_axis_tdata  = skid[rptr];
  assign m_axis_tlast  = skid_last[rptr];

  always @* begin
    for (lane = 0; lane < LANES; lane = lane + 1)
      landed[lane*WIDTH +: WIDTH] =
        to_sample($signed(out_word[lane*DATA_W +: DATA_W]));
  end

  always @(posedge aclk) begin
    if (!aresetn) begin
      out_index <= 0; out_done <= 1'b0;
      rd1 <= 1'b0; rd2 <= 1'b0; rd3 <= 1'b0;
      wptr <= 3'd0; rptr <= 3'd0; filled <= 4'd0; pending <= 4'd0;
    end else begin
      out_done <= 1'b0;
      if (rd3) begin
        skid[wptr] <= landed;
        skid_last[wptr] <= rd3_last;
        wptr <= wptr + 3'd1;
        if (rd3_last) out_done <= 1'b1;
      end
      // Selecting the bank already costs a block RAM's clock-to-output, so
      // the rounding addition waits for its own cycle.
      rd3 <= rd2;
      rd3_last <= rd2_last;
      if (rd2)
        for (lane = 0; lane < LANES; lane = lane + 1)
          out_word[lane*DATA_W +: DATA_W] <=
            $signed(out_sel[lane*DATA_W +: DATA_W]) + SAMP_ROUND;
      rd2 <= rd1;
      rd2_last <= rd1_last;
      if (rd1) out_sel <= out_source;
      rd1 <= issue;
      if (issue) begin
        out_read_bank <= parity(out_index[LOGN-1:0]);
        rd1_last <= (out_index == n_last);
        out_index <= out_index + 1'b1;
      end
      if (pop) rptr <= rptr + 3'd1;
      filled <= filled + (rd3 ? 4'd1 : 4'd0) - (pop ? 4'd1 : 4'd0);
      pending <= pending + (issue ? 4'd1 : 4'd0) - (pop ? 4'd1 : 4'd0);
      if (fft_state != FFT_OUT) out_index <= 0;
    end
  end

  // ------------------------------------------------------------------
  // Memory port arbitration.  Deciding which bank takes the butterfly's sum
  // and which takes its product, and arbitrating that against the FIR, both
  // happen a cycle ahead, so the block RAMs see address and data straight
  // from registers.  Resolving it in front of the memory instead put a
  // multiplexer plus its routing between a register and the RAM's data pins,
  // which was the whole of the remaining timing margin.
  // ------------------------------------------------------------------
  wire fir_bank = parity(fir_result_index);

  always @(posedge aclk) begin
    if (!aresetn) begin
      bank0_we <= 1'b0;
      bank1_we <= 1'b0;
    end else begin
      bank0_we <= 1'b0;
      bank1_we <= 1'b0;
      if (fir_result_valid) begin
        // The FIR always wins: it cannot stall without dropping a sample.
        bank0_we <= !fir_bank;
        bank1_we <= fir_bank;
        bank0_waddr <= {fir_result_buf, fir_result_index[ADDR_W-1:0]};
        bank1_waddr <= {fir_result_buf, fir_result_index[ADDR_W-1:0]};
        bank0_wdata <= fir_result;
        bank1_wdata <= fir_result;
      end else if (p5_valid) begin
        bank0_we <= 1'b1;
        bank1_we <= 1'b1;
        if (parity(p5_ia)) begin
          bank1_waddr <= {fft_buf, p5_ia[ADDR_W-1:0]}; bank1_wdata <= p5_sum;
          bank0_waddr <= {fft_buf, p5_ib[ADDR_W-1:0]}; bank0_wdata <= p5_product;
        end else begin
          bank0_waddr <= {fft_buf, p5_ia[ADDR_W-1:0]}; bank0_wdata <= p5_sum;
          bank1_waddr <= {fft_buf, p5_ib[ADDR_W-1:0]}; bank1_wdata <= p5_product;
        end
      end
    end
  end

  always @* begin
    if (fft_state == FFT_OUT) begin
      bank0_raddr = {fft_buf, out_index[ADDR_W-1:0]};
      bank1_raddr = {fft_buf, out_index[ADDR_W-1:0]};
    end else if (parity(a_ia)) begin
      bank1_raddr = {fft_buf, a_ia[ADDR_W-1:0]};
      bank0_raddr = {fft_buf, a_ib[ADDR_W-1:0]};
    end else begin
      bank0_raddr = {fft_buf, a_ia[ADDR_W-1:0]};
      bank1_raddr = {fft_buf, a_ib[ADDR_W-1:0]};
    end
  end
endmodule
