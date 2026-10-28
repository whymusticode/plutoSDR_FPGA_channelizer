// Channelizer control registers (AXI4-Lite, CPU clock), replacing the mask GPIO.
//
//   0x00 ID       RO  0x50464232 ("PFB2")
//   0x04 BUILD    RO  [7:0] log2 N_MAX, [15:8] taps per phase, [23:16] coef bits,
//                     [24] CONTROL[1] (bypass) exists
//   0x08 CONTROL  RW  [0] hold the channelizer in reset, [1] bypass: the DMA
//                     gets the raw receivers instead of the selected channels,
//                     [11:8] log2 channels
//   0x0C STATUS   RW1C [0] input dropped samples, [1] output dropped samples;
//                 RO [8] held in reset
//   0x10 COEF_IDX RW  phase * TAPS + tap of the next COEF_DATA write
//   0x14 COEF_DATA WO 18-bit tap (Q(16 + log2 channels)); advances COEF_IDX
//   0x18 COMMIT   WO  any write moves MASK into the channelizer at once
//   0x40 + 4k MASK RW channels 32k..32k+31, fftshift numbering (n/2 = centre)
//
// Configuring a size: CONTROL = hold | logn << 8; write the taps for that
// size; CONTROL = logn << 8. The channelizer takes logn as it leaves reset.
// The mask can change at any time; COMMIT applies all words together and the
// selector switches at the next frame boundary. Tap writes and commits each
// cross into the channelizer clock through one handshake, and the write
// response waits for it, so back-to-back writes cannot overrun it.
module pfb_ctrl #(
  parameter integer N_MAX = 256,
  parameter integer LOGN_MAX = 8,
  parameter integer TAPS = 8,
  parameter integer TAP_W = 3,
  parameter integer BOOT_LOGN = 6
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  input  wire        s_axi_aclk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  wire        s_axi_aresetn,
  input  wire [11:0] s_axi_awaddr,
  input  wire [2:0]  s_axi_awprot,
  input  wire        s_axi_awvalid,
  output reg         s_axi_awready,
  input  wire [31:0] s_axi_wdata,
  input  wire [3:0]  s_axi_wstrb,
  input  wire        s_axi_wvalid,
  output reg         s_axi_wready,
  output wire [1:0]  s_axi_bresp,
  output reg         s_axi_bvalid,
  input  wire        s_axi_bready,
  input  wire [11:0] s_axi_araddr,
  input  wire [2:0]  s_axi_arprot,
  input  wire        s_axi_arvalid,
  output reg         s_axi_arready,
  output reg  [31:0] s_axi_rdata,
  output wire [1:0]  s_axi_rresp,
  output reg         s_axi_rvalid,
  input  wire        s_axi_rready,

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 pfb_clk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_RESET pfb_reset:pfb_resetn" *)
  input  wire        pfb_clk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 pfb_resetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  output wire        pfb_resetn,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 pfb_reset RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_HIGH" *)
  output wire        pfb_reset,
  output wire [3:0]  pfb_logn,
  output wire        coef_we,
  output wire [LOGN_MAX-1:0] coef_wphase,
  output wire [TAP_W-1:0]    coef_wtap,
  output wire [17:0] coef_wdata,
  output reg  [N_MAX-1:0] channel_mask,
  input  wire        in_clk,
  input  wire        in_drop,      // in_clk: the ingress discarded a sample
  input  wire        out_drop,     // pfb_clk: the output FIFO discarded a beat
  output wire        rx_bypass     // in_clk: CONTROL[1]
);
  localparam [31:0] ID = 32'h50464232;
  localparam integer MASK_WORDS = N_MAX / 32;
  localparam integer IDX_W = LOGN_MAX + TAP_W;
  localparam [7:0] BUILD_LOGN = LOGN_MAX, BUILD_TAPS = TAPS, BUILD_COEF_W = 18;
  localparam [7:0] BUILD_FEATURES = 8'h01;   // [0] bypass

  assign s_axi_bresp = 2'b00;
  assign s_axi_rresp = 2'b00;

  reg                 hold, bypass;
  reg  [3:0]          logn;
  reg  [IDX_W-1:0]    coef_idx;
  reg  [17:0]         coef_val;
  reg  [N_MAX-1:0]    mask_shadow;
  reg  [1:0]          dropped;

  // ---- write channel: one transaction at a time ---------------------------
  localparam [2:0] W_IDLE = 3'd0, W_DO = 3'd1, W_HI = 3'd2, W_LO = 3'd3, W_RESP = 3'd4;
  reg [2:0]  wstate;
  reg [11:0] waddr;
  reg [31:0] wdata;
  reg        sending_coef, coef_send, mask_send;
  wire       coef_rcv, mask_rcv;
  wire       rcv = sending_coef ? coef_rcv : mask_rcv;
  wire [1:0] drop_seen;
  integer    k;

  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      wstate <= W_IDLE;
      s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_bvalid <= 1'b0;
      hold <= 1'b0; bypass <= 1'b0; logn <= BOOT_LOGN;
      coef_idx <= 0; coef_send <= 1'b0; mask_send <= 1'b0; sending_coef <= 1'b0;
      mask_shadow <= 0; dropped <= 2'b00;
    end else begin
      s_axi_awready <= 1'b0;
      s_axi_wready <= 1'b0;
      dropped <= dropped | drop_seen;
      case (wstate)
        W_IDLE:
          if (s_axi_awvalid && s_axi_wvalid) begin
            s_axi_awready <= 1'b1; s_axi_wready <= 1'b1;
            waddr <= s_axi_awaddr; wdata <= s_axi_wdata;
            wstate <= W_DO;
          end
        W_DO: begin
          wstate <= W_RESP;
          case (waddr[11:2])
            10'h02: begin hold <= wdata[0]; bypass <= wdata[1]; logn <= wdata[11:8]; end
            10'h03: dropped <= (dropped | drop_seen) & ~wdata[1:0];
            10'h04: coef_idx <= wdata[IDX_W-1:0];
            10'h05: begin
              coef_val <= wdata[17:0];
              sending_coef <= 1'b1; coef_send <= 1'b1; wstate <= W_HI;
            end
            10'h06: begin sending_coef <= 1'b0; mask_send <= 1'b1; wstate <= W_HI; end
            default:
              for (k = 0; k < MASK_WORDS; k = k + 1)
                if (waddr[11:2] == 10'h10 + k) mask_shadow[32*k +: 32] <= wdata;
          endcase
        end
        W_HI:
          if (rcv) begin coef_send <= 1'b0; mask_send <= 1'b0; wstate <= W_LO; end
        W_LO:
          if (!rcv) begin
            if (sending_coef) coef_idx <= coef_idx + 1'b1;
            wstate <= W_RESP;
          end
        W_RESP: begin
          s_axi_bvalid <= 1'b1;
          if (s_axi_bvalid && s_axi_bready) begin
            s_axi_bvalid <= 1'b0;
            wstate <= W_IDLE;
          end
        end
        default: wstate <= W_IDLE;
      endcase
    end
  end

  // ---- read channel -------------------------------------------------------
  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0;
    end else begin
      s_axi_arready <= 1'b0;
      if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
      else if (s_axi_arvalid && !s_axi_rvalid && !s_axi_arready) begin
        s_axi_arready <= 1'b1;
        s_axi_rvalid <= 1'b1;
        case (s_axi_araddr[11:2])
          10'h00: s_axi_rdata <= ID;
          10'h01: s_axi_rdata <= {BUILD_FEATURES, BUILD_COEF_W, BUILD_TAPS, BUILD_LOGN};
          10'h02: s_axi_rdata <= {20'd0, logn, 6'd0, bypass, hold};
          10'h03: s_axi_rdata <= {23'd0, hold, 6'd0, dropped};
          10'h04: s_axi_rdata <= coef_idx;
          default: begin
            s_axi_rdata <= 32'd0;
            for (k = 0; k < MASK_WORDS; k = k + 1)
              if (s_axi_araddr[11:2] == 10'h10 + k) s_axi_rdata <= mask_shadow[32*k +: 32];
          end
        endcase
      end
    end
  end

  // ---- into the channelizer clock -----------------------------------------
  // Held in reset from configuration until the CPU domain is out of reset.
  // logn only changes while reset is held, so a plain synchronizer is enough.
  reg  hold_src = 1'b1;
  wire hold_sync;
  always @(posedge s_axi_aclk) hold_src <= hold || !s_axi_aresetn;
  xpm_cdc_sync_rst #(.DEST_SYNC_FF(4), .INIT(1)) hold_cdc (
    .src_rst(hold_src), .dest_clk(pfb_clk), .dest_rst(hold_sync));
  assign pfb_reset = hold_sync;
  assign pfb_resetn = !hold_sync;

  xpm_cdc_array_single #(.DEST_SYNC_FF(4), .SRC_INPUT_REG(1), .WIDTH(4)) logn_cdc (
    .src_clk(s_axi_aclk), .src_in(logn), .dest_clk(pfb_clk), .dest_out(pfb_logn));

  xpm_cdc_array_single #(.DEST_SYNC_FF(4), .SRC_INPUT_REG(1), .WIDTH(1)) bypass_cdc (
    .src_clk(s_axi_aclk), .src_in(bypass), .dest_clk(in_clk), .dest_out(rx_bypass));

  wire [IDX_W+17:0] coef_bus;
  xpm_cdc_handshake #(.DEST_EXT_HSK(0), .DEST_SYNC_FF(4), .SRC_SYNC_FF(4),
                      .WIDTH(IDX_W + 18)) coef_cdc (
    .src_clk(s_axi_aclk), .src_in({coef_idx, coef_val}), .src_send(coef_send),
    .src_rcv(coef_rcv), .dest_clk(pfb_clk), .dest_req(coef_we), .dest_ack(1'b0),
    .dest_out(coef_bus));
  assign coef_wphase = coef_bus[18 + TAP_W +: LOGN_MAX];
  assign coef_wtap = coef_bus[18 +: TAP_W];
  assign coef_wdata = coef_bus[17:0];

  wire [N_MAX-1:0] mask_bus;
  wire mask_req;
  xpm_cdc_handshake #(.DEST_EXT_HSK(0), .DEST_SYNC_FF(4), .SRC_SYNC_FF(4),
                      .WIDTH(N_MAX)) mask_cdc (
    .src_clk(s_axi_aclk), .src_in(mask_shadow), .src_send(mask_send),
    .src_rcv(mask_rcv), .dest_clk(pfb_clk), .dest_req(mask_req), .dest_ack(1'b0),
    .dest_out(mask_bus));
  always @(posedge pfb_clk) if (mask_req) channel_mask <= mask_bus;
  initial channel_mask = 0;

  // ---- dropped-sample flags back to the CPU -------------------------------
  // Each source flips a toggle per event; an edge on the synchronized toggle
  // sets the sticky bit.
  reg in_toggle = 1'b0, out_toggle = 1'b0;
  always @(posedge in_clk) if (in_drop) in_toggle <= ~in_toggle;
  always @(posedge pfb_clk) if (out_drop) out_toggle <= ~out_toggle;
  wire [1:0] toggles;
  reg  [1:0] toggles_d = 2'b00;
  xpm_cdc_array_single #(.DEST_SYNC_FF(4), .SRC_INPUT_REG(0), .WIDTH(1)) in_drop_cdc (
    .src_clk(in_clk), .src_in(in_toggle), .dest_clk(s_axi_aclk), .dest_out(toggles[0]));
  xpm_cdc_array_single #(.DEST_SYNC_FF(4), .SRC_INPUT_REG(0), .WIDTH(1)) out_drop_cdc (
    .src_clk(pfb_clk), .src_in(out_toggle), .dest_clk(s_axi_aclk), .dest_out(toggles[1]));
  always @(posedge s_axi_aclk) toggles_d <= toggles;
  assign drop_seen = toggles ^ toggles_d;
endmodule
