// Behavioural stand-ins for the XPM CDC macros used in hdl/, for iverilog.
// Same ports and handshake behaviour, without the metastability modelling.
module xpm_cdc_sync_rst #(parameter integer DEST_SYNC_FF = 4, parameter integer INIT = 1,
                          parameter integer INIT_SYNC_FF = 0, parameter integer SIM_ASSERT_CHK = 0) (
  input src_rst, input dest_clk, output dest_rst);
  reg [DEST_SYNC_FF-1:0] s = {DEST_SYNC_FF{INIT[0]}};
  always @(posedge dest_clk) s <= {s[DEST_SYNC_FF-2:0], src_rst};
  assign dest_rst = s[DEST_SYNC_FF-1];
endmodule

module xpm_cdc_array_single #(parameter integer DEST_SYNC_FF = 4, parameter integer INIT_SYNC_FF = 0,
                              parameter integer SIM_ASSERT_CHK = 0, parameter integer SRC_INPUT_REG = 1,
                              parameter integer WIDTH = 2) (
  input src_clk, input [WIDTH-1:0] src_in, input dest_clk, output [WIDTH-1:0] dest_out);
  reg [WIDTH-1:0] r = 0;
  reg [WIDTH-1:0] s [0:DEST_SYNC_FF-1];
  integer k;
  initial for (k = 0; k < DEST_SYNC_FF; k = k + 1) s[k] = 0;
  always @(posedge src_clk) r <= src_in;
  wire [WIDTH-1:0] src = SRC_INPUT_REG ? r : src_in;
  always @(posedge dest_clk) begin
    s[0] <= src;
    for (k = 1; k < DEST_SYNC_FF; k = k + 1) s[k] <= s[k-1];
  end
  assign dest_out = s[DEST_SYNC_FF-1];
endmodule

module xpm_cdc_handshake #(parameter integer DEST_EXT_HSK = 0, parameter integer DEST_SYNC_FF = 4,
                           parameter integer INIT_SYNC_FF = 0, parameter integer SIM_ASSERT_CHK = 0,
                           parameter integer SRC_SYNC_FF = 4, parameter integer WIDTH = 1) (
  input src_clk, input [WIDTH-1:0] src_in, input src_send, output src_rcv,
  input dest_clk, output reg dest_req = 1'b0, input dest_ack, output reg [WIDTH-1:0] dest_out);
  reg [WIDTH-1:0] held;
  reg send_q = 1'b0;
  always @(posedge src_clk) begin
    send_q <= src_send;
    if (src_send && !send_q) held <= src_in;
  end
  reg [DEST_SYNC_FF-1:0] req_s = 0;
  always @(posedge dest_clk) begin
    req_s <= {req_s[DEST_SYNC_FF-2:0], send_q};
    dest_req <= 1'b0;
    if (req_s[DEST_SYNC_FF-2] && !req_s[DEST_SYNC_FF-1]) begin
      dest_out <= held;
      dest_req <= 1'b1;
    end
  end
  reg [SRC_SYNC_FF-1:0] ack_s = 0;
  always @(posedge src_clk) ack_s <= {ack_s[SRC_SYNC_FF-2:0], req_s[DEST_SYNC_FF-1]};
  assign src_rcv = ack_s[SRC_SYNC_FF-1];
endmodule
