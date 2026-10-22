// The PFB core is DIF: its output beats are bit-reversed. Translate the beat
// counter back to natural FFT-bin numbering, then to the fftshift channel
// (natural + n/2 mod n, so channel n/2 is the LO), and keep only the beats
// whose channel is set in the mask. A new mask takes effect at a frame
// boundary. logn changes only while rst is held.
module pfb_dual_bin_selector #(
    parameter integer N_MAX = 256,
    parameter integer LOGN_MAX = 8
) (
    input clk, input rst, input [3:0] logn, input [N_MAX-1:0] channel_mask,
    input s_valid, input s_last, input [63:0] s_data,
    output reg m_valid, output reg [63:0] m_data
);
    reg [LOGN_MAX-1:0] output_index;
    reg [N_MAX-1:0] active_mask;
    reg [LOGN_MAX-1:0] half;            // n/2: the top channel bit to flip
    wire [LOGN_MAX-1:0] reversed;
    genvar b;
    generate
        for (b = 0; b < LOGN_MAX; b = b + 1) begin : g_rev
            assign reversed[b] = output_index[LOGN_MAX - 1 - b];
        end
    endgenerate
    // The beat's bin is its logn-bit reversal, which sits at the top of the
    // LOGN_MAX-bit reversal.
    wire [LOGN_MAX-1:0] natural_bin = reversed >> (LOGN_MAX - logn);

    // Stage a finds the channel, stage b looks it up: a 256-way mask
    // multiplexer after the shift does not fit one 200 MHz cycle.
    reg a_valid, a_last;
    reg [LOGN_MAX-1:0] a_channel;
    reg [63:0] a_data;
    reg b_valid, b_select;
    reg [63:0] b_data;

    always @(posedge clk) begin
        if (rst) begin
            output_index <= 0;
            active_mask <= channel_mask;
            half <= 1 << (logn - 1);
            a_valid <= 1'b0; b_valid <= 1'b0; m_valid <= 1'b0;
        end else begin
            a_valid <= s_valid;
            if (s_valid) begin
                a_channel <= natural_bin ^ half;
                a_last <= s_last;
                a_data <= s_data;
                output_index <= s_last ? 0 : output_index + 1'b1;
            end
            // The frame's last beat is judged by the old mask, then the
            // new one is taken.
            b_valid <= a_valid;
            if (a_valid) begin
                b_select <= active_mask[a_channel];
                b_data <= a_data;
                if (a_last) active_mask <= channel_mask;
            end
            m_valid <= b_valid && b_select;
            m_data <= b_data;
        end
    end
endmodule
