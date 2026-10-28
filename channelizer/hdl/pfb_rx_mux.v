// What the RX DMA gets, in the radio clock domain: the selected channels, or
// with bypass set the two receivers as the radio delivers them. Raw samples
// are shifted up 3 bits to the channels' Q18 scale (12-bit ADC codes, sign
// extended, still fit 16 bits), so the host reads both the same way.
module pfb_rx_mux (
    input clk, input bypass,
    input rx0_valid, input [15:0] rx0_i, input [15:0] rx0_q,
    input rx1_valid, input [15:0] rx1_i, input [15:0] rx1_q,
    input pfb_valid, input [63:0] pfb_data,   // {rx1 q, rx1 i, rx0 q, rx0 i}
    output reg m_valid = 1'b0, output reg [63:0] m_data
);
    always @(posedge clk) begin
        m_valid <= bypass ? rx0_valid && rx1_valid : pfb_valid;
        m_data <= bypass ? {rx1_q[12:0], 3'd0, rx1_i[12:0], 3'd0, rx0_q[12:0], 3'd0, rx0_i[12:0], 3'd0}
                       : pfb_data;
    end
endmodule
