// Capture both coherent AD9361 receivers as one word before crossing into
// the 200 MHz PFB clock domain. The radio cannot be backpressured; a full
// FIFO discards the sample intact and pulses drop.
module pfb_dual_ingress (
    input rst, input rx_clk,
    input rx0_valid, input signed [15:0] rx0_i, input signed [15:0] rx0_q,
    input rx1_valid, input signed [15:0] rx1_i, input signed [15:0] rx1_q,
    input fabric_clk, input rd_en,
    output [63:0] rd_data, output empty, output reg drop
);
    wire full;
    wire write_sample = rx0_valid && rx1_valid;

    pfb_async_fifo64 fifo (
        .rst(rst), .wr_clk(rx_clk),
        .wr_data({rx1_q, rx1_i, rx0_q, rx0_i}), .wr_en(write_sample), .full(full),
        .rd_clk(fabric_clk), .rd_data(rd_data), .rd_en(rd_en), .empty(empty)
    );

    always @(posedge rx_clk) drop <= !rst && write_sample && full;
endmodule
