module pfb_async_fifo64 (
    input rst,
    input wr_clk, input [63:0] wr_data, input wr_en, output full,
    input rd_clk, output [63:0] rd_data, input rd_en, output empty
);
    wire valid, overflow, underflow, prog_empty, prog_full, raw_full;
    wire safe_wr_en = wr_en && !raw_full;
    assign full = raw_full;
    xpm_fifo_async #(
        .CDC_SYNC_STAGES(2), .DOUT_RESET_VALUE("0"),
        .ECC_MODE("no_ecc"), .FIFO_MEMORY_TYPE("block"),
        .FIFO_READ_LATENCY(0), .FIFO_WRITE_DEPTH(512),
        .FULL_RESET_VALUE(0), .PROG_EMPTY_THRESH(10),
        .PROG_FULL_THRESH(502), .RD_DATA_COUNT_WIDTH(9),
        .READ_DATA_WIDTH(64), .READ_MODE("fwft"),
        .RELATED_CLOCKS(0), .SIM_ASSERT_CHK(0),
        .USE_ADV_FEATURES("0707"), .WAKEUP_TIME(0),
        .WRITE_DATA_WIDTH(64), .WR_DATA_COUNT_WIDTH(9)
    ) fifo (
        .rst(rst), .wr_clk(wr_clk), .wr_en(safe_wr_en), .din(wr_data),
        .full(raw_full), .overflow(overflow), .wr_ack(), .wr_data_count(),
        .prog_full(prog_full), .almost_full(),
        .rd_clk(rd_clk), .rd_en(rd_en), .dout(rd_data), .empty(empty),
        .underflow(underflow), .data_valid(valid), .rd_data_count(),
        .prog_empty(prog_empty), .almost_empty(),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0),
        .sbiterr(), .dbiterr()
    );
endmodule
