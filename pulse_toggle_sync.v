module pulse_toggle_sync (
    input  wire src_clk,
    input  wire src_rst_n,
    input  wire src_pulse,
    input  wire dst_clk,
    input  wire dst_rst_n,
    output reg  dst_pulse
);

reg toggle_src;
always @(posedge src_clk or negedge src_rst_n) begin
    if (!src_rst_n) toggle_src <= 1'b0;
    else if (src_pulse) toggle_src <= ~toggle_src;
end

reg sync0, sync1, sync0_d;
always @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) begin
        sync0 <= 1'b0; sync1 <= 1'b0; sync0_d <= 1'b0; dst_pulse <= 1'b0;
    end else begin
        sync0 <= toggle_src;
        sync1 <= sync0;
        dst_pulse <= sync1 ^ sync0_d;
        sync0_d <= sync1;
    end
end

endmodule