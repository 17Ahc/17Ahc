module led(
    input                    app_rx_data_valid,
    input        [71:0]      app_rx_data_buffer, // newest byte at [71:64]
    input                    udp_rx_clk,
    input                    reset,
    output       [3:0]       led_data_1,
    output       [15:0]      dled
);

// local tmp for atomic processing
reg [71:0] tmp_win;
reg [71:0] rotated_local;
reg [3:0]  idx_local;
reg [63:0] led_data_reg;

always @(posedge udp_rx_clk or negedge reset) begin
    if (!reset) begin
        tmp_win <= 72'h0;
        rotated_local <= 72'h0;
        idx_local <= 4'd9;
        led_data_reg <= 64'h0;
    end else begin
        if (app_rx_data_valid) begin
            // blocking sample
            tmp_win = app_rx_data_buffer;

            // find header 0x55
            if (tmp_win[71:64] == 8'h55) idx_local = 4'd0;
            else if (tmp_win[63:56] == 8'h55) idx_local = 4'd1;
            else if (tmp_win[55:48] == 8'h55) idx_local = 4'd2;
            else if (tmp_win[47:40] == 8'h55) idx_local = 4'd3;
            else if (tmp_win[39:32] == 8'h55) idx_local = 4'd4;
            else if (tmp_win[31:24] == 8'h55) idx_local = 4'd5;
            else if (tmp_win[23:16] == 8'h55) idx_local = 4'd6;
            else if (tmp_win[15:8]  == 8'h55) idx_local = 4'd7;
            else if (tmp_win[7:0]   == 8'h55) idx_local = 4'd8;
            else idx_local = 4'd9;

            // rotate
            case (idx_local)
                4'd0: rotated_local = { tmp_win[71:64], tmp_win[63:56], tmp_win[55:48],
                                        tmp_win[47:40], tmp_win[39:32], tmp_win[31:24],
                                        tmp_win[23:16], tmp_win[15:8],  tmp_win[7:0] };
                4'd1: rotated_local = { tmp_win[63:56], tmp_win[55:48], tmp_win[47:40],
                                        tmp_win[39:32], tmp_win[31:24], tmp_win[23:16],
                                        tmp_win[15:8],  tmp_win[7:0],   tmp_win[71:64] };
                4'd2: rotated_local = { tmp_win[55:48], tmp_win[47:40], tmp_win[39:32],
                                        tmp_win[31:24], tmp_win[23:16], tmp_win[15:8],
                                        tmp_win[7:0],   tmp_win[71:64], tmp_win[63:56] };
                4'd3: rotated_local = { tmp_win[47:40], tmp_win[39:32], tmp_win[31:24],
                                        tmp_win[23:16], tmp_win[15:8],  tmp_win[7:0],
                                        tmp_win[71:64], tmp_win[63:56], tmp_win[55:48] };
                4'd4: rotated_local = { tmp_win[39:32], tmp_win[31:24], tmp_win[23:16],
                                        tmp_win[15:8],  tmp_win[7:0],   tmp_win[71:64],
                                        tmp_win[63:56], tmp_win[55:48], tmp_win[47:40] };
                4'd5: rotated_local = { tmp_win[31:24], tmp_win[23:16], tmp_win[15:8],
                                        tmp_win[7:0],   tmp_win[71:64], tmp_win[63:56],
                                        tmp_win[55:48], tmp_win[47:40], tmp_win[39:32] };
                4'd6: rotated_local = { tmp_win[23:16], tmp_win[15:8],  tmp_win[7:0],
                                        tmp_win[71:64], tmp_win[63:56], tmp_win[55:48],
                                        tmp_win[47:40], tmp_win[39:32], tmp_win[31:24] };
                4'd7: rotated_local = { tmp_win[15:8],  tmp_win[7:0],   tmp_win[71:64],
                                        tmp_win[63:56], tmp_win[55:48], tmp_win[47:40],
                                        tmp_win[39:32], tmp_win[31:24], tmp_win[23:16] };
                4'd8: rotated_local = { tmp_win[7:0],   tmp_win[71:64], tmp_win[63:56],
                                        tmp_win[55:48], tmp_win[47:40], tmp_win[39:32],
                                        tmp_win[31:24], tmp_win[23:16], tmp_win[15:8] };
                default: rotated_local = rotated_local;
            endcase

            if (idx_local != 4'd9) begin
                led_data_reg <= {
                    rotated_local[63:56],
                    rotated_local[55:48],
                    rotated_local[47:40],
                    rotated_local[39:32],
                    rotated_local[31:24],
                    rotated_local[23:16],
                    rotated_local[15:8],
                    rotated_local[7:0]
                };
            end
        end
    end
end

assign led_data_1 = led_data_reg[63:60];
assign dled = led_data_reg[55:40];

endmodule