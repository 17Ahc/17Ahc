module seg_control(
    input                    app_rx_data_valid,
    input        [71:0]      app_rx_data_buffer, // newest byte at [71:64]
    input                    udp_rx_clk,
    input                    reset,
    output reg  [15:0]       dled
);

// 8 display nibbles
reg [3:0] seg_digits [7:0];

// scan/multiplex driver (唯一驱动 dled)
reg [2:0] scan_cnt;
reg [19:0] scan_timer;

// local/tmp used inside clocked always with blocking assigns
reg [71:0] tmp_win;
reg [71:0] rotated_local;
reg [3:0]  idx_local;

// seven-seg encodings
parameter [7:0] SEG_0 = 8'b1100_0000;
parameter [7:0] SEG_1 = 8'b1111_1001;
parameter [7:0] SEG_2 = 8'b1010_0100;
parameter [7:0] SEG_3 = 8'b1011_0000;
parameter [7:0] SEG_4 = 8'b1001_1001;
parameter [7:0] SEG_5 = 8'b1001_0010;
parameter [7:0] SEG_6 = 8'b1000_0010;
parameter [7:0] SEG_7 = 8'b1111_1000;
parameter [7:0] SEG_8 = 8'b1000_0000;
parameter [7:0] SEG_9 = 8'b1001_0000;
parameter [7:0] SEG_A = 8'b1000_1000;
parameter [7:0] SEG_B = 8'b1000_0011;
parameter [7:0] SEG_C = 8'b1100_0110;
parameter [7:0] SEG_D = 8'b1010_0001;
parameter [7:0] SEG_E = 8'b1000_0110;
parameter [7:0] SEG_F = 8'b1000_1110;
parameter [7:0] SEG_OFF = 8'b1111_1111;

function [7:0] digit_to_seg;
    input [3:0] digit;
    begin
        case (digit)
            4'h0: digit_to_seg = SEG_0;
            4'h1: digit_to_seg = SEG_1;
            4'h2: digit_to_seg = SEG_2;
            4'h3: digit_to_seg = SEG_3;
            4'h4: digit_to_seg = SEG_4;
            4'h5: digit_to_seg = SEG_5;
            4'h6: digit_to_seg = SEG_6;
            4'h7: digit_to_seg = SEG_7;
            4'h8: digit_to_seg = SEG_8;
            4'h9: digit_to_seg = SEG_9;
            4'hA: digit_to_seg = SEG_A;
            4'hB: digit_to_seg = SEG_B;
            4'hC: digit_to_seg = SEG_C;
            4'hD: digit_to_seg = SEG_D;
            4'hE: digit_to_seg = SEG_E;
            4'hF: digit_to_seg = SEG_F;
            default: digit_to_seg = SEG_OFF;
        endcase
    end
endfunction

// Core: on a valid byte, atomically sample window (tmp_win) and compute rotation+index
// using blocking assignments so computation uses the freshly-sampled window in same cycle.
always @(posedge udp_rx_clk or negedge reset) begin
    if (!reset) begin
        seg_digits[0] <= 4'h0;
        seg_digits[1] <= 4'h0;
        seg_digits[2] <= 4'h0;
        seg_digits[3] <= 4'h0;
        seg_digits[4] <= 4'h0;
        seg_digits[5] <= 4'h0;
        seg_digits[6] <= 4'h0;
        seg_digits[7] <= 4'h0;
        tmp_win <= 72'h0;
        rotated_local <= 72'h0;
        idx_local <= 4'd9;
    end else begin
        if (app_rx_data_valid) begin
            // blocking sample of current window so following logic uses latest bytes this cycle
            tmp_win = app_rx_data_buffer; // blocking assign

            // find first header 0xAA (MSB->LSB)
            if (tmp_win[71:64] == 8'hAA) idx_local = 4'd0;
            else if (tmp_win[63:56] == 8'hAA) idx_local = 4'd1;
            else if (tmp_win[55:48] == 8'hAA) idx_local = 4'd2;
            else if (tmp_win[47:40] == 8'hAA) idx_local = 4'd3;
            else if (tmp_win[39:32] == 8'hAA) idx_local = 4'd4;
            else if (tmp_win[31:24] == 8'hAA) idx_local = 4'd5;
            else if (tmp_win[23:16] == 8'hAA) idx_local = 4'd6;
            else if (tmp_win[15:8]  == 8'hAA) idx_local = 4'd7;
            else if (tmp_win[7:0]   == 8'hAA) idx_local = 4'd8;
            else idx_local = 4'd9;

            // rotate locally based on idx_local (blocking assignments)
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

            // commit update to displayed digits if header found
            if (idx_local != 4'd9) begin
                seg_digits[0] <= rotated_local[63:60];
                seg_digits[1] <= rotated_local[55:52];
                seg_digits[2] <= rotated_local[47:44];
                seg_digits[3] <= rotated_local[39:36];
                seg_digits[4] <= rotated_local[31:28];
                seg_digits[5] <= rotated_local[23:20];
                seg_digits[6] <= rotated_local[15:12];
                seg_digits[7] <= rotated_local[7:4];
            end
            // else keep previous
        end
    end
end

// scanning / multiplexing (唯一驱动 dled)
always @(posedge udp_rx_clk or negedge reset) begin
    if (!reset) begin
        scan_cnt <= 3'd0;
        scan_timer <= 20'd0;
        dled <= 16'h0000;
    end else begin
        if (scan_timer >= 20'd50000) begin
            scan_timer <= 20'd0;
            scan_cnt <= scan_cnt + 1'b1;
            if (scan_cnt == 3'd7) scan_cnt <= 3'd0;
        end else begin
            scan_timer <= scan_timer + 1'b1;
        end

        case (scan_cnt)
            3'd0: dled <= {8'b11111110, digit_to_seg(seg_digits[0])};
            3'd1: dled <= {8'b11111101, digit_to_seg(seg_digits[1])};
            3'd2: dled <= {8'b11111011, digit_to_seg(seg_digits[2])};
            3'd3: dled <= {8'b11110111, digit_to_seg(seg_digits[3])};
            3'd4: dled <= {8'b11101111, digit_to_seg(seg_digits[4])};
            3'd5: dled <= {8'b11011111, digit_to_seg(seg_digits[5])};
            3'd6: dled <= {8'b10111111, digit_to_seg(seg_digits[6])};
            3'd7: dled <= {8'b01111111, digit_to_seg(seg_digits[7])};
            default: dled <= 16'h0000;
        endcase
    end
end

endmodule