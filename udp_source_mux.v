`timescale 1ns / 1ps
module udp_source_mux(
    input  wire         clk,
    input  wire         reset_n,
    input               mem_clk,
    input  wire         sel_cam,
    input               Sdr_init_done,
    // 摄像头：8-bit bytes
    input  wire [7:0]   cam_data,
    input  wire         cam_data_valid,
    input  wire [15:0]  cam_data_length, // 字节长度
    input  wire         cam_data_done,    

    // SD/TF：假定 24-bit 单位数据（如原工程）
    input  wire [23:0]  sd_data,
    input  wire         sd_data_valid,
    input  wire [15:0]  sd_data_length,  // 以 24-bit word 为单位或以字节（请按实际单位调整）
    input  wire         sd_data_done,

    // 统一输出到 udp 发送模块（24-bit data）
    output reg  [23:0]  app_tx_data,
    output reg          app_tx_data_valid,
    output reg  [15:0]  app_tx_data_length, // 输出以 24-bit words 计数
    output reg          app_tx_data_done
);
// 内部：用于摄像头字节打包
reg [1:0]  cam_byte_cnt;
reg [23:0] cam_pack_buf;
reg [15:0] cam_word_count; // 24-bit word count for camera frames
reg [15:0] cam_len_bytes;  // remaining bytes to process
reg        sd_valid_prev;
reg [15:0] sd_word_cnt;
reg        sd_end_req;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        app_tx_data        <= 24'd0;
        app_tx_data_valid  <= 1'b0;
        app_tx_data_length <= 16'd0;
        app_tx_data_done   <= 1'b0;

        cam_byte_cnt       <= 2'd0;
        cam_pack_buf       <= 24'd0;
        cam_word_count     <= 16'd0;

        sd_word_cnt        <= 16'd0;
        sd_valid_prev      <= 1'b0;
    end else begin
        app_tx_data_valid <= 1'b0;
        app_tx_data_done  <= 1'b0;

        if (sel_cam) begin
            if (cam_data_valid) begin
                // shift existing buffer and append new byte at LSB side
                cam_pack_buf <= {cam_pack_buf[15:0], cam_data};
                cam_byte_cnt <= cam_byte_cnt + 1'b1;
                if (cam_byte_cnt == 2) begin
                    // have 3 bytes now -> output a 24-bit word (MSB was earlier)
                    app_tx_data <= {cam_pack_buf[15:0], cam_data};
                    app_tx_data_valid <= 1'b1;
                    cam_word_count <= cam_word_count + 1'b1;
                    cam_byte_cnt <= 2'd0;
                    cam_pack_buf <= 24'd0;
                end
            end

            if (cam_data_done) begin
                // frame end: if leftover bytes, send last partial word padded with zeros
                if (cam_byte_cnt != 0) begin
                    // left-shift existing bytes to MSB and zero-pad lower bytes
                    app_tx_data <= cam_pack_buf << ((3 - cam_byte_cnt) * 8);
                    app_tx_data_valid <= 1'b1;
                    cam_word_count <= cam_word_count + 1'b1;
                    cam_byte_cnt <= 2'd0;
                    cam_pack_buf <= 24'd0;
                end
                // output total word count and done flag
                app_tx_data_length <= cam_word_count;
                app_tx_data_done <= 1'b1;
                cam_word_count <= 16'd0;
            end
        end else begin
            // SD path (unchanged)
            if (sd_data_valid && sd_data_valid != sd_valid_prev) begin
                app_tx_data <= sd_data;
                app_tx_data_valid <= 1'b1;
                sd_word_cnt <= sd_word_cnt + 1'b1;
            end
            sd_valid_prev <= sd_data_valid;

            if (sd_data_done) begin
                app_tx_data_length <= sd_word_cnt;
                app_tx_data_done <= 1'b1;
                sd_word_cnt <= 16'd0;
            end
        end
    end
end


endmodule