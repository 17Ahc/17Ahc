`timescale 1ns / 1ps
module udp_source_mux(
    input  wire         clk,
    input  wire         reset_n,
    input               mem_clk,
    input  wire        udp_clk,
    input wire sel_cam,
    input                rst_n_v2,
    // 摄像头：8-bit bytes
    input  wire [7:0]   cam_data,
    input  wire         cam_data_valid,
    input  wire [15:0]  cam_data_length, // 字节长度
    input  wire         cam_data_done,    
    input   wire [8:0] sdr2udp_rdusedw,   
    input   wire [23:0] Sdr_rd_dout   ,  
    input  wire  [11:0] udp_wrusedw,   
    // SD/TF：假定 24-bit 单位数据（如原工程）
    input  wire [23:0]  sd_data,
    input  wire         sd_data_valid,
    input  wire [15:0]  sd_data_length,  // 以 24-bit word 为单位或以字节（请按实际单位调整）
    input  wire         sd_data_done,
    input               sys_rst_n_2,
    // 统一输出到 udp 发送模块（24-bit data）
    output reg  [23:0]  app_tx_data,
    output reg          app_tx_data_valid,
    output reg  [15:0]  app_tx_data_length, // 输出以 24-bit words 计数
    output reg          app_tx_data_done
);

wire rst_n/* synthesis syn_keep=1 */;
assign  rst_n = sys_rst_n_2 ;    
 wire cam_sdr_read_req;
 assign cam_sdr_read_req = camser_fifo_re  ;
 wire  sdr2udp_re;
 assign sdr2udp_dout    = {8'h00, Sdr_rd_dout};     // 24->32 拓宽
 assign sdr2udp_rdusedw = udp_wrusedw[8:0];   
// 内部：用于摄像头字节打包
reg [1:0]  cam_byte_cnt;
reg [23:0] cam_pack_buf;
reg [15:0] cam_word_count; // 24-bit word count for camera frames
reg [15:0] cam_len_bytes;  // remaining bytes to process
reg        sd_valid_prev;
reg [15:0] sd_word_cnt;
reg        sd_end_req;
 assign clk_udp= udp_clk;
// 状emachine 简单控制（idle / send）
localparam IDLE = 1'b0, SEND = 1'b1;
reg state;
 pulse_toggle_sync u_cam_req_sync (
     .src_clk  (clk_udp),
     .src_rst_n(rst_n_v2),
     .src_pulse(cam_sdr_read_req),
     .dst_clk  (mem_clk),           // frame_read_write 的1�7 mem_clk
     .dst_rst_n(rst_n),
     .dst_pulse()
 );
  cam_to_udp_serializer  t4(
     .clk            (clk_udp),
     .rst_n          (rst_n_v2),
     .fifo_rd_usedw  (sdr2udp_rdusedw),
     .fifo_dout      (sdr2udp_dout),
     .fifo_re        (camser_fifo_re),
     .cam_data       (camser_word),
     .cam_data_valid (camser_word_valid)
 );
 // ========== 32-bit -> 8-bit 字节序列匄1�7 ==========
 wire [7:0] cam_byte;
 wire       cam_byte_valid;
 wire       cam_byte_last;
  wire [31:0] camser_word;
 wire        camser_word_valid;
 cam_word_to_bytes t5 (
     .clk            (clk_udp),
     .rst_n          (rst_n_v2),
     .in_word        (camser_word),
     .in_word_valid  (camser_word_valid),
     .out_byte       (cam_byte),
     .out_byte_valid (cam_byte_valid),
     .out_word_last  (cam_byte_last)
 );
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        app_tx_data         <= 24'd0;
        app_tx_data_valid   <= 1'b0;
        app_tx_data_length  <= 16'd0;
        app_tx_data_done    <= 1'b0;

        cam_byte_cnt        <= 2'd0;
        cam_pack_buf        <= 24'd0;
        cam_word_count      <= 16'd0;
        cam_len_bytes       <= 16'd0;
        state               <= IDLE;
    end else begin
        // 默认清除有效位（需要时赋为 1）
        app_tx_data_valid <= 1'b0;
        app_tx_data_done  <= 1'b0;

        // Camera 走相应分支，SD 直接转发
        if (sel_cam)  begin
            case (state)
            IDLE: begin
                // 等待摄像头数据开始：在第一个有效字节时初始化并进入 SEND
                if (cam_data_valid) begin
                    cam_pack_buf   <= {16'd0, cam_data}; // 把第1字节放到最低字节，后续左移填充
                    cam_byte_cnt   <= 2'd1;
                    cam_len_bytes  <= cam_data_length;
                    cam_word_count <= (cam_data_length + 2) / 3; // 24-bit words count
                    // 在进入 SEND 前就设定输出长度，避免时序依赖
                    app_tx_data_length <= (cam_data_length + 2) / 3;
                    state <= SEND;
                end
            end

            SEND: begin
                // 在 SEND 状态处理后续字节（cam_data_valid 每字节到来）
                if (cam_data_valid) begin
                    // 把新字节放到低8位并左移已存数据
                    cam_pack_buf <= {cam_pack_buf[15:0], cam_data};
                    cam_byte_cnt <= cam_byte_cnt + 1'b1;

                    // 如果已经拼得 3 个字节（即 cam_byte_cnt 新值 == 3），输出一个 24-bit word
                    if (cam_byte_cnt == 2'd2) begin
                        app_tx_data       <= {cam_pack_buf[15:0], cam_data}; // 完整 24-bit
                        app_tx_data_valid <= 1'b1;

                        // 更新剩余字节计数（减 3 或清零）
                        if (cam_len_bytes <= 3) begin
                            // 最后一包
                            cam_len_bytes <= 16'd0;
                            app_tx_data_done <= 1'b1;
                            state <= IDLE;
                            cam_byte_cnt <= 2'd0;
                            cam_pack_buf <= 24'd0;
                        end else begin
                            cam_len_bytes <= cam_len_bytes - 3;
                            cam_byte_cnt <= 2'd0;
                            cam_pack_buf <= 24'd0;
                            // stay in SEND
                        end
                    end
                end
                // 当摄像头信号表明 frame done 且还有未满 3 字节的残余时，需要在后续时钟输出残余（处理 cam_data_done）
                if (cam_data_done && cam_byte_cnt != 2'd0) begin
                    // 输出残余字节（低位已按上面左移填充，剩余高位为 0）
                    app_tx_data       <= cam_pack_buf;
                    app_tx_data_valid <= 1'b1;
                    app_tx_data_done  <= 1'b1;
                    // 清理
                    cam_byte_cnt      <= 2'd0;
                    cam_pack_buf      <= 24'd0;
                    cam_len_bytes     <= 16'd0;
                    state             <= IDLE;
                end
            end

            endcase
        end else begin
            // SD/TF 路径：直接转发 24-bit words
            if (sd_data_valid) begin
                app_tx_data       <= sd_data;
                app_tx_data_valid <= 1'b1;
                // 在 SD 帧开始时立刻写入长度（更稳健）
                if (app_tx_data_length == 16'd0) app_tx_data_length <= sd_data_length;
            end
            if (sd_data_done) begin
                app_tx_data_done <= 1'b1;
                // 清理 camera 相关状态以防残留
                cam_pack_buf   <= 24'd0;
                cam_byte_cnt   <= 2'd0;
                state          <= IDLE;
            end
        end

        // 当 frame done，被上层捕获后清 length，允许下一帧重新设置
        if (app_tx_data_done) begin
            app_tx_data_length <= 16'd0;
        end
    end
end

endmodule