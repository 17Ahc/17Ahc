module cam_word_to_bytes (
    input  wire        clk,
    input  wire        rst_n,

    // 输入：来自 cam_to_udp_serializer
    input  wire [31:0] in_word,
    input  wire        in_word_valid, // 单词有效

    // 输出：字节流 (udp 域)
    output reg  [7:0]  out_byte,
    output reg         out_byte_valid,
    // 每个 32-bit 单词最后一个字节脉冲（可用于打帧或计数）
    output reg         out_word_last
);

reg [1:0] byte_cnt;
reg [31:0] word_reg;
reg       word_busy;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        byte_cnt       <= 2'd0;
        word_reg       <= 32'd0;
        out_byte       <= 8'd0;
        out_byte_valid <= 1'b0;
        out_word_last  <= 1'b0;
        word_busy      <= 1'b0;
    end
    else begin
        out_byte_valid <= 1'b0;
        out_word_last  <= 1'b0;

        if (!word_busy) begin
            if (in_word_valid) begin
                word_reg  <= in_word;
                word_busy <= 1'b1;
                byte_cnt  <= 2'd0;
                // 立即输出第一个字节（高位先行）
                out_byte       <= in_word[31:24];
                out_byte_valid <= 1'b1;
                byte_cnt       <= 2'd1;
            end
        end
        else begin
            case (byte_cnt)
                2'd0: begin
                    out_byte <= word_reg[31:24];
                    out_byte_valid <= 1'b1;
                    byte_cnt <= 2'd1;
                end
                2'd1: begin
                    out_byte <= word_reg[23:16];
                    out_byte_valid <= 1'b1;
                    byte_cnt <= 2'd2;
                end
                2'd2: begin
                    out_byte <= word_reg[15:8];
                    out_byte_valid <= 1'b1;
                    byte_cnt <= 2'd3;
                end
                2'd3: begin
                    out_byte <= word_reg[7:0];
                    out_byte_valid <= 1'b1;
                    out_word_last <= 1'b1;
                    word_busy <= 1'b0;
                    byte_cnt <= 2'd0;
                end
                default: begin
                    word_busy <= 1'b0;
                    byte_cnt <= 2'd0;
                end
            endcase
        end
    end
end

endmodule
