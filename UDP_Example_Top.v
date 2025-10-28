`timescale 1ns / 1ps
//********************************************************************** 
// -------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>Copyright Notice<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
// ------------------------------------------------------------------- 
//             /\ --------------- 
//            /  \ ------------- 
//           / /\ \ -----------
//          / /  \ \ ---------
//         / /    \ \ ------- 
//        / /      \ \ ----- 
//       / /_ _ _   \ \ --- 
//      /_ _ _ _ _\  \_\ -
//*********************************************************************** 
// Author: suluyang 
// Email:luyang.su@anlogic.com 
// Date:2020/11/17 
// Description: 
// 2022/03/10:  修改时钟结构
//              箢�化约杄1�7
//              添加 soft fifo 
//              添加 debug 功能
// 2023/02/16 :add dynamic_local_ip_address port
// 
// web：www.anlogic.com 
//------------------------------------------------------------------- 
//*********************************************************************/
    `define UDP_LOOP_BACK
//  `define DEBUG_UDP
module UDP_Example_Top(
      input               key1,  
      input  clk_50, 
      input  sys_rst_n,
      input  sd_miso , 
      output sd_clk  , 
      output sd_cs   , 
      output sd_mosi ,   
      input               phy1_rgmii_rx_clk,
      input               phy1_rgmii_rx_ctl,
      input [3:0]         phy1_rgmii_rx_data,
      output wire         phy1_rgmii_tx_clk,
      output wire         phy1_rgmii_tx_ctl,
      output wire [3:0]   phy1_rgmii_tx_data,     
      input               key2,
      output            [3:0] led_data ,
      output            [15:0] dled,
        
    `ifdef DEBUG_UDP
      output wire         debug_out,
    `endif
      output			HDMI_CLK_P,
	  output			HDMI_D2_P,
	  output			HDMI_D1_P,
	  output			HDMI_D0_P,
      
      input               cam_pclk,     
      input               cam_vsync,    
      input               cam_href,     
      input  [7:0]        cam_data,     
      output              cam_rst_n,    
      output              cam_pwdn,     
      output              cam_scl,      
      inout               cam_sda , 
      input               key4_btn
      
);
parameter  DEVICE             = "EG4";//"PH1","EG4"
parameter  LOCAL_UDP_PORT_NUM = 16'h0001;       
parameter  LOCAL_IP_ADDRESS   = 32'hc0a80001;       
parameter  LOCAL_MAC_ADDRESS  = 48'h0123456789ab;
parameter  DST_UDP_PORT_NUM   = 16'h0002;       
parameter  DST_IP_ADDRESS     = 32'hc0a80003;

// 第二个版本的模块输出信号
wire [15:0] dled_led;
wire [15:0] dled_seg;
wire [3:0]  led_data_led;

/*------------------------------*/
/*------------------------------*/
// 第一个版本的信号
wire         app_rx_data_valid;//synthesis keep 
wire [7:0]   app_rx_data;//synthesis keep       
wire [15:0]  app_rx_data_length;//synthesis keep
wire [15:0]  app_rx_port_num;

wire         udp_tx_ready;//synthesis keep
wire         app_tx_ack;//synthesis keep
wire         app_tx_data_request;//synthesis keep
wire         app_tx_data_valid;//synthesis keep 
wire [7:0]   app_tx_data;//synthesis keep       
wire  [15:0] udp_data_length;

wire  [7:0]  tpg_data           ;
wire         tpg_data_valid     ;
wire  [15:0] tpg_data_udp_length;

//temac signals
wire        tx_stop;
wire [7:0]  tx_ifg_val;
wire        pause_req;
wire [15:0] pause_val;
wire [47:0] pause_source_addr;
wire [47:0] unicast_address;
wire [19:0] mac_cfg_vector;  

wire        temac_tx_ready;//synthesis keep
wire        temac_tx_valid;//synthesis keep
wire [7:0]  temac_tx_data;//synthesis keep 
wire        temac_tx_sof;
wire        temac_tx_eof;
            
wire        temac_rx_ready;
wire        temac_rx_valid;//synthesis keep
wire [7:0]  temac_rx_data;//synthesis keep 
wire        temac_rx_sof;
wire        temac_rx_eof;

wire        rx_correct_frame;
wire        rx_error_frame;
wire [1:0]  TRI_speed;

assign TRI_speed = 2'b10;//千兆2'b10 百兆2'b01 十兆2'b00

wire        rx_clk_int; 
wire        rx_clk_en_int;
wire        tx_clk_int; 
wire        tx_clk_en_int;

wire        temac_clk;//synthesis keep
wire        udp_clk;  //synthesis keep
wire        temac_clk90;//synthesis keep
wire        clk_125_out;
wire        clk_12_5_out;
wire        clk_1_25_out;
wire        rx_valid;//synthesis keep   
wire [7:0]  rx_data;//synthesis keep    
wire [7:0]  tx_data; //synthesis keep   
wire        tx_valid; //synthesis keep  
wire        tx_rdy;         
wire        tx_collision;   
wire        tx_retransmit;

wire        reset,reset_reg;
wire        clk_50_out;
reg [7:0]   phy_reset_cnt='d0;
reg [7:0]   soft_reset_cnt=8'hff;
reg sys_rst_n_1,sys_rst_n_2;
wire  key2_internal;
assign key2_internal =sys_rst_n_2;
wire locked;
wire clk_50m ,clk_50m_180deg /* synthesis syn_keep=1 */;
assign  reset = ~sys_rst_n || reset_reg || (soft_reset_cnt != 'd0);
wire clk_sample;//synthesis keep = 1    
wire key4;
wire cam_mode_sel;
assign cam_mode_sel = key4;  
pll_50 u_pll_50(
  .refclk (clk_50)   ,//clk改为SYS_CLK试试
  .reset  (!sys_rst_n_2)   ,
  .extlock(locked)  ,
  .clk0_out  (clk_50m),
  .clk1_out  (clk_50m_180deg),
  .clk2_out  (clk_sample),
  .clk3_out  (pixel_clk_5x ),
  .clk4_out  (pixel_clk    )
);

    always @(posedge clk_50 or negedge sys_rst_n)           
        begin                                        
            if(!sys_rst_n)  begin
sys_rst_n_1 <= 1'b0;
sys_rst_n_2 <= 1'b0;
            end                             
                                                   
            else begin
sys_rst_n_1 <= sys_rst_n;
sys_rst_n_2 <= sys_rst_n_1;
            end                                                                 
        end                                                                          
wire rst_n/* synthesis syn_keep=1 */;
assign  rst_n = sys_rst_n_2 ;    
wire rst_n_v2 = key2;
wire sd_rd_start_en;
wire [31:0] sd_rd_sec_addr ;
wire sd_rd_busy;
wire sd_rd_val_en ,sd_init_done/* synthesis syn_keep=1 */;
wire [15:0] sd_rd_val_data;
wire sdr_wr_en;//synthesis keep 
wire [31:0]sdr_wr_data;//synthesis keep 
wire camser_fifo_re;
wire Sdr_busy;
wire video_clk;
wire video_read_en;
wire [7:0] cam_byte;
wire cam_byte_valid;
wire Sdr_init_ref_vld;
//SD卡顶层控制模坄1�7
sd_ctrl_top t1_sd_ctrl_top(
    .clk_ref                (clk_50m),
    .clk_ref_180deg         (clk_50m_180deg),
    .rst_n                  (rst_n),
    //SD卡接叄1�7
    .sd_miso                (sd_miso),
    .sd_clk                 (sd_clk),
    .sd_cs                  (sd_cs),
    .sd_mosi                (sd_mosi),
    // //用户写SD卡接叄1�7
    // .wr_start_en            (1'b0),                      //不需要写入数捄1�7,写入接口赋����为0
    // .wr_sec_addr            (32'b0),
    // .wr_data                (16'b0),
    // .wr_busy                (),
    // .wr_req                 (),
    // //用户读SD卡接叄1�7
    .rd_start_en            (sd_rd_start_en),
    .rd_sec_addr            (sd_rd_sec_addr),
    .rd_busy                (sd_rd_busy),
    .rd_val_en              (sd_rd_val_en),
    .rd_val_data            (sd_rd_val_data),    
    
    .sd_init_done           (sd_init_done)
    );  


    reg key1_d0,key1_d1;
    always @(posedge clk_50 or negedge rst_n)           
        begin                                        
            if(!rst_n)    begin
            key1_d0<=1'b0;
            key1_d1<=1'b0;           
            end                                                                  
            else  begin
            key1_d0<=key1;  // 使用key1代替原来的sd_reset_rd
            key1_d1<=key1_d0;  
            end                                   
        end      

    wire sd_reset_rd_flag;
    assign   sd_reset_rd_flag =( {key1_d0,key1_d1}  ==   2'b10 )   ?  1'b0 :1'b1;    

                                                      
wire Sdr_init_done /* synthesis syn_keep=1 */;
//读取SD卡图牄1�7
wire full_flag_sdr;
sd_read_photo t2_sd_read_photo(
    .clk                   (clk_50m),
    //系统初始化完成之各1�7,再开始从SD卡中读取图片
    .rst_n                 (rst_n & Sdr_init_done & sd_init_done & sd_reset_rd_flag ), 
    .ddr_max_addr          (24'd307200),       
    .sd_sec_num            (16'd1801), 
    .rd_busy               (sd_rd_busy),
    .sd_rd_val_en          (sd_rd_val_en),
    .sd_rd_val_data        (sd_rd_val_data),
    .rd_start_en           (sd_rd_start_en),
    .rd_sec_addr           (sd_rd_sec_addr),
    .sdr_wr_en             (sdr_wr_en  ),
    .sdr_wr_data           (sdr_wr_data),
    .full_flag_sdr(full_flag_sdr)
    );     

wire Sdr_rd_en            ;//synthesis keep
wire [23:0] Sdr_rd_dout   ;//synthesis keep

wire  [11:0] udp_wrusedw;//synthesis keep
sdram_top t3_sdram (
    .SYS_CLK            (clk_50m),
    .ext_mem_clk        (ext_mem_clk),
    .rst_n              (rst_n & sd_reset_rd_flag),
    .sd_clk             (clk_50m),
    .sdr_data_valid     (sdr_wr_en),
    .sdr_data           (sdr_wr_data),
    .Sdr_rd_en          (Sdr_rd_en),
    .Sdr_rd_dout        (Sdr_rd_dout),
    .Sdr_init_done      (Sdr_init_done),
    .wr_done            (wr_done),
    .full_flag          (full_flag),
    .full_flag_sdr      (full_flag_sdr),
    .udp_wrusedw        (udp_wrusedw)
    );
 
 
 wire [7:0]  cam_data;
 wire        cam_data_valid;
 wire [15:0] cam_data_length;
 wire        cam_data_done;
 wire [23:0] sd_data;
 wire        sd_data_valid;
 wire [15:0] sd_data_length;
 wire        sd_data_done;
 wire [23:0] app_tx_data_src;
 wire        app_tx_data_valid_src;
 wire [15:0] app_tx_data_length_src;
 wire        app_tx_data_done_src;
 wire [15:0] app_tx_data_length;
 wire        app_tx_data_done   ;
reg key4_sync0;
reg key4_sync1;
reg key4_prev;
reg key4_reg;
//always @(posedge ext_mem_clk or negedge rst_n) begin
//    if (!rst_n) begin
//        key4_sync0 <= 1'b1;
//        key4_sync1 <= 1'b1;
//        key4_prev  <= 1'b1;
//        key4_reg   <= 1'b0; // 默认 TF 模式
//    end else begin
//        // 同步外部按键（假定按键为低有效，如按下产生下降沿）
//        key4_sync0 <= key4_btn;
//        key4_sync1 <= key4_sync0;
//
//        // 下降沿检测：前态为 1 且当前为 0 -> 触发翻转
//        if (key4_prev && !key4_sync1) begin
//            key4_reg <= ~key4_reg;
//        end
//
//        key4_prev <= key4_sync1;
//    end
//end
//

parameter  V_CMOS_DISP = 11'd768;                  //CMOS分辨率--行
parameter  H_CMOS_DISP = 11'd1024;                 //CMOS分辨率--列	
parameter  TOTAL_H_PIXEL = H_CMOS_DISP + 12'd1216; //CMOS分辨率--行
parameter  TOTAL_V_PIXEL = V_CMOS_DISP + 12'd504;    
 ov5640_dri u_ov5640_dri(
     .clk               (clk_50m),
     .rst_n             (rst_n),
 
     .cam_pclk          (cam_pclk ),
     .cam_vsync         (cam_vsync),
     .cam_href          (cam_href ),
     .cam_data          (cam_data ),
     .cam_rst_n         (cam_rst_n),
     .cam_pwdn          (cam_pwdn ),
     .cam_scl           (cam_scl  ),
     .cam_sda           (cam_sda  ),
     
     .capture_start     (Sdr_init_done),
     .cmos_h_pixel      (H_CMOS_DISP),
     .cmos_v_pixel      (V_CMOS_DISP),
     .total_h_pixel     (TOTAL_H_PIXEL),
     .total_v_pixel     (TOTAL_V_PIXEL),
     .cmos_frame_vsync  (cmos_frame_vsync),
     .cmos_frame_href   (cmos_frame_href),
     .cmos_frame_valid  (cmos_frame_valid),
     .cmos_frame_data   (cmos_wr_data)
     );   
 wire cam_write_req;
 wire cam_write_req_ack;
 wire cam_write_en;
 wire [31:0] cam_write_data;
   
 ov5640_delay u_ov5640_delay(
     .clk               (cam_pclk),
     .rst_n             (rst_n),
     .cmos_frame_vsync  (cmos_frame_vsync),
     .cmos_frame_href   (cmos_frame_href),
     .cmos_frame_valid  (cmos_frame_valid),
     .cmos_wr_data   (cmos_wr_data),
     
     .cam_write_req(cam_write_req),
     .cam_write_req_ack(cam_write_req_ack),
     .cam_write_en(cam_write_en),
     .cam_write_data(cam_write_data)
 );
 wire ext_mem_clk_sft;
 frame_read_write frame_read_write_m0(
     .mem_clk					(ext_mem_clk),
     .rst						(~rst_n),
     .Sdr_init_done				(Sdr_init_done),
     .Sdr_init_ref_vld			(Sdr_init_ref_vld),
     .Sdr_busy					(Sdr_busy),
     
     .App_rd_en					(App_rd_en),
     .App_rd_addr				(App_rd_addr),
     .Sdr_rd_en					(Sdr_rd_en),
     .Sdr_rd_dout				(Sdr_rd_dout),
     
     .read_clk                   (video_clk           ),
 	.read_req                   (video_read_req           ),
 	.read_req_ack               (video_read_req_ack       ),
 	.read_finish                (                   ),
 	.read_addr_0                (24'd0              ), //first frame base address is 0
 	.read_addr_1                (24'd0         ),
 	.read_addr_2                (24'd0              ),
 	.read_addr_3                (24'd0              ),
 	.read_addr_index            (2'd0               ), //use only read_addr_0
 	.read_len                   (24'd786432         ), //frame size//24'd786432
 	.read_en                    (video_read_en            ),
 	.read_data                  (video_read_data          ),
     
     .App_wr_en					(App_wr_en),
     .App_wr_addr				(App_wr_addr),
     .App_wr_din					(App_wr_din),
     .App_wr_dm					(App_wr_dm),
     
     .write_clk                  (cam_pclk        ),
 	.write_req                  (cam_write_req        ),
 	.write_req_ack              (cam_write_req_ack    ),
 	.write_finish               (                 ),
 	.write_addr_0               (24'd0            ),
 	.write_addr_1               (24'd0       ),
 	.write_addr_2               (24'd0            ),
 	.write_addr_3               (24'd0            ),
 	.write_addr_index           (2'd0             ), //use only write_addr_0
 	.write_len                  (24'd786432       ), //frame size
 	.write_en                   (cam_write_en         ),
 	.write_data                 (cam_write_data       )
 );
 
 wire [8:0] sdr2udp_rdusedw;
 wire [31:0] sdr2udp_dout;
 wire  sdr2udp_re;
  
 // 扩展并映射1�7
 assign sdr2udp_dout    = {8'h00, Sdr_rd_dout};     // 24->32 拓宽
 assign sdr2udp_rdusedw = udp_wrusedw[8:0];         // 佄1�79位作丄1�7 usedw
 // sdr2udp_re 甄1�7 cam_to_udp_serializer 的1�7 fifo_re 驱动刄1�7 Sdr_rd_en

 assign sdr2udp_re = camser_fifo_re;
 wire cam_sdr_read_req;
 assign cam_sdr_read_req = camser_fifo_re  ;
 assign clk_udp= udp_clk;

 // ========== 帧边畄1�7 / 帧长度（占位＄1�7 ==========
 // 说明：建议在写侧（像素域或写兄1�7 SDRAM 时）生成帧长度或 SOF/EOF 并跨域传递����1�7
 // 下面为简化计数����辑，需用真实跨域帧结束信号替换 cam_frame_end_pulse〄1�7
 reg [15:0] cam_frame_len_r;
 reg        cam_frame_done_r;
 reg [15:0] cam_frame_len_count;
 wire cam_frame_end_pulse;
// 把 sel_cam 指向 key4_reg（顶层内部信号）
wire sel_cam;
assign sel_cam = 1'b1;
 udp_source_mux u_udp_source_mux (
     .clk                (clk_udp),
     .reset_n            (rst_n_v2 ),
     .sel_cam            (sel_cam),
 
     .cam_data           (cam_byte),                 // from cam_word_to_bytes
     .cam_data_valid     (cam_byte_valid),
     .cam_data_length    (cam_frame_len_r),          // frame length in bytes (synchronized to clk_udp)
     .cam_data_done      (cam_frame_done_r),
 
     .sd_data            (Sdr_rd_dout),              // if SD path used, else can be 24'd0
     .sd_data_valid      (1'b0),                     // SD feed disable for camera-only test
     .sd_data_length     (16'd0),
     .sd_data_done       (1'b0),
     
     .app_tx_data        (app_tx_data_src),
     .app_tx_data_valid  (app_tx_data_valid_src),
     .app_tx_data_length (app_tx_data_length_src),
     .app_tx_data_done   (app_tx_data_done_src)
 );
wire [8:0] sdr2udp_rdusedw;
wire [23:0] Sdr_rd_dout ;
wire [11:0] udp_wrusedw;    
wire rst_n ;    
wire cam_sdr_read_req;
assign cam_sdr_read_req = camser_fifo_re  ;
wire  sdr2udp_re;
assign sdr2udp_dout    = {8'h00, Sdr_rd_dout};     // 24->32 拓宽
assign sdr2udp_rdusedw = udp_wrusedw[8:0]; 
wire reset_n_udp;
wire reset_n_memclk;
assign reset_n_memclk = rst_n & Sdr_init_done;
assign reset_n_udp    = rst_n & Sdr_init_done;  
assign clk_udp= udp_clk;
pulse_toggle_sync u_cam_req_sync (
    .src_clk  (clk_udp),
    .src_rst_n(reset_n_udp),
    .src_pulse(cam_sdr_read_req),
    .dst_clk  (ext_mem_clk),           // frame_read_write / mem domain clock
    .dst_rst_n(reset_n_memclk),
    .dst_pulse(ext_rd_req_memclk)
);
pulse_toggle_sync u_frame_end_sync (
    .src_clk  (ext_mem_clk),
    .src_rst_n(reset_n_memclk),
    .src_pulse(wr_done),
    .dst_clk  (clk_udp),
    .dst_rst_n(reset_n_udp),
    .dst_pulse(cam_frame_end_pulse)
);
  cam_to_udp_serializer  t4(
     .clk            (clk_udp),
     .rst_n          (reset_n_udp),
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
     .rst_n          (reset_n_udp),
     .in_word        (camser_word),
     .in_word_valid  (camser_word_valid),
     .out_byte       (cam_byte),
     .out_byte_valid (cam_byte_valid),
     .out_word_last  (cam_byte_last)
 );
always @(posedge clk_udp or negedge rst_n_v2) begin
    if (!rst_n_v2) begin
        cam_frame_len_r    <= 16'd0;
        cam_frame_done_r   <= 1'b0;
        cam_frame_len_count<= 16'd0;
    end
    else begin
        cam_frame_done_r <= 1'b0;
        if (cam_byte_valid) begin
            if (cam_frame_len_count != 16'hFFFF)
                cam_frame_len_count <= cam_frame_len_count + 1;
        end
        if (cam_frame_end_pulse) begin
            cam_frame_len_r    <= cam_frame_len_count;
            cam_frame_done_r   <= 1'b1;
            cam_frame_len_count<= 16'd0;
        end
    end
end

always @(posedge clk_50_out or negedge sys_rst_n)
begin
    if(~sys_rst_n)
        phy_reset_cnt<='d0;
    else if(phy_reset_cnt < 255)
        phy_reset_cnt<= phy_reset_cnt+1;
    else
        phy_reset_cnt<=phy_reset_cnt;
end

assign  reset = ~sys_rst_n || reset_reg || (soft_reset_cnt != 'd0);
assign  phy_reset = phy_reset_cnt[7];


always @(posedge udp_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
        soft_reset_cnt<=8'hff;
    else if(soft_reset_cnt > 0)
        soft_reset_cnt<= soft_reset_cnt-1;
    else
        soft_reset_cnt<=soft_reset_cnt;
end

// 第二个版本的缓冲区����辑
reg [71:0] led_buffer;  // LED模式数据缓冲匄1�7
reg [71:0] seg_buffer;  // 数码管模式数据缓冲区
reg led_buffer_valid;   // LED缓冲区有效标忄1�7
reg seg_buffer_valid;   // 数码管缓冲区有效标志
reg current_buffer;     // 当前活跃缓冲匄1�7: 0=LED, 1=数码箄1�7
reg [3:0] byte_cnt;     // 计数0-8，共9个字芄1�7

always @(posedge udp_clk or negedge rst_n_v2) begin
    if(!rst_n_v2) begin
        byte_cnt <= 4'b0;
        current_buffer <= 1'b0; // 默认LED缓冲匄1�7
        led_buffer_valid <= 1'b0;
        seg_buffer_valid <= 1'b0;
        led_buffer <= 72'h0;
        seg_buffer <= 72'h0;
    end
    else begin
        led_buffer_valid <= 1'b0; // 默认清除有效标志
        seg_buffer_valid <= 1'b0;
        
        if (app_rx_data_valid) begin
            if (byte_cnt < 9) begin
                // 在第丢�个字节检测模弄1�7
                if (byte_cnt == 0) begin
                    case (app_rx_data)
                        8'h55: current_buffer <= 1'b0; // LED模式
                        8'hAA: current_buffer <= 1'b1; // 数码管模弄1�7
                        default: current_buffer <= current_buffer;
                    endcase
                end
                
                // 存储当前字节到对应位罄1�7
                if (current_buffer == 1'b0) begin
                    // 存储到LED缓冲匄1�7
                    case(byte_cnt)
                        0: led_buffer[71:64] <= app_rx_data;
                        1: led_buffer[63:56] <= app_rx_data;
                        2: led_buffer[55:48] <= app_rx_data;
                        3: led_buffer[47:40] <= app_rx_data;
                        4: led_buffer[39:32] <= app_rx_data;
                        5: led_buffer[31:24] <= app_rx_data;
                        6: led_buffer[23:16] <= app_rx_data;
                        7: led_buffer[15:8]  <= app_rx_data;
                        8: led_buffer[7:0]   <= app_rx_data;
                    endcase
                end else begin
                    // 存储到数码管缓冲匄1�7
                    case(byte_cnt)
                        0: seg_buffer[71:64] <= app_rx_data;
                        1: seg_buffer[63:56] <= app_rx_data;
                        2: seg_buffer[55:48] <= app_rx_data;
                        3: seg_buffer[47:40] <= app_rx_data;
                        4: seg_buffer[39:32] <= app_rx_data;
                        5: seg_buffer[31:24] <= app_rx_data;
                        6: seg_buffer[23:16] <= app_rx_data;
                        7: seg_buffer[15:8]  <= app_rx_data;
                        8: seg_buffer[7:0]   <= app_rx_data;
                    endcase
                end
                
                byte_cnt <= byte_cnt + 1;
            end
            else begin
                // 收到完整9字节后，设置有效标志并重置计数器
                byte_cnt <= 4'b0;
                if (current_buffer == 1'b0) begin
                    led_buffer_valid <= 1'b1;
                end else begin
                    seg_buffer_valid <= 1'b1;
                end
            end
        end
    end
end

`ifdef DEBUG_UDP
//=========================================================
//debug signal
//=========================================================
reg       debug_app_rx_data_valid   ;//synthesis keep
reg [7:0] debug_app_rx_data         ;//synthesis keep
reg       debug_app_tx_data_valid   ;//synthesis keep
reg [7:0] debug_app_tx_data         ;//synthesis keep
reg       debug_temac_tx_valid      ;//synthesis keep
reg [7:0] debug_temac_tx_data       ;//synthesis keep
reg       debug_temac_rx_valid      ;//synthesis keep
reg [7:0] debug_temac_rx_data       ;//synthesis keep
reg       debug_rx_valid            ;//synthesis keep
reg [7:0] debug_rx_data             ;//synthesis keep
reg       debug_tx_valid            ;//synthesis keep
reg [7:0] debug_tx_data             ;//synthesis keep

reg [31:0] debug_frame_temac_cnt_rx ;//synthesis keep
reg [31:0] debug_frame_app_cnt_rx   ;//synthesis keep
reg [31:0] debug_frame_fifo_cnt_rx  ;//synthesis keep
reg [31:0] debug_frame_temac_cnt_tx ;//synthesis keep
reg [31:0] debug_frame_app_cnt_tx   ;//synthesis keep
reg [31:0] debug_frame_fifo_cnt_tx  ;//synthesis keep

wire udp_debug_out;
// wire debug_out;
assign debug_out =   debug_app_rx_data_valid
                   | debug_app_rx_data      
                   | debug_app_tx_data_valid
                   | debug_app_tx_data      
                   | debug_temac_tx_valid   
                   | debug_temac_tx_data    
                   | debug_temac_rx_valid   
                   | debug_temac_rx_data    
                   | debug_rx_valid         
                   | debug_rx_data          
                   | debug_tx_valid       
                   | debug_tx_data
                   | debug_frame_temac_cnt_rx
                   | debug_frame_app_cnt_rx  
                   | debug_frame_fifo_cnt_rx 
                   | debug_frame_temac_cnt_tx
                   | debug_frame_app_cnt_tx  
                   | debug_frame_fifo_cnt_tx 
                   | udp_debug_out;

reg       debug_0;
reg [7:0] debug_1;
reg       debug_2;
reg [7:0] debug_3;
reg       debug_4;
reg [7:0] debug_5;
reg       debug_6;
reg [7:0] debug_7;
reg       debug_8;
reg [7:0] debug_9;
reg       debug_a;
reg [7:0] debug_b;

reg       debug_0_d;
reg [7:0] debug_1_d;
reg       debug_2_d;
reg [7:0] debug_3_d;
reg       debug_4_d;
reg [7:0] debug_5_d;
reg       debug_6_d;
reg [7:0] debug_7_d;
reg       debug_8_d;
reg [7:0] debug_9_d;
reg       debug_a_d;
reg [7:0] debug_b_d;

always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_app_rx_data_valid <= 'd0;
        debug_app_rx_data       <= 'd0;
        debug_app_tx_data_valid <= 'd0;
        debug_app_tx_data       <= 'd0;
        debug_temac_tx_valid    <= 'd0;
        debug_temac_tx_data     <= 'd0;
        debug_temac_rx_valid    <= 'd0;
        debug_temac_rx_data     <= 'd0;
        debug_rx_valid          <= 'd0;
        debug_rx_data           <= 'd0;
        debug_tx_valid          <= 'd0;
        debug_tx_data           <= 'd0;
    end
    else
    begin
        debug_app_rx_data_valid <=debug_0_d;
        debug_app_rx_data       <=debug_1_d;
        debug_app_tx_data_valid <=debug_2_d;
        debug_app_tx_data       <=debug_3_d;
        debug_temac_tx_valid    <=debug_4_d;
        debug_temac_tx_data     <=debug_5_d;
        debug_temac_rx_valid    <=debug_6_d;
        debug_temac_rx_data     <=debug_7_d;
        debug_rx_valid          <=debug_8_d;
        debug_rx_data           <=debug_9_d;
        debug_tx_valid          <=debug_a_d;
        debug_tx_data           <=debug_b_d;
    end
end

always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_0_d <= 'd0;
        debug_1_d <= 'd0;
        debug_2_d <= 'd0;
        debug_3_d <= 'd0;
        debug_4_d <= 'd0;
        debug_5_d <= 'd0;
        debug_6_d <= 'd0;
        debug_7_d <= 'd0;
        debug_8_d <= 'd0;
        debug_9_d <= 'd0;
        debug_a_d <= 'd0;
        debug_b_d <= 'd0;
    end
    else
    begin
        debug_0_d <= debug_0 ;
        debug_1_d <= debug_1 ;
        debug_2_d <= debug_2 ;
        debug_3_d <= debug_3 ;
        debug_4_d <= debug_4 ;
        debug_5_d <= debug_5 ;
        debug_6_d <= debug_6 ;
        debug_7_d <= debug_7 ;
        debug_8_d <= debug_8 ;
        debug_9_d <= debug_9 ;
        debug_a_d <= debug_a ;
        debug_b_d <= debug_b ;
    end
end

always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_0 <= 'd0;
        debug_1 <= 'd0;
        debug_2 <= 'd0;
        debug_3 <= 'd0;
        debug_4 <= 'd0;
        debug_5 <= 'd0;
        debug_6 <= 'd0;
        debug_7 <= 'd0;
        debug_8 <= 'd0;
        debug_9 <= 'd0;
        debug_a <= 'd0;
        debug_b <= 'd0;
    end
    else
    begin
        debug_0 <= app_rx_data_valid   ;
        debug_1 <= app_rx_data         ;
        debug_2 <= app_tx_data_valid   ;
        debug_3 <= app_tx_data         ;
        debug_4 <= !temac_tx_valid     ;
        debug_5 <= temac_tx_data       ;
        debug_6 <= !temac_rx_valid     ;
        debug_7 <= temac_rx_data       ;
        debug_8 <= rx_valid            ;
        debug_9 <= rx_data             ;
        debug_a <= tx_valid            ;
        debug_b <= tx_data             ;
    end
end

always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_frame_fifo_cnt_rx <= 'd0;
    end
    else if( !debug_6_d && debug_6)
    begin
        debug_frame_fifo_cnt_rx <= debug_frame_fifo_cnt_rx + 'd1;
    end
    else
    begin
        debug_frame_fifo_cnt_rx <=debug_frame_fifo_cnt_rx;
    end
end

always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_frame_fifo_cnt_tx <= 'd0;
    end
    else if( !debug_4_d && debug_4)
    begin
        debug_frame_fifo_cnt_tx <= debug_frame_fifo_cnt_tx + 'd1;
    end
    else
    begin
        debug_frame_fifo_cnt_tx <=debug_frame_fifo_cnt_tx;
    end
end


always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_frame_app_cnt_rx  <= 'd0;
    end
    else if( !debug_0_d && debug_0)
    begin
        debug_frame_app_cnt_rx  <= debug_frame_app_cnt_rx + 'd1;
    end
    else
    begin
        debug_frame_app_cnt_rx  <=debug_frame_app_cnt_rx;
    end
end

always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_frame_app_cnt_tx  <= 'd0;
    end
    else if( !debug_2_d && debug_2)
    begin
        debug_frame_app_cnt_tx  <= debug_frame_app_cnt_tx + 'd1;
    end
    else
    begin
        debug_frame_app_cnt_tx  <=debug_frame_app_cnt_tx;
    end
end

always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_frame_temac_cnt_rx    <= 'd0;
    end
    else if( !debug_8_d && debug_8)
    begin
        debug_frame_temac_cnt_rx    <= debug_frame_temac_cnt_rx + 'd1;
    end
    else
    begin
        debug_frame_temac_cnt_rx    <=debug_frame_temac_cnt_rx;
    end
end

always @(posedge temac_clk or negedge sys_rst_n)
begin
    if(~sys_rst_n)
    begin
        debug_frame_temac_cnt_tx    <= 'd0;
    end
    else if( !debug_a_d && debug_a)
    begin
        debug_frame_temac_cnt_tx    <= debug_frame_temac_cnt_tx + 'd1;
    end
    else
    begin
        debug_frame_temac_cnt_tx    <=debug_frame_temac_cnt_tx;
    end
end

`endif

//============================================================
// 参数配置逻辑
//============================================================
//霢�配置的客户端接口（初始默认����）
assign  tx_stop    = 1'b0;
assign  tx_ifg_val = 8'h00;
assign  pause_req  = 1'b0;
assign  pause_val  = 16'h0;
assign  pause_source_addr = 48'h5af1f2f3f4f5;
// assign  unicast_address   = 48'hab8967452301;
assign  unicast_address   = {   LOCAL_MAC_ADDRESS[7:0],
                                LOCAL_MAC_ADDRESS[15:8],
                                LOCAL_MAC_ADDRESS[23:16],
                                LOCAL_MAC_ADDRESS[31:24],
                                LOCAL_MAC_ADDRESS[39:32],
                                LOCAL_MAC_ADDRESS[47:40]
                            };


assign  mac_cfg_vector    = {1'b0,2'b00,TRI_speed,8'b00000010,7'b0000010}; //地址过滤模式、流控配置��������度配置、接收器配置、发送器配置
//assign  mac_cfg_vector    = {1'b0,2'b00,2'b10,8'b00000010,7'b0000010}; //地址过滤模式、流控配置��������度配置、接收器配置、发送器配置
//assign  mac_cfg_vector    = {1'b0,2'b00,2'b01,8'b00000010,7'b0000010}; //地址过滤模式、流控配置��������度配置、接收器配置、发送器配置
//assign  mac_cfg_vector    = {1'b0,2'b00,2'b00,8'b00000010,7'b0000010}; //地址过滤模式、流控配置��������度配置、接收器配置、发送器配置

//-----------------------------------------------------
//test dynamic_local_ip_address
//-----------------------------------------------------

//参数定义
reg [32:0] cnt0;
wire      end_cnt0;
wire      add_cnt0;
reg [7:0] cnt1;
wire      end_cnt1;
wire      add_cnt1;

//计数噄1�72
 always @(posedge udp_clk or negedge sys_rst_n_2)begin
     if(!sys_rst_n_2)begin
         cnt0 <= 0;
     end
     else if(add_cnt0)begin
         if(end_cnt0)
             cnt0 <= 0;
         else
             cnt0 <= cnt0 + 1;
     end
 end

 assign add_cnt0 = 1;
 assign end_cnt0 = add_cnt0 && 0;

 always @(posedge udp_clk or negedge sys_rst_n_2)begin 
     if(!sys_rst_n_2)begin
         cnt1 <= 0;
     end
     else if(add_cnt1)begin
         if(end_cnt1)
             cnt1 <= 0;
         else
             cnt1 <= cnt1 + 1;
     end
 end

 assign add_cnt1 = end_cnt0;
 assign end_cnt1 = add_cnt1 && cnt1== 15;  

reg [31:0]  input_local_ip_address;
reg         input_local_ip_address_valid;

always@(posedge udp_clk or posedge reset)
begin
    if(reset) 
    begin
        input_local_ip_address      <= LOCAL_IP_ADDRESS;
        input_local_ip_address_valid<= 1'b0;
    end
    else if(end_cnt0 == 1'b1)
    begin
        input_local_ip_address      <= {LOCAL_IP_ADDRESS[31:8],cnt1};
        input_local_ip_address_valid<= 1'b1;
    end
    else
    begin
        input_local_ip_address      <= input_local_ip_address;
        input_local_ip_address_valid<= 1'b1;
    end
end

reg [15:0] input_local_udp_port_num;
reg        input_local_udp_port_num_valid;

always@(posedge udp_clk or posedge reset)
begin
    if(reset) 
    begin
        input_local_udp_port_num      <= LOCAL_UDP_PORT_NUM;
        input_local_udp_port_num_valid<= 1'b0;
    end
    else
    begin
        input_local_udp_port_num      <= input_local_ip_address[3:0] + 3;
        input_local_udp_port_num_valid<= 1'b1;
    end
end


// // -----------------------------------------------------
wire VGA_EN;
wire  dis_en;
wire [11:0] VGA_D;

// // app
app u1_app (
    .sys_clk                    (pixel_clk                ),
    .udp_rx_clk                 (udp_clk                ),
    .udp_tx_clk                 (udp_clk                ),
    .reset                      (key2                  ), 
    .app_rx_data_valid          (app_rx_data_valid      ), 
    .app_rx_data                (app_rx_data            ), 
    .app_rx_data_length         (app_rx_data_length     ), 
    .app_rx_port_num            (app_rx_port_num        ),
    .VGA_HSYNC	                (VGA_HSYNC              ),
	.VGA_VSYNC 	                (VGA_VSYNC              ),
	.VGA_D                      (VGA_D                  ),
    .rd_en                      (rd_en                  ),
    .VGA_EN                     (VGA_EN)
);      

    	wire [7:0]	VGA_R;
		wire [7:0]	VGA_G;
		wire [7:0]	VGA_B;

assign VGA_R = {VGA_D[11:8],4'b0};	
assign VGA_G = {VGA_D[7:4],4'b0};
assign VGA_B = {VGA_D[3:0],4'b0};
hdmi_tx #(.FAMILY("EG4"))	//EF2、EF3、EG4、AL3、PH1

 u2_hdmi_tx
	(
		.PXLCLK_I(pixel_clk),
		.PXLCLK_5X_I(pixel_clk_5x),

		.RST_N (key2),
		
		//VGA
		.VGA_HS (VGA_HSYNC ),
		.VGA_VS (VGA_VSYNC ),
		.VGA_DE (VGA_EN ),
		.VGA_RGB({VGA_R,VGA_G,VGA_B}),

		//HDMI
		.HDMI_CLK_P(HDMI_CLK_P),
		.HDMI_D2_P (HDMI_D2_P ),
		.HDMI_D1_P (HDMI_D1_P ),
		.HDMI_D0_P (HDMI_D0_P )	
		
	);

clk_gen_rst_gen#(
    .DEVICE         (DEVICE     )
)u_clk_gen(
    .reset                (~sys_rst_n                ),//~sys_rst_n 
    .clk_in         (clk_50     ),
    .rst_out        (reset_reg  ),
    .clk_125_out0   (temac_clk  ),
    .clk_125_out1   (clk_125_out),
    .clk_125_out2   (temac_clk90),
    .clk_12_5_out   (clk_12_5_out),
    .clk_1_25_out   (clk_1_25_out),
    .clk_25_out     (clk_50_out )
);

// 第二个版本的LED和数码管模块
led u_led_module(
    .app_rx_data_valid       (led_buffer_valid),
    .app_rx_data_buffer      (led_buffer),
    .udp_rx_clk              (udp_clk),
    .reset                   (rst_n_v2),
    .led_data_1              (led_data_led),
    .dled                    (dled_led)
);

seg_control u_seg_control_module(
    .app_rx_data_valid       (seg_buffer_valid),
    .app_rx_data_buffer      (seg_buffer),
    .udp_rx_clk              (udp_clk),
    .reset                   (rst_n_v2),
    .dled                    (dled_seg)
);

assign dled = (current_buffer == 1'b1) ? dled_seg : dled_led;
assign led_data = led_data_led;

udp_data_tpg u1_udp_data_tpg(
    .clk                (udp_clk            ),
    .reset              (~key2_internal              ),

    .tpg_data           (tpg_data           ),//数据输出
    .tpg_data_valid     (tpg_data_valid     ),//数据有效信号
    .tpg_data_udp_length(tpg_data_udp_length),//数据长度（包含帧头）
    .tpg_data_done      (tpg_data_done      ),
    
    .tpg_data_enable    (phy_reset          ),
    .tpg_data_header0   (16'haabb           ),//帧头0
    .tpg_data_header1   (16'hccdd           ),//帧头1
    .tpg_data_type      (16'ha8b8           ),//数据帧类垄1�7
    .tpg_data_length    (16'h00ff           ),//数据长度500
    .tpg_data_num       (16'h000a           ),//产生的帧个数10
    .tpg_data_ifg       (8'd130             )
);

wire [23:0] image_data;
// assign image_data = (Sdr_rd_dout[23:16] * 76 +Sdr_rd_dout[23:16] * 150 +Sdr_rd_dout[7:0] * 30 ) >>8;
assign image_data = Sdr_rd_dout[23:0] ;

//------------------------------------------------------------
//udp_loopback
//------------------------------------------------------------

udp_loopback#(
    .DEVICE(DEVICE)
)
 u2_udp_loopback
 (
    .app_rx_clk                 (ext_mem_clk                ),
    .app_tx_clk                 (udp_clk                ),
    .reset                      (reset                ),//reset
    .udp_wrusedw                   (udp_wrusedw),
    `ifdef UDP_LOOP_BACK    
    .app_rx_data           (app_tx_data_src),       // 24-bit stream from udp_source_mux
    .app_rx_data_valid     (app_tx_data_valid_src),
    .app_rx_data_length    (app_tx_data_length_src),
    `else   
    .app_rx_data                (tpg_data               ),
    .app_rx_data_valid          (tpg_data_valid         ),
    .app_rx_data_length         (tpg_data_udp_length    ),
    `endif              
    .full_flag                  (full_flag),
    .udp_tx_ready               (udp_tx_ready           ),
    .app_tx_ack                 (app_tx_ack             ),
    .app_tx_data                (app_tx_data            ),
    .app_tx_data_request        (app_tx_data_request    ),
    .app_tx_data_valid          (app_tx_data_valid      ),
    .udp_data_length            (udp_data_length        )   
);


// udp_loopback#(
//    .DEVICE(DEVICE)
// )
// u2_udp_loopback
// (
//    .app_rx_clk                 (udp_clk                ),
//    .app_tx_clk                 (udp_clk                ),
//    .reset                      (reset                 ),//reset
    
//    `ifdef UDP_LOOP_BACK    
//    .app_rx_data                (app_rx_data            ),
//    .app_rx_data_valid          (app_rx_data_valid      ),
//    .app_rx_data_length         (app_rx_data_length     ),
//    `else   
//    .app_rx_data                (tpg_data               ),
//    .app_rx_data_valid          (tpg_data_valid         ),
//    .app_rx_data_length         (tpg_data_udp_length    ),
//    `endif              
    
//    .udp_tx_ready               (udp_tx_ready           ),
//    .app_tx_ack                 (app_tx_ack             ),
//    .app_tx_data                (app_tx_data            ),
//    .app_tx_data_request        (app_tx_data_request    ),
//    .app_tx_data_valid          (app_tx_data_valid      ),
//    .udp_data_length            (udp_data_length        )   
// );
//------------------------------------------------------------  
//UDP
//------------------------------------------------------------       
udp_ip_protocol_stack #
(
    .DEVICE                     (DEVICE                 ),
    .LOCAL_UDP_PORT_NUM         (LOCAL_UDP_PORT_NUM     ),
    .LOCAL_IP_ADDRESS           (LOCAL_IP_ADDRESS       ),
    .LOCAL_MAC_ADDRESS          (LOCAL_MAC_ADDRESS      )
)   
u3_udp_ip_protocol_stack    
(   
    .udp_rx_clk                 (udp_clk                ),
    .udp_tx_clk                 (udp_clk                ),
    .reset                      (reset                 ), 
    .udp2app_tx_ready           (udp_tx_ready           ), 
    .udp2app_tx_ack             (app_tx_ack             ), 
    .app_tx_request             (app_tx_data_request    ), 
    .app_tx_data_valid          (app_tx_data_valid      ), 
    .app_tx_data                (app_tx_data            ), 
    .app_tx_data_length         (udp_data_length        ), 
    .app_tx_dst_port            (DST_UDP_PORT_NUM       ), 
    .ip_tx_dst_address          (DST_IP_ADDRESS         ), 
    
    .input_local_udp_port_num      (input_local_udp_port_num      ),
    .input_local_udp_port_num_valid(input_local_udp_port_num_valid),
    
    .input_local_ip_address     (input_local_ip_address     ),
    .input_local_ip_address_valid(input_local_ip_address_valid),
    
    .app_rx_data_valid          (app_rx_data_valid            ), 
    .app_rx_data                (app_rx_data                        ), 
    .app_rx_data_length         (app_rx_data_length          ), 
    .app_rx_port_num            (app_rx_port_num                ), 
    .temac_rx_ready             (temac_rx_ready         ),//output
    .temac_rx_valid             (!temac_rx_valid        ),//input
    .temac_rx_data              (temac_rx_data          ),//input
    .temac_rx_sof               (temac_rx_sof           ),//input
    .temac_rx_eof               (temac_rx_eof           ),//input
    .temac_tx_ready             (temac_tx_ready         ),//input
    .temac_tx_valid             (temac_tx_valid         ),//output
    .temac_tx_data              (temac_tx_data          ),//output
    .temac_tx_sof               (temac_tx_sof           ),//output
    .temac_tx_eof               (temac_tx_eof           ),//output
`ifdef DEBUG_UDP
    .udp_debug_out              (udp_debug_out          ),
`endif
    .ip_rx_error                (                       ), 
    .arp_request_no_reply_error (                       )
);
wire phy1_rgmii_rx_clk_0;
wire phy1_rgmii_rx_clk_90;
rx_pll u_rx_pll(
	.refclk		( phy1_rgmii_rx_clk),
    .reset       (1'b0),
    .clk0_out	(phy1_rgmii_rx_clk_0),
	.clk1_out	(phy1_rgmii_rx_clk_90),
    .clk3_out   (ext_mem_clk         )
);
//------------------------------------------------------------  
//TEMAC
//------------------------------------------------------------  
temac_block#(
    .DEVICE               (DEVICE                   )
)  
u4_trimac_block
(
    .reset                (reset                   ),
    .gtx_clk              (clk_125_out                ),//input   125M
    .gtx_clk_90           (temac_clk90                ),//input   125M
    .rx_clk               (rx_clk_int               ),//output  125M 25M    2.5M
    .rx_clk_en            (rx_clk_en_int            ),//output  1    12.5M  1.25M
    .rx_data              (rx_data                  ),
    .rx_data_valid        (rx_valid                 ),
    .rx_correct_frame     (rx_correct_frame         ),
    .rx_error_frame       (rx_error_frame           ),
    .rx_status_vector     (                         ),
    .rx_status_vld        (                         ),
//  .tri_speed            (tri_speed                ),//output
    .tx_clk               (tx_clk_int               ),//output  125M
    .tx_clk_en            (tx_clk_en_int            ),//output  1    12.5M  1.25M 占空比不寄1�7
    .tx_data              (tx_data                  ),
    .tx_data_en           (tx_valid                 ),
    .tx_rdy               (tx_rdy                   ),//temac_tx_ready
    .tx_stop              (tx_stop                  ),//input
    .tx_collision         (tx_collision             ),
    .tx_retransmit        (tx_retransmit            ),
    .tx_ifg_val           (tx_ifg_val               ),//input
    .tx_status_vector     (                         ),
    .tx_status_vld        (                         ),
    .pause_req            (pause_req                ),//input
    .pause_val            (pause_val                ),//input
    .pause_source_addr    (pause_source_addr        ),//input
    .unicast_address      (unicast_address          ),//input
    .mac_cfg_vector       (mac_cfg_vector           ),//input
    .rgmii_txd            (phy1_rgmii_tx_data       ),
    .rgmii_tx_ctl         (phy1_rgmii_tx_ctl        ),
    .rgmii_txc            (phy1_rgmii_tx_clk        ),
    .rgmii_rxd            (phy1_rgmii_rx_data       ),
    .rgmii_rx_ctl         (phy1_rgmii_rx_ctl        ),
    .rgmii_rxc            (phy1_rgmii_rx_clk_90        ),
    .inband_link_status   (                         ),
    .inband_clock_speed   (                         ),
    .inband_duplex_status (                         )
);

udp_clk_gen#(
    .DEVICE               (DEVICE                   )
)           
u5_temac_clk_gen(           
    .reset                (~sys_rst_n               ),//~sys_rst_n 
    .tri_speed            (TRI_speed                ),
    .clk_125_in           (clk_125_out              ),//125M  
    .clk_12_5_in          (clk_12_5_out             ),//12.5M 
    .clk_1_25_in          (clk_1_25_out             ),//1.25M 
    .udp_clk_out          (udp_clk                  )
);

tx_client_fifo #
(
    .DEVICE               (DEVICE                   )
)
u6_tx_fifo
(
    .rd_clk               (tx_clk_int               ),
    .rd_sreset            (reset                   ),
    .rd_enable            (tx_clk_en_int            ),
    .tx_data              (tx_data                  ),
    .tx_data_valid        (tx_valid                 ),
    .tx_ack               (tx_rdy                   ),
    .tx_collision         (tx_collision             ),
    .tx_retransmit        (tx_retransmit            ),
    .overflow             (                         ),
                            
    .wr_clk               (udp_clk                  ),
    .wr_sreset            (reset                   ),
    .wr_data              (temac_tx_data            ),
    .wr_sof_n             (temac_tx_sof             ),
    .wr_eof_n             (temac_tx_eof             ),
    .wr_src_rdy_n         (temac_tx_valid           ),
    .wr_dst_rdy_n         (temac_tx_ready           ),//temac_tx_ready
    .wr_fifo_status       (                         )
);

rx_client_fifo# 
(
    .DEVICE               (DEVICE                   )
)                           
u7_rx_fifo                  
(                           
    .wr_clk               (rx_clk_int               ),
    .wr_enable            (rx_clk_en_int            ),
    .wr_sreset            (reset                    ),
    .rx_data              (rx_data                  ),
    .rx_data_valid        (rx_valid                 ),
    .rx_good_frame        (rx_correct_frame         ),
    .rx_bad_frame         (rx_error_frame           ),
    .overflow             (                         ),
    .rd_clk               (udp_clk                  ),
    .rd_sreset            (reset                   ),
    .rd_data_out          (temac_rx_data            ),//output reg [7:0] rd_data_out,
    .rd_sof_n             (temac_rx_sof             ),//output reg       rd_sof_n,
    .rd_eof_n             (temac_rx_eof             ),//output           rd_eof_n,
    .rd_src_rdy_n         (temac_rx_valid           ),//output reg       rd_src_rdy_n,
    .rd_dst_rdy_n         (temac_rx_ready           ),//input            rd_dst_rdy_n,
    .rx_fifo_status       (                         )
);



endmodule