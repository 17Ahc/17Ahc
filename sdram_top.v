`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: anlgoic
// Author: 	xg
// description:  sdram top wrapper (merged / adapted to camera-standard engine usage)
// This version accepts both SYS_CLK and ext_mem_clk so it matches the top-level
// instantiation in your camera project (which passes both clk_50m and ext_mem_clk).
//
// Key points:
//  - Keeps the existing app_wrrd + sdr_as_ram + PHY instantiation structure,
//    but exposes/accepts ext_mem_clk so frame_read_write (mem domain) and
//    the SDRAM controller use the same mem clock from top.
//  - Preserves Sdr_init_done, wr_done, Sdr_rd_en, Sdr_rd_dout, and udp_wrusedw signals
//    to match the rest of your design and the camera standard engine wiring.
//  - Use SYS_CLK as reference clock for PLL that produces Clk_sft used by sdr_as_ram.
//    ext_mem_clk is passed into the app_wrrd and sdr_as_ram as Sdr_clk (mem domain).
//////////////////////////////////////////////////////////////////////////////////

`define DEBUG

`include "../include/global_def.v"

module sdram_top(
    input                                    SYS_CLK,          // board/system reference clock (e.g. 50MHz)
    input                                    ext_mem_clk,      // external memory clock (from top / pll)
    input                                    rst_n,
    input                                    sd_clk,           // preserved for compatibility (unused in many flows)
   // output                                   LED,
    output                                   Sdr_init_done,
    output                                   wr_done,
    input                                    sdr_data_valid,
    input              [  23: 0]             sdr_data,
    output                                   Sdr_rd_en,
    output             [`DATA_WIDTH-1: 0]    Sdr_rd_dout,
    input                                    full_flag,
    output                                   full_flag_sdr,
    input           [11:0]                   udp_wrusedw,
    output                                   SDRAM_CLK,
    output                                   SDR_CKE,
    output                                   SDR_RAS,
    output                                   SDR_CAS,
    output                                   SDR_WE,
    output             [`BA_WIDTH-1: 0]      SDR_BA,
    output             [`ROW_WIDTH-1:0]      SDR_ADDR,
    output             [`DM_WIDTH-1:0]       SDR_DM,
    inout              [`DATA_WIDTH-1:0]     SDR_DQ
    );

    // Internal signals
    wire                                lock;
    wire                                local_clk;
    wire                                Clk_sft;
    wire                                Rst_n;
    wire                                Rst /*synthesis syn_keep=1 */;
    wire                                Sdr_init_ref_vld;
    wire                                Sdr_busy;

    // app <-> sdr interface wires
    wire                                App_wr_en;
    wire               [`ADDR_WIDTH-1: 0]App_wr_addr;
    wire               [`DM_WIDTH-1: 0] App_wr_dm;
    wire               [`DATA_WIDTH-1: 0]App_wr_din;
    wire                                App_rd_en;
    wire               [`ADDR_WIDTH-1: 0]App_rd_addr;

    // ------------------------------------------------------------------
    // PLL: derive any shifted clocks required by vendor sdr_as_ram instance
    // SYS_CLK is used as PLL reference to generate Clk_sft used by sdr_as_ram.
    // ext_mem_clk (provided by top) is used as Sdr_clk (memory domain).
    // ------------------------------------------------------------------
    clk_pll u0_clk(
        .refclk    (SYS_CLK),
        .reset     (1'b0),
        .extlock   (lock),
        .clk0_out  (local_clk),    // not used directly, kept for compatibility
        .clk2_out  (Clk_sft),      // shifted clock used by sdr_as_ram
        .clk3_out  ()              // video_clk (unused here)
    );

    // Combine reset with PLL lock to avoid releases before PLL stable
    assign Rst_n = rst_n & lock;
    assign Rst   = ~Rst_n;

    // ------------------------------------------------------------------
    // app_wrrd: converts incoming sdr_data (from e.g. SD card or camera writer)
    // into the sdram application interface (App_wr_*/App_rd_*). It also
    // provides read data path Sdr_rd_dout and read enable Sdr_rd_en.
    // Keep clk domain for app_wrrd on ext_mem_clk (mem domain).
    // sd_clk is passed to app_wrrd for any SD side timing (preserved).
    // ------------------------------------------------------------------
    app_wrrd u1_app_wrrd(
        .clk                (ext_mem_clk),          // mem domain clock
        .sd_clk             (sd_clk),               // sd clock (kept)
        .rst_n              (Rst_n),
        .full_flag_net      (full_flag),
        .Sdr_init_done      (Sdr_init_done),
        .Sdr_init_ref_vld   (Sdr_init_ref_vld),
        .sdr_data_valid     (sdr_data_valid),
        .sdr_data           (sdr_data),
        .App_wr_en          (App_wr_en),
        .App_wr_addr        (App_wr_addr),
        .App_wr_dm          (App_wr_dm),
        .App_wr_din         (App_wr_din),
        .wr_done            (wr_done),
        .App_rd_en          (App_rd_en),
        .App_rd_addr        (App_rd_addr),
        .Sdr_rd_en          (Sdr_rd_en),
        .Sdr_rd_dout        (Sdr_rd_dout),
        .Sdr_busy           (Sdr_busy),
        .full_flag          (full_flag_sdr),
        .udp_wrusedw        (udp_wrusedw)
        // .Check_ok(...)
    );

    // ------------------------------------------------------------------
    // sdr_as_ram instance: vendor / project provided SDRAM controller wrapper
    // Uses ext_mem_clk as Sdr_clk and Clk_sft from PLL for the "sft" clock path.
    // ------------------------------------------------------------------
    sdr_as_ram #(.self_refresh_open(1'b1))
    u2_ram(
        .Sdr_clk            (ext_mem_clk),
        .Sdr_clk_sft        (Clk_sft),
        .Rst                (!Rst_n),

        .Sdr_init_done      (Sdr_init_done),
        .Sdr_init_ref_vld   (Sdr_init_ref_vld),
        .Sdr_busy           (Sdr_busy),

        .App_ref_req        (1'b0),

        .App_wr_en          (App_wr_en),
        .App_wr_addr        (App_wr_addr),
        .App_wr_dm          (App_wr_dm),
        .App_wr_din         (App_wr_din),

        .App_rd_en          (App_rd_en),
        .App_rd_addr        (App_rd_addr),
        .Sdr_rd_en          (Sdr_rd_en),
        .Sdr_rd_dout        (Sdr_rd_dout),

        .SDRAM_CLK          (SDRAM_CLK),
        .SDR_RAS            (SDR_RAS),
        .SDR_CAS            (SDR_CAS),
        .SDR_WE             (SDR_WE),
        .SDR_BA             (SDR_BA),
        .SDR_ADDR           (SDR_ADDR),
        .SDR_DM             (SDR_DM),
        .SDR_DQ             (SDR_DQ)
    );

    // Keep SDRAM CKE enabled
    assign SDR_CKE = 1'b1;

`ifndef SIMULATION
    EG_PHY_SDRAM_2M_32 sdram_phy (
        .clk    (SDRAM_CLK),
        .ras_n  (SDR_RAS),
        .cas_n  (SDR_CAS),
        .we_n   (SDR_WE),
        .addr   (SDR_ADDR[10:0]),
        .ba     (SDR_BA),
        .dq     (SDR_DQ),
        .cs_n   (1'b0),
        .dm0    (SDR_DM[0]),
        .dm1    (SDR_DM[1]),
        .dm2    (SDR_DM[2]),
        .dm3    (SDR_DM[3]),
        .cke    (1'b1)
    );
`endif

endmodule