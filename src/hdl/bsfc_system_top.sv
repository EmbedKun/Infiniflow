// SPDX-License-Identifier: BSD-2-Clause-Views
// Top Level Integration Wrapper with Dual CMAC & CDC Async FIFOs
// - Architecture: Asynchronous Mode
// - User Logic: Runs on 300 MHz (Derived from sys_clk MMCM)
// - CMAC Logic: Runs on 322 MHz (gt_txusrclk2)
// - Crossing: xpm_fifo_async used for all data paths.
// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none

module bsfc_system_top #
(
    // ---------------------------------------------------------
    // Global Configuration
    // ---------------------------------------------------------
    parameter QUEUE_INDEX_WIDTH = 16,
    parameter DATA_WIDTH        = 512,
    parameter FCP_AXIS_WIDTH    = 512, 
    
    // Injector Params
    parameter REQ_TAG_WIDTH     = 8,
    parameter LEN_WIDTH         = 16,
    parameter OP_TABLE_SIZE     = 16,
    parameter PKT_LEN_BYTES     = 1536,
    parameter PIPELINE          = 3+(QUEUE_INDEX_WIDTH > 12 ? QUEUE_INDEX_WIDTH-12 : 0),
    
    // Switch Params
    parameter ACTIVE_FIFO_DEPTH = 1024,
    parameter BUFFER_ADDR_WIDTH = 10,
    parameter STAT_WIDTH        = 32,
    parameter DRAIN_RATIO_M     = 4,
    parameter QMAX              = 6,
    parameter QMIN              = 3,
    parameter IGNORE_FCP_MODE   = 2
)
(
    input  wire sys_clk_p,      // 100MHz Diff Input
    input  wire sys_clk_n,
    input  wire sys_rst_n,      // Global Reset

    // ---------------------------------------------------------
    // CMAC 0 (Upstream Node)
    // ---------------------------------------------------------
    input  wire gt0_ref_clk_p, gt0_ref_clk_n,
    input  wire [3:0] gt0_rxp_in, gt0_rxn_in,
    output wire [3:0] gt0_txp_out, gt0_txn_out,

    // ---------------------------------------------------------
    // CMAC 1 (Downstream Node)
    // ---------------------------------------------------------
    input  wire gt1_ref_clk_p, gt1_ref_clk_n,
    input  wire [3:0] gt1_rxp_in, gt1_rxn_in,
    output wire [3:0] gt1_txp_out, gt1_txn_out
);

    // ========================================================================
    // 1. Clock Generation (MMCM: 100M -> 300M, 125M)
    // ========================================================================
    wire clk_100m_raw;
    wire init_clk;      // 125 MHz (CMAC Init)
    wire user_clk;      // 300 MHz (User Logic)
    wire sys_clk;       // 100 MHz (Debug)
    wire mmcm_locked;
    wire sys_rst_async; // Async Reset from pin
    
    IBUFDS sys_clk_ibufds (.I(sys_clk_p), .IB(sys_clk_n), .O(clk_100m_raw));

    wire clk_fb_in, clk_fb_out;
    wire clk_125m_out, clk_300m_out, clk_100m_out;

    // VCO = 1500 MHz (100 * 15)
    // Out0 = 1500 / 12 = 125 MHz
    // Out1 = 1500 / 5  = 300 MHz
    // Out2 = 1500 / 15 = 100 MHz
    MMCME4_BASE #(
        .BANDWIDTH("OPTIMIZED"), .STARTUP_WAIT("FALSE"),
        .CLKIN1_PERIOD(10.0), 
        .CLKFBOUT_MULT_F(10.0), .DIVCLK_DIVIDE(1),
        .CLKOUT0_DIVIDE_F(8.0),
        .CLKOUT1_DIVIDE(4),
        .CLKOUT2_DIVIDE(10)
    ) u_mmcm_gen (
        .CLKIN1(clk_100m_raw), .CLKFBIN(clk_fb_in), .CLKFBOUT(clk_fb_out),
        .CLKOUT0(clk_125m_out), .CLKOUT1(clk_300m_out), .CLKOUT2(clk_100m_out),
        .LOCKED(mmcm_locked), .RST(~sys_rst_n), .PWRDWN(1'b0),
        .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6()
    );

    BUFG u_bufg_init (.I(clk_125m_out), .O(init_clk)); 
    BUFG u_bufg_user (.I(clk_300m_out), .O(user_clk)); // 300 MHz for Logic
    BUFG u_bufg_sys  (.I(clk_100m_out), .O(sys_clk));  
    BUFG u_bufg_fb   (.I(clk_fb_out),   .O(clk_fb_in));

    assign sys_rst_async = ~sys_rst_n || !mmcm_locked;

    // ========================================================================
    // 2. CMAC Instantiations (Running on 322 MHz)
    // ========================================================================
    // Common CMAC signals
    wire gt0_txusrclk2, gt0_rx_clk;
    wire gt1_txusrclk2, gt1_rx_clk;
    wire gt0_usr_tx_reset, gt0_usr_rx_reset;
    wire gt1_usr_tx_reset, gt1_usr_rx_reset;
    wire stat0_rx_aligned, stat1_rx_aligned;
    wire stat0_tx_total_packets, stat1_rx_total_packets;

    // CMAC0 Interfaces
    wire [511:0] c0_tx_tdata, c0_rx_tdata;
    wire [63:0]  c0_tx_tkeep, c0_rx_tkeep;
    wire c0_tx_tvalid, c0_tx_tlast, c0_tx_tready;
    wire c0_rx_tvalid, c0_rx_tlast;

    // CMAC1 Interfaces
    wire [511:0] c1_tx_tdata, c1_rx_tdata;
    wire [63:0]  c1_tx_tkeep, c1_rx_tkeep;
    wire c1_tx_tvalid, c1_tx_tlast, c1_tx_tready;
    wire c1_rx_tvalid, c1_rx_tlast;

    // Clock Assignments
    assign gt0_rx_clk = gt0_txusrclk2; 
    assign gt1_rx_clk = gt1_txusrclk2; 

    // --- CMAC 0 (Upstream) ---
    cmac_usplus_0 u_cmac_0 (
        .gt_ref_clk_p(gt0_ref_clk_p), .gt_ref_clk_n(gt0_ref_clk_n),
        .gt_rxp_in(gt0_rxp_in), .gt_rxn_in(gt0_rxn_in),
        .gt_txp_out(gt0_txp_out), .gt_txn_out(gt0_txn_out),
        .init_clk(init_clk),
        .gt_txusrclk2(gt0_txusrclk2), .rx_clk(gt0_rx_clk),
        .sys_reset(sys_rst_async),
        .usr_tx_reset(gt0_usr_tx_reset), .usr_rx_reset(gt0_usr_rx_reset),
        .gtwiz_reset_tx_datapath(1'b0), .gtwiz_reset_rx_datapath(1'b0),
        .ctl_tx_enable(1'b1), .ctl_rx_enable(1'b1), .gt_loopback_in(12'h000),
        
        .tx_axis_tready(c0_tx_tready), .tx_axis_tvalid(c0_tx_tvalid),
        .tx_axis_tdata(c0_tx_tdata), .tx_axis_tlast(c0_tx_tlast), .tx_axis_tkeep(c0_tx_tkeep),
        .rx_axis_tvalid(c0_rx_tvalid), .rx_axis_tdata(c0_rx_tdata), .rx_axis_tlast(c0_rx_tlast), .rx_axis_tkeep(c0_rx_tkeep),
        .stat_rx_aligned(stat0_rx_aligned), .stat_tx_total_packets(stat0_tx_total_packets),
        .drp_clk(init_clk), .drp_addr(0), .drp_di(0), .drp_en(0), .drp_we(0)
    );

    // --- CMAC 1 (Downstream) ---
    cmac_usplus_1 u_cmac_1 (
        .gt_ref_clk_p(gt1_ref_clk_p), .gt_ref_clk_n(gt1_ref_clk_n),
        .gt_rxp_in(gt1_rxp_in), .gt_rxn_in(gt1_rxn_in),
        .gt_txp_out(gt1_txp_out), .gt_txn_out(gt1_txn_out),
        .init_clk(init_clk),
        .gt_txusrclk2(gt1_txusrclk2), .rx_clk(gt1_rx_clk),
        .sys_reset(sys_rst_async),
        .usr_tx_reset(gt1_usr_tx_reset), .usr_rx_reset(gt1_usr_rx_reset),
        .gtwiz_reset_tx_datapath(1'b0), .gtwiz_reset_rx_datapath(1'b0),
        .ctl_tx_enable(1'b1), .ctl_rx_enable(1'b1), .gt_loopback_in(12'h000),
        
        .tx_axis_tready(c1_tx_tready), .tx_axis_tvalid(c1_tx_tvalid),
        .tx_axis_tdata(c1_tx_tdata), .tx_axis_tlast(c1_tx_tlast), .tx_axis_tkeep(c1_tx_tkeep),
        .rx_axis_tvalid(c1_rx_tvalid), .rx_axis_tdata(c1_rx_tdata), .rx_axis_tlast(c1_rx_tlast), .rx_axis_tkeep(c1_rx_tkeep),
        .stat_rx_aligned(stat1_rx_aligned), .stat_rx_total_packets(stat1_rx_total_packets),
        .drp_clk(init_clk), .drp_addr(0), .drp_di(0), .drp_en(0), .drp_we(0)
    );

    // ========================================================================
    // 3. Rate Monitors (Run on CMAC 322MHz Domain)
    // ========================================================================
    localparam ONE_SEC_CYCLES = 322265625;
    wire [63:0] tx0_total_pkts, rx1_total_pkts;
    wire [31:0] tx0_pps, rx1_pps;

    // Reset for Rate Meter needs to be safe for 322M domain
    wire rst_322m_0 = sys_rst_async || gt0_usr_tx_reset;
    wire rst_322m_1 = sys_rst_async || gt1_usr_rx_reset;

    cmac_rate_meter u_meter_tx0 (.clk(gt0_txusrclk2), .rst(rst_322m_0), .stat_pulse(stat0_tx_total_packets), .cnt_total(tx0_total_pkts), .cnt_rate(tx0_pps));
    cmac_rate_meter u_meter_rx1 (.clk(gt1_txusrclk2), .rst(rst_322m_1), .stat_pulse(stat1_rx_total_packets), .cnt_total(rx1_total_pkts), .cnt_rate(rx1_pps));

    // ========================================================================
    // 4. User Logic Reset Generation (Sync to 300MHz)
    // ========================================================================
    reg [2:0] rst_user_sync_reg;
    wire rst_user_300;
    
    // Combine all potential reset sources (Async System Reset + CMAC Not Ready)
    // Logic should be held in reset if ANY CMAC is not ready.
    wire raw_reset_combine = sys_rst_async || gt0_usr_tx_reset || gt0_usr_rx_reset || gt1_usr_tx_reset || gt1_usr_rx_reset || !stat0_rx_aligned || !stat1_rx_aligned;

    always @(posedge user_clk) begin
        rst_user_sync_reg <= {rst_user_sync_reg[1:0], raw_reset_combine};
    end
    assign rst_user_300 = rst_user_sync_reg[2];

    // ========================================================================
    // 5. Upstream Logic (Injector) - Domain: user_clk (300MHz)
    // ========================================================================
    
    // Enable Logic
    reg enable_up;
    reg [23:0] up_cnt;
    always @(posedge user_clk) begin
        if (rst_user_300) begin
            up_cnt <= 0; enable_up <= 0;
        end else begin
            if (up_cnt[23]) enable_up <= 1; else up_cnt <= up_cnt + 1;
        end
    end

    // --- Injector -> Async FIFO -> CMAC0 TX ---
    // Injector output (300M)
    wire [511:0] inj_tdata;
    wire [63:0]  inj_tkeep;
    wire         inj_tvalid, inj_tlast, inj_tready;
    wire         fifo_inj_full, fifo_inj_empty;

    // xpm_fifo_async: 300M (Write) -> 322M (Read)
    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(512), .WRITE_DATA_WIDTH(577), .READ_DATA_WIDTH(577),
        .READ_MODE("fwft"), .USE_ADV_FEATURES("0000"), .CDC_SYNC_STAGES(2),.FIFO_MEMORY_TYPE("ultra")
    ) u_fifo_inj_c0 (
        .rst(rst_user_300), 
        .wr_clk(user_clk),
        .din({inj_tlast, inj_tkeep, inj_tdata}),
        .wr_en(inj_tvalid), .full(fifo_inj_full),
        
        .rd_clk(gt0_txusrclk2),
        .dout({c0_tx_tlast, c0_tx_tkeep, c0_tx_tdata}),
        .rd_en(c0_tx_tready), .empty(fifo_inj_empty)
    );
    assign inj_tready = ~fifo_inj_full;
    assign c0_tx_tvalid = ~fifo_inj_empty;

    // --- CMAC0 RX -> Async FIFO -> Unpacker ---
    // CMAC0 RX (322M) -> FIFO -> Unpacker (300M)
    wire [511:0] unp_fifo_data;
    wire         unp_fifo_valid;
    wire         fifo_c0_empty;
    
    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(512), .WRITE_DATA_WIDTH(512), .READ_DATA_WIDTH(512),
        .READ_MODE("fwft"), .USE_ADV_FEATURES("0000"), .CDC_SYNC_STAGES(2)
    ) u_fifo_c0_unp (
        .rst(gt0_usr_rx_reset), // Reset from Write Domain
        .wr_clk(gt0_txusrclk2), // CMAC RX is on TX clock
        .din(c0_rx_tdata), 
        .wr_en(c0_rx_tvalid),
        
        .rd_clk(user_clk),
        .dout(unp_fifo_data),
        .rd_en(1'b1), // Unpacker always ready
        .empty(fifo_c0_empty), .full()
    );
    assign unp_fifo_valid = ~fifo_c0_empty;

    // FCP Unpacker (Combinatorial/Logic on 300MHz)
    wire us_fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] us_fcp_vc;
    wire [STAT_WIDTH-1:0] us_fcp_fccl, us_fcp_qlen, us_fcp_fccr;

    fcp_sink_adapter #(.AXIS_WIDTH(DATA_WIDTH)) u_fcp_unpacker (
        .s_axis_fcp_tdata(unp_fifo_data), .s_axis_fcp_tvalid(unp_fifo_valid), .s_axis_fcp_tready(),
        .fcp_valid(us_fcp_valid), .fcp_vc(us_fcp_vc),
        .fcp_fccl(us_fcp_fccl), .fcp_qlen(us_fcp_qlen), .fcp_fccr(us_fcp_fccr)
    );

    // Injector Instance (300MHz)
    wire [63:0] inj_tx_cnt;
    massive_traffic_injector #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH), .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .LEN_WIDTH(LEN_WIDTH), .OP_TABLE_SIZE(OP_TABLE_SIZE), .PIPELINE(PIPELINE),
        .DATA_WIDTH(DATA_WIDTH), .PKT_LEN_BYTES(PKT_LEN_BYTES),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH), .QMAX(QMAX), .QMIN(QMIN), .IGNORE_FCP_MODE(IGNORE_FCP_MODE)
    ) u_upstream_logic (
        .clk(user_clk), .rst(rst_user_300), .enable(enable_up),
        .stop_queue_idx(0), .stop_cmd_valid(0),
        .m_axis_pkt_tdata(inj_tdata), .m_axis_pkt_tvalid(inj_tvalid),
        .m_axis_pkt_tlast(inj_tlast), .m_axis_pkt_tkeep(inj_tkeep),
        .m_axis_pkt_tready(inj_tready),
        .fcp_valid(us_fcp_valid), .fcp_vc(us_fcp_vc),
        .fcp_fccl(us_fcp_fccl), .fcp_qlen(us_fcp_qlen), .fcp_fccr(us_fcp_fccr),
        .m_axis_tx_pkt_count(inj_tx_cnt)
    );

    // ========================================================================
    // 6. Downstream Logic (Switch) - Domain: user_clk (300MHz)
    // ========================================================================

    // --- CMAC1 RX -> Async FIFO -> Switch ---
    // CMAC1 RX (322M) -> FIFO -> Switch (300M)
    wire [511:0] sw_rx_tdata;
    wire [63:0]  sw_rx_tkeep;
    wire         sw_rx_tvalid, sw_rx_tlast, sw_rx_tready;
    wire         fifo_c1_empty;
    
    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(512), .WRITE_DATA_WIDTH(577), .READ_DATA_WIDTH(577),
        .READ_MODE("fwft"), .USE_ADV_FEATURES("0000"), .CDC_SYNC_STAGES(2),.FIFO_MEMORY_TYPE("ultra")
    ) u_fifo_c1_sw (
        .rst(gt1_usr_rx_reset), // Reset from Write Domain
        .wr_clk(gt1_txusrclk2),
        .din({c1_rx_tlast, c1_rx_tkeep, c1_rx_tdata}),
        .wr_en(c1_rx_tvalid),
        
        .rd_clk(user_clk),
        .dout({sw_rx_tlast, sw_rx_tkeep, sw_rx_tdata}),
        .rd_en(sw_rx_tready), .empty(fifo_c1_empty), .full()
    );
    assign sw_rx_tvalid = ~fifo_c1_empty;

    // FCP Output Wires
    wire ds_fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] ds_fcp_vc;
    wire [STAT_WIDTH-1:0] ds_fcp_fccl, ds_fcp_qlen, ds_fcp_fccr;

    // Switch Model (300MHz)
    downstream_switch_model_v2 #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH), .DATA_WIDTH(DATA_WIDTH),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH), .STAT_WIDTH(STAT_WIDTH), .DRAIN_RATIO_M(DRAIN_RATIO_M)
    ) u_downstream_logic (
        .clk(user_clk), .rst(rst_user_300),
        .s_axis_pkt_tdata(sw_rx_tdata), .s_axis_pkt_tvalid(sw_rx_tvalid),
        .s_axis_pkt_tlast(sw_rx_tlast), .s_axis_pkt_tkeep(sw_rx_tkeep),
        .s_axis_pkt_tready(sw_rx_tready),
        .m_axis_pkt_tdata(), .m_axis_pkt_tvalid(),
        .fcp_valid(ds_fcp_valid), .fcp_vc(ds_fcp_vc),
        .fcp_fccl(ds_fcp_fccl), .fcp_qlen(ds_fcp_qlen), .fcp_fccr(ds_fcp_fccr)
    );

    // --- Packer -> Async FIFO -> CMAC1 TX ---
    // Packer (Comb) -> FIFO -> CMAC1 TX (322M)
    wire [511:0] pk_tdata;
    wire         pk_tvalid, pk_tready;
    wire         fifo_pk_full, fifo_pk_empty;

    fcp_source_adapter #(.AXIS_WIDTH(DATA_WIDTH)) u_fcp_packer (
        .fcp_valid(ds_fcp_valid), .fcp_vc(ds_fcp_vc),
        .fcp_fccl(ds_fcp_fccl), .fcp_qlen(ds_fcp_qlen), .fcp_fccr(ds_fcp_fccr),
        .m_axis_fcp_tdata(pk_tdata), .m_axis_fcp_tvalid(pk_tvalid), .m_axis_fcp_tready(pk_tready)
    );

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(512), .WRITE_DATA_WIDTH(512), .READ_DATA_WIDTH(512),
        .READ_MODE("fwft"), .USE_ADV_FEATURES("0000"), .CDC_SYNC_STAGES(2)
    ) u_fifo_pk_c1 (
        .rst(rst_user_300), 
        .wr_clk(user_clk),
        .din(pk_tdata),
        .wr_en(pk_tvalid), .full(fifo_pk_full),
        
        .rd_clk(gt1_txusrclk2),
        .dout(c1_tx_tdata),
        .rd_en(c1_tx_tready), .empty(fifo_pk_empty)
    );
    assign pk_tready   = ~fifo_pk_full;
    assign c1_tx_tvalid = ~fifo_pk_empty;
    assign c1_tx_tkeep  = {64{1'b1}};
    assign c1_tx_tlast  = 1'b1;

    // ========================================================================
    // 8. ILA (Debug)
    // ========================================================================
    // Note: ILA must be clocked by the domain it probes. 
    
    wire [31:0] tx0_pps_sync;
    wire [31:0] rx1_pps_sync;
    reg         gt_clk_heartbeat = 0;
    
    // A. Clock Heartbeat Generation (To verify GT clock is alive)
    // Runs on gt0_txusrclk2, captured by ILA on user_clk
    always @(posedge gt0_txusrclk2) begin
        gt_clk_heartbeat <= ~gt_clk_heartbeat;
    end

    // B. Statistics Synchronization (PPS is quasi-static, 1 update/sec)
    xpm_cdc_array_single #(
        .DEST_SYNC_FF(2), .WIDTH(32), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0)
    ) u_sync_tx_pps (
        .src_clk(gt0_txusrclk2), .src_in(tx0_pps),
        .dest_clk(user_clk),     .dest_out(tx0_pps_sync)
    );

    xpm_cdc_array_single #(
        .DEST_SYNC_FF(2), .WIDTH(32), .SIM_ASSERT_CHK(0), .SRC_INPUT_REG(0)
    ) u_sync_rx_pps (
        .src_clk(gt1_txusrclk2), .src_in(rx1_pps),
        .dest_clk(user_clk),     .dest_out(rx1_pps_sync)
    );

    // ---------------------------------------------------------
    // 8.2 ILA Instantiation
    // ---------------------------------------------------------
    // IMPORTANT: Make sure to generate an ILA IP in Vivado with:
    // - Name: ila_0
    
    ila_0 u_ila_debug (
        .clk(user_clk), // ILA runs on 300MHz User Clock

        // [Probe 0] FCP Upstream Protocol (Grouped for readability)
        .probe0 ({us_fcp_valid, us_fcp_vc, us_fcp_fccr, us_fcp_fccl}), 
        // Bit Mapping Suggestion:
        // [79:48] FCCL (32b)
        // [47:16] FCCR (32b)
        // [15:0]  VC   (16b)
        // [80]    Valid(1b) -> Total 81 bits (Adjust ILA width to 81 or round up to 88/96)

        // [Probe 1] Statistics & Heartbeats
        .probe1 (tx0_pps_sync),     // [31:0] CMAC0 TX Speed (Pkts/Sec)
        .probe2 (rx1_pps_sync),     // [31:0] CMAC1 RX Speed (Pkts/Sec)
        .probe3 (gt_clk_heartbeat), // [0:0]  Toggling means GT Clock is alive
        
        // [Probe 2] User Logic Status
        .probe4 (rst_user_300),     // [0:0]  User Reset Status (Should be 0)
        .probe5 (inj_tx_cnt),       // [63:0] Injection Counter (Should increment)
        
        // [Probe 3] Data Path (Downstream Switch Input)
        // WARNING: 512-bit wide probe consumes massive BRAM resources!
        // If timing fails, consider probing only the lower 64 bits.
        .probe6 (sw_rx_tdata),      // [511:0] Switch RX Data
        .probe7 (sw_rx_tvalid),     // [0:0]   Switch RX Valid
        .probe8 (sw_rx_tready)      // [0:0]   Switch RX Ready
    );
endmodule