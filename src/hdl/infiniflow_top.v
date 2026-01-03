// SPDX-License-Identifier: BSD-2-Clause-Views
// Top Level Integration Wrapper with Dual CMAC and CDC
// Author: mkxue-FNIL
// Update: Data Width aligned to 512-bit (CMAC Native Width)
// Line Rate Version: Core Logic : 300MHz, 2^16 VCs

`timescale 1ns / 1ps
`default_nettype none

module infiniflow_top #
(
    // ---------------------------------------------------------
    // Global Configuration
    // ---------------------------------------------------------
    parameter QUEUE_INDEX_WIDTH = 16,
    
    // [CHANGE] Aligned User Logic to CMAC Width (512-bit)
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
    parameter DRAIN_RATIO_M     = 8,
    parameter QMAX              = 6,
    parameter QMIN              = 3,
    parameter IGNORE_FCP_MODE   = 1   //Debug Used
)
(
    // System Clock (Input 100MHz Differential)
    input  wire sys_clk_p,
    input  wire sys_clk_n,
    input  wire sys_rst_n,

    // ---------------------------------------------------------
    // CMAC 0 (Upstream Node) Physical Interface
    // ---------------------------------------------------------
    input  wire gt0_ref_clk_p,
    input  wire gt0_ref_clk_n,
    input  wire [3:0] gt0_rxp_in,
    input  wire [3:0] gt0_rxn_in,
    output wire [3:0] gt0_txp_out,
    output wire [3:0] gt0_txn_out,

    // ---------------------------------------------------------
    // CMAC 1 (Downstream Node) Physical Interface
    // ---------------------------------------------------------
    input  wire gt1_ref_clk_p,
    input  wire gt1_ref_clk_n,
    input  wire [3:0] gt1_rxp_in,
    input  wire [3:0] gt1_rxn_in,
    output wire [3:0] gt1_txp_out,
    output wire [3:0] gt1_txn_out
);

    // ========================================================================
    // 1. Clock & Reset Management (Robust Architecture)
    // ========================================================================
    
    // Raw Input Clock (Only drives MMCM)
    wire clk_100m_raw;
    IBUFDS sys_clk_ibufds (.I(sys_clk_p), .IB(sys_clk_n), .O(clk_100m_raw));
    
    // Generated Clocks
    wire sys_clk;     // 100MHz (Clean User Logic Clock from MMCM)
    wire init_clk;    // 125MHz (For CMAC DRP/Init)
    wire sys_rst;     // Active High Reset
    wire mmcm_locked;

    assign sys_rst = ~sys_rst_n;

    // 1.1 MMCM: Generate 125MHz and 100MHz
    wire clk_mmcm_feedback_in, clk_mmcm_feedback_out;
    wire init_clk_unbuffered, sys_clk_unbuffered;

    MMCME4_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .STARTUP_WAIT("FALSE"),
        .CLKIN1_PERIOD(10.0),    // Input: 100 MHz
        .CLKFBOUT_MULT_F(15.0),  // VCO = 1500 MHz
        .DIVCLK_DIVIDE(1),
        .CLKOUT0_DIVIDE_F(12.0),  // 125 MHz
        .CLKOUT1_DIVIDE(5),     // 300 MHz
        .CLKOUT0_DUTY_CYCLE(0.5), .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DUTY_CYCLE(0.5), .CLKOUT1_PHASE(0.0)
    ) u_mmcm_gen (
        .CLKIN1(clk_100m_raw),
        .CLKFBIN(clk_mmcm_feedback_in), 
        .CLKOUT0(init_clk_unbuffered),  // 125M
        .CLKOUT1(sys_clk_unbuffered),   // 300M
        .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(), 
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        .CLKFBOUT(clk_mmcm_feedback_out),
        .CLKFBOUTB(),
        .LOCKED(mmcm_locked),
        .RST(sys_rst),
        .PWRDWN(1'b0)
    );

    BUFG u_bufg_init (.I(init_clk_unbuffered), .O(init_clk));
    BUFG u_bufg_sys  (.I(sys_clk_unbuffered),  .O(sys_clk));  
    BUFG u_bufg_fb   (.I(clk_mmcm_feedback_out), .O(clk_mmcm_feedback_in));
   
    // ========================================================================
    // 2. CMAC Signals Declaration
    // ========================================================================
    wire gt0_usr_tx_reset, gt0_usr_rx_reset;
    wire stat0_rx_aligned;
    wire gt1_usr_tx_reset, gt1_usr_rx_reset;
    wire stat1_rx_aligned;

    // ========================================================================
    // 3. Safe Startup Logic & CDC Synchronization
    // ========================================================================
    wire [5:0] raw_status = {gt0_usr_tx_reset, gt0_usr_rx_reset, stat0_rx_aligned,
                             gt1_usr_tx_reset, gt1_usr_rx_reset, stat1_rx_aligned};
    wire [5:0] sync_status;

    xpm_cdc_array_single #(
        .DEST_SYNC_FF(2), .WIDTH(6), .SIM_ASSERT_CHK(0)
    ) u_sync_status (
        .src_clk(1'b0), 
        .src_in(raw_status),
        .dest_clk(sys_clk),
        .dest_out(sync_status)
    );

    wire system_ready;
    assign system_ready = mmcm_locked; // Simplified ready logic based on your request

    reg  enable_reg;
    wire enable_internal;
    reg [15:0] link_stable_cnt;

    always @(posedge sys_clk) begin
        if (sys_rst || !mmcm_locked) begin
            link_stable_cnt <= 0;
            enable_reg      <= 0;
        end else begin
            if (system_ready) begin
                if (link_stable_cnt == 16'hFFFF) begin
                    enable_reg <= 1'b1;
                end else begin
                    link_stable_cnt <= link_stable_cnt + 1;
                    enable_reg      <= 1'b0;
                end
            end else begin
                link_stable_cnt <= 0;
                enable_reg      <= 1'b0; 
            end
        end
    end
    assign enable_internal = enable_reg;

    // ========================================================================
    // 4. User Logic Signal Declarations (System Clock Domain)
    // ========================================================================
    // NOTE: DATA_WIDTH is now 512
    
    // Injector Output (Data)
    wire [DATA_WIDTH-1:0]   inj_tdata;
    wire                    inj_tvalid;
    wire                    inj_tlast;
    wire [DATA_WIDTH/8-1:0] inj_tkeep;
    wire                    inj_tready; 

    // Switch Input (Data)
    wire [DATA_WIDTH-1:0]   sw_tdata;
    wire                    sw_tvalid;
    wire                    sw_tlast;
    wire [DATA_WIDTH/8-1:0] sw_tkeep;
    wire                    sw_tready;

    // Discrete FCP Signals
    wire                      ds_fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] ds_fcp_vc;
    wire [STAT_WIDTH-1:0]     ds_fcp_fccl;
    wire [STAT_WIDTH-1:0]     ds_fcp_qlen;
    wire [STAT_WIDTH-1:0]     ds_fcp_fccr;
    
    wire                      us_fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] us_fcp_vc;
    wire [STAT_WIDTH-1:0]     us_fcp_fccl;
    wire [STAT_WIDTH-1:0]     us_fcp_qlen;
    wire [STAT_WIDTH-1:0]     us_fcp_fccr;

    // Adapter Signals
    wire [FCP_AXIS_WIDTH-1:0] packer_tdata;
    wire                      packer_tvalid;
    wire                      packer_tready;

    wire [FCP_AXIS_WIDTH-1:0] unpacker_tdata;
    wire                      unpacker_tvalid;
    wire                      unpacker_tready;

    // Debug Signals
    wire [63:0] m_axis_tx_pkt_count;
    wire [STAT_WIDTH-1:0] dbg_buffer_free_count;
    wire [STAT_WIDTH-1:0] dbg_total_rx_count;

    // ========================================================================
    // 5. User Logic Instantiation
    // ========================================================================

    // A. Upstream: Traffic Injector
    massive_traffic_injector #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .LEN_WIDTH(LEN_WIDTH),
        .OP_TABLE_SIZE(OP_TABLE_SIZE),
        .PIPELINE(PIPELINE),
        .DATA_WIDTH(DATA_WIDTH),       // 512
        .PKT_LEN_BYTES(PKT_LEN_BYTES), // 64
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .QMAX(QMAX),
        .QMIN(QMIN),
        .IGNORE_FCP_MODE(IGNORE_FCP_MODE)
    ) upstream_inst (
        .clk(sys_clk),
        .rst(sys_rst),
        .enable(enable_internal),
        .m_axis_pkt_tdata(inj_tdata),
        .m_axis_pkt_tvalid(inj_tvalid),
        .m_axis_pkt_tlast(inj_tlast),
        .m_axis_pkt_tkeep(inj_tkeep),
        .m_axis_pkt_tready(inj_tready),
        .fcp_valid(us_fcp_valid),
        .fcp_vc(us_fcp_vc),
        .fcp_fccl(us_fcp_fccl),
        .fcp_fccr(us_fcp_fccr),
        .fcp_qlen(us_fcp_qlen),
        .m_axis_tx_pkt_count(m_axis_tx_pkt_count)
    );

    // B. Downstream: Switch Model
    downstream_switch_model_v2 #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),       // 512
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .STAT_WIDTH(STAT_WIDTH),
        .DRAIN_RATIO_M(DRAIN_RATIO_M)
    ) downstream_inst (
        .clk(sys_clk),
        .rst(sys_rst),
        .s_axis_pkt_tdata(sw_tdata),
        .s_axis_pkt_tvalid(sw_tvalid),
        .s_axis_pkt_tlast(sw_tlast),
        .s_axis_pkt_tkeep(sw_tkeep),
        .s_axis_pkt_tready(sw_tready),
        .fcp_valid(ds_fcp_valid),
        .fcp_vc(ds_fcp_vc),
        .fcp_fccl(ds_fcp_fccl),
        .fcp_qlen(ds_fcp_qlen),
        .fcp_fccr(ds_fcp_fccr),
        .dbg_buffer_free_count(dbg_buffer_free_count),
        .dbg_total_rx_count(dbg_total_rx_count)
    );

    // C. Adapters
    fcp_source_adapter #(.AXIS_WIDTH(FCP_AXIS_WIDTH)) fcp_packer (
        .fcp_valid(ds_fcp_valid),
        .fcp_vc(ds_fcp_vc),
        .fcp_fccl(ds_fcp_fccl),
        .fcp_qlen(ds_fcp_qlen),
        .fcp_fccr(ds_fcp_fccr),
        .m_axis_fcp_tdata(packer_tdata),
        .m_axis_fcp_tvalid(packer_tvalid),
        .m_axis_fcp_tready(packer_tready) 
    );

    fcp_sink_adapter #(.AXIS_WIDTH(FCP_AXIS_WIDTH)) fcp_unpacker (
        .s_axis_fcp_tdata(unpacker_tdata),
        .s_axis_fcp_tvalid(unpacker_tvalid),
        .s_axis_fcp_tready(unpacker_tready),
        .fcp_valid(us_fcp_valid),
        .fcp_vc(us_fcp_vc),
        .fcp_fccl(us_fcp_fccl), .fcp_qlen(us_fcp_qlen), .fcp_fccr(us_fcp_fccr)
    );

    // ========================================================================
    // 6. CDC FIFOs (Simplified for 512-bit to 512-bit Direct Map)
    // ========================================================================

    // CMAC Common Widths
    localparam CMAC_DATA_WIDTH = 512;
    localparam CMAC_KEEP_WIDTH = 64;

    // CMAC Wires
    wire gt0_txusrclk2, gt0_rx_clk;
    wire gt1_txusrclk2, gt1_rx_clk;

    // CMAC0 TX Interface
    wire [CMAC_DATA_WIDTH-1:0] c0_tx_tdata;
    wire [CMAC_KEEP_WIDTH-1:0] c0_tx_tkeep;
    wire                       c0_tx_tvalid, c0_tx_tlast, c0_tx_tready;

    // CMAC0 RX Interface
    wire [CMAC_DATA_WIDTH-1:0] c0_rx_tdata;
    wire [CMAC_KEEP_WIDTH-1:0] c0_rx_tkeep;
    wire                       c0_rx_tvalid, c0_rx_tlast;

    // CMAC1 TX Interface
    wire [CMAC_DATA_WIDTH-1:0] c1_tx_tdata;
    wire [CMAC_KEEP_WIDTH-1:0] c1_tx_tkeep;
    wire                       c1_tx_tvalid, c1_tx_tlast, c1_tx_tready;

    // CMAC1 RX Interface
    wire [CMAC_DATA_WIDTH-1:0] c1_rx_tdata;
    wire [CMAC_KEEP_WIDTH-1:0] c1_rx_tkeep;
    wire                       c1_rx_tvalid, c1_rx_tlast;


    // ------------------------------------------------------------------------
    // CDC 1: Injector (512b) -> CMAC 0 TX (512b)
    // [CHANGE] Direct FIFO, no packing logic needed
    // ------------------------------------------------------------------------
    // Width = 512 (Data) + 64 (Keep) + 1 (Last) = 577 bits
    wire [576:0] fifo1_din, fifo1_dout;
    wire         fifo1_wr_en, fifo1_full;
    wire         fifo1_rd_en, fifo1_empty;

    assign fifo1_din   = {inj_tlast, inj_tkeep, inj_tdata};
    assign fifo1_wr_en = inj_tvalid;
    assign inj_tready  = ~fifo1_full;

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("auto"),
        .FIFO_WRITE_DEPTH(512), // Depth adjusted
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(577),
        .READ_DATA_WIDTH(577),
        .USE_ADV_FEATURES("0000")
    ) u_fifo_inj_to_c0 (
        .rst(sys_rst),
        .wr_clk(sys_clk),
        .din(fifo1_din),
        .wr_en(fifo1_wr_en),
        .full(fifo1_full),
        .rd_clk(gt0_txusrclk2),
        .dout(fifo1_dout),
        .rd_en(fifo1_rd_en),
        .empty(fifo1_empty),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );

    assign {c0_tx_tlast, c0_tx_tkeep, c0_tx_tdata} = fifo1_dout;
    assign c0_tx_tvalid = ~fifo1_empty; 
    assign fifo1_rd_en  = c0_tx_tready & ~fifo1_empty;


    // ------------------------------------------------------------------------
    // CDC 2: CMAC 0 RX (512b) -> Unpacker (512b)
    // [CHANGE] Pass full width
    // ------------------------------------------------------------------------
    wire [576:0] fifo2_din, fifo2_dout;
    wire         fifo2_empty;
    
    // We pass TLAST/TKEEP just in case, even if unpacker mainly uses TDATA
    assign fifo2_din = {c0_rx_tlast, c0_rx_tkeep, c0_rx_tdata}; 

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(512),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(577),
        .READ_DATA_WIDTH(577),
        .USE_ADV_FEATURES("0000")
    ) u_fifo_c0_to_unpacker (
        .rst(sys_rst || gt0_usr_rx_reset),
        .wr_clk(gt0_txusrclk2),
        .din(fifo2_din),
        .wr_en(c0_rx_tvalid),
        .full(),
        .rd_clk(sys_clk),
        .dout(fifo2_dout),
        .rd_en(unpacker_tready),
        .empty(fifo2_empty),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );
    
    assign unpacker_tdata  = fifo2_dout[511:0];
    assign unpacker_tvalid = ~fifo2_empty;

    // ------------------------------------------------------------------------
    // CDC 3: Packer (512b) -> CMAC 1 TX (512b)
    // [CHANGE] Direct FIFO, no padding needed
    // ------------------------------------------------------------------------
    wire [511:0] fifo3_dout_data;
    wire         fifo3_empty;
    wire         fifo3_rd_en;
    
    // Packer output is usually 512b data, but misses tkeep/tlast
    // We assume 1 beat per FCP packet, so we regenerate TLAST/TKEEP at CMAC side
    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(512),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(512),
        .READ_DATA_WIDTH(512),
        .USE_ADV_FEATURES("0000")
    ) u_fifo_packer_to_c1_direct (
        .rst(sys_rst),
        .wr_clk(sys_clk),
        .din(packer_tdata),
        .wr_en(packer_tvalid),
        .full(),
        .rd_clk(gt1_txusrclk2),
        .dout(fifo3_dout_data),
        .rd_en(fifo3_rd_en),
        .empty(fifo3_empty),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );

    assign c1_tx_tdata  = fifo3_dout_data;
    assign c1_tx_tkeep  = {64{1'b1}}; // Keep all bytes valid
    assign c1_tx_tlast  = 1'b1;       // Always single beat for FCP
    assign c1_tx_tvalid = ~fifo3_empty;
    assign fifo3_rd_en  = c1_tx_tready & ~fifo3_empty;

    // ------------------------------------------------------------------------
    // CDC 4: CMAC 1 RX (512b) -> Switch (512b)
    // [CHANGE] Direct FIFO, removed chunking logic
    // ------------------------------------------------------------------------
    wire [576:0] fifo4_din, fifo4_dout;
    wire         fifo4_rd_en, fifo4_empty;

    assign fifo4_din = {c1_rx_tlast, c1_rx_tkeep, c1_rx_tdata};

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(512),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(577),
        .READ_DATA_WIDTH(577),
        .USE_ADV_FEATURES("0000")
    ) u_fifo_c1_to_sw (
        .rst(sys_rst || gt1_usr_rx_reset),
        .wr_clk(gt1_txusrclk2),
        .din(fifo4_din),
        .wr_en(c1_rx_tvalid),
        .full(),
        .rd_clk(sys_clk),
        .dout(fifo4_dout),
        .rd_en(fifo4_rd_en),
        .empty(fifo4_empty),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );

    assign {sw_tlast, sw_tkeep, sw_tdata} = fifo4_dout;
    assign sw_tvalid   = ~fifo4_empty;
    assign fifo4_rd_en = sw_tready & ~fifo4_empty;
    
    wire [7:0] gt_powergoodout;
    wire stat0_tx_total_packets, stat1_rx_total_packets;
    
    // ========================================================================
    // 7. CMAC Instantiations (UNCHANGED)
    // ========================================================================
    cmac_usplus_0 u_cmac_0 (
        .gt_rxp_in(gt0_rxp_in), .gt_rxn_in(gt0_rxn_in),
        .gt_txp_out(gt0_txp_out), .gt_txn_out(gt0_txn_out),
        .gt_ref_clk_p(gt0_ref_clk_p), .gt_ref_clk_n(gt0_ref_clk_n),
        .init_clk(init_clk),
        .gt_txusrclk2(gt0_txusrclk2),
        .rx_clk(gt0_txusrclk2),
        .gt_ref_clk_out(),
        
        .sys_reset(sys_rst),
        .gtwiz_reset_tx_datapath(1'b0), .gtwiz_reset_rx_datapath(1'b0),
        .usr_tx_reset(gt0_usr_tx_reset), .usr_rx_reset(gt0_usr_rx_reset),
        .core_tx_reset(1'b0), .core_rx_reset(1'b0), .core_drp_reset(1'b0),
        
        .ctl_tx_enable(1'b1), .ctl_rx_enable(1'b1),
        .ctl_tx_send_idle(1'b0), .ctl_tx_send_rfi(1'b0), .ctl_tx_send_lfi(1'b0),
        .ctl_tx_test_pattern(1'b0), .ctl_rx_test_pattern(1'b0), .ctl_rx_force_resync(1'b0),
        .gt_powergoodout(gt_powergoodout[3:0]),
        .gt_loopback_in(12'h000),
        
        // TX
        .tx_axis_tready(c0_tx_tready),
        .tx_axis_tvalid(c0_tx_tvalid),
        .tx_axis_tdata(c0_tx_tdata),
        .tx_axis_tlast(c0_tx_tlast),
        .tx_axis_tkeep(c0_tx_tkeep),
        .tx_axis_tuser(1'b0), .tx_ovfout(), .tx_unfout(), .tx_preamblein(56'd0),
        
        // RX
        .rx_axis_tvalid(c0_rx_tvalid),
        .rx_axis_tdata(c0_rx_tdata),
        .rx_axis_tlast(c0_rx_tlast),
        .rx_axis_tkeep(c0_rx_tkeep),
        .rx_axis_tuser(), .rx_preambleout(),
        
        .stat_rx_aligned(stat0_rx_aligned),
        .stat_rx_status(),
        .stat_tx_total_packets(stat0_tx_total_packets),
        
        .drp_clk(init_clk), .drp_addr(10'b0), .drp_di(16'b0), .drp_en(1'b0), .drp_we(1'b0), .drp_do(), .drp_rdy()
    );

    cmac_usplus_1 u_cmac_1 (
        .gt_rxp_in(gt1_rxp_in), .gt_rxn_in(gt1_rxn_in),
        .gt_txp_out(gt1_txp_out), .gt_txn_out(gt1_txn_out),
        .gt_ref_clk_p(gt1_ref_clk_p), .gt_ref_clk_n(gt1_ref_clk_n),
        .init_clk(init_clk),
        .gt_txusrclk2(gt1_txusrclk2),
        .rx_clk(gt1_txusrclk2),
        .gt_ref_clk_out(),
        
        .sys_reset(sys_rst),
        .gtwiz_reset_tx_datapath(1'b0), .gtwiz_reset_rx_datapath(1'b0),
        .usr_tx_reset(gt1_usr_tx_reset), .usr_rx_reset(gt1_usr_rx_reset),
        .core_tx_reset(1'b0), .core_rx_reset(1'b0), .core_drp_reset(1'b0),
        
        .ctl_tx_enable(1'b1), .ctl_rx_enable(1'b1),
        .ctl_tx_send_idle(1'b0), .ctl_tx_send_rfi(1'b0), .ctl_tx_send_lfi(1'b0),
        .ctl_tx_test_pattern(1'b0), .ctl_rx_test_pattern(1'b0), .ctl_rx_force_resync(1'b0),
        .gt_powergoodout(gt_powergoodout[7:4]),
        .gt_loopback_in(12'h000),
        
        // TX
        .tx_axis_tready(c1_tx_tready),
        .tx_axis_tvalid(c1_tx_tvalid),
        .tx_axis_tdata(c1_tx_tdata),
        .tx_axis_tlast(c1_tx_tlast),
        .tx_axis_tkeep(c1_tx_tkeep),
        .tx_axis_tuser(1'b0), .tx_ovfout(), .tx_unfout(), .tx_preamblein(56'd0),
        
        // RX
        .rx_axis_tvalid(c1_rx_tvalid),
        .rx_axis_tdata(c1_rx_tdata),
        .rx_axis_tlast(c1_rx_tlast),
        .rx_axis_tkeep(c1_rx_tkeep),
        .rx_axis_tuser(), .rx_preambleout(),
        
        .stat_rx_aligned(stat1_rx_aligned),
        .stat_rx_status(),
        .stat_rx_total_packets(stat1_rx_total_packets),
        
        .drp_clk(init_clk), .drp_addr(10'b0), .drp_di(16'b0), .drp_en(1'b0), .drp_we(1'b0), .drp_do(), .drp_rdy()
    );


    // ========================================================================
    // 8. ILA Debug Core (System Clock Domain)
    // ========================================================================
    // [FIX] Probes sliced to 64-bit to prevent ILA width mismatch errors during synthesis.
    // If you regenerated ILA for 512-bit, you can remove the [63:0] slicing.
    
    ila_0 u_ila (
        .clk(sys_clk), 

        // Group 1: Data Path (SLICED to 64-bit for compatibility)
        .probe0 (inj_tdata[63:0]),  .probe1 (inj_tkeep[7:0]), .probe2 (inj_tlast), .probe3 (inj_tvalid), .probe4 (inj_tready),
        .probe5 (sw_tdata[63:0]),   .probe6 (sw_tkeep[7:0]),  .probe7 (sw_tlast),  .probe8 (sw_tvalid),  .probe9 (sw_tready),

        // Group 2: FCP Downstream
        .probe10(ds_fcp_valid), .probe11(ds_fcp_vc), .probe12(ds_fcp_qlen), .probe13(ds_fcp_fccl), .probe14(ds_fcp_fccr),

        // Group 3: FCP Upstream
        .probe15(us_fcp_valid), .probe16(us_fcp_vc), .probe17(us_fcp_qlen), .probe18(us_fcp_fccl), .probe19(us_fcp_fccr),

        // Group 4: Adapters
        .probe20(packer_tvalid), .probe21(stat1_rx_total_packets), .probe22(unpacker_tvalid), .probe23(enable_internal),

        // Group 5: Link Status
        .probe24(stat0_rx_aligned), .probe25(stat1_rx_aligned), .probe26(m_axis_tx_pkt_count), .probe27(dbg_buffer_free_count),
        
        // Group 6: Line rate Analagy
        .probe28(tx0_total_pkts), .probe29(tx0_pps), .probe30(rx1_total_pkts), .probe31(rx1_pps)
    );
    wire [63:0] tx0_total_pkts, rx1_total_pkts;
    wire [31:0] tx0_pps, rx1_pps;
    cmac_rate_meter u_meter_tx0 (.clk(gt0_txusrclk2), .rst(gt0_usr_tx_reset), .stat_pulse(stat0_tx_total_packets), .cnt_total(tx0_total_pkts), .cnt_rate(tx0_pps));
    cmac_rate_meter u_meter_rx1 (.clk(gt1_txusrclk2), .rst(gt1_usr_rx_reset), .stat_pulse(stat1_rx_total_packets), .cnt_total(rx1_total_pkts), .cnt_rate(rx1_pps));

endmodule
`resetall