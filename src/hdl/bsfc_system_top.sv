// SPDX-License-Identifier: BSD-2-Clause-Views
// Top Level Integration Wrapper with Dual CMAC and CDC
// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none

module bsfc_system_top #
(
    // ---------------------------------------------------------
    // Global Configuration
    // ---------------------------------------------------------
    parameter QUEUE_INDEX_WIDTH = 16,
    parameter DATA_WIDTH        = 64,  // User Logic Data Width
    parameter FCP_AXIS_WIDTH    = 128, // User Logic FCP Width
    
    // Injector Params
    parameter REQ_TAG_WIDTH     = 8,
    parameter LEN_WIDTH         = 16,
    parameter OP_TABLE_SIZE     = 16,
    parameter PKT_LEN_BYTES     = 64,
    parameter PIPELINE          = 3+(QUEUE_INDEX_WIDTH > 12 ? QUEUE_INDEX_WIDTH-12 : 0),
    
    // Switch Params
    parameter ACTIVE_FIFO_DEPTH = 1024,
    parameter BUFFER_ADDR_WIDTH = 10,
    parameter STAT_WIDTH        = 32,
    parameter DRAIN_RATIO_M     = 8,
    parameter QMAX              = 6,
    parameter QMIN              = 3,
    parameter IGNORE_FCP_MODE = 1   //Debug Used
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
    // input  wire init_clk, // [REMOVED] Generated internally
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
    // ------------------------------------------------------------------------
    wire clk_mmcm_feedback_in, clk_mmcm_feedback_out;
    wire init_clk_unbuffered, sys_clk_unbuffered;

    MMCME4_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .STARTUP_WAIT("FALSE"),
        .CLKIN1_PERIOD(10.0),    // Input: 100 MHz
        .CLKFBOUT_MULT_F(10.0),  // VCO = 1000 MHz (Valid Range: 600-1333 MHz)
        .DIVCLK_DIVIDE(1),
        
        // Channel 0: 125 MHz (1000 / 8.0)
        // Note: CLKOUT0 supports fractional divide (_F)
        .CLKOUT0_DIVIDE_F(8.0),  
        
        // Channel 1: 100 MHz (1000 / 10)
        // [FIX] CLKOUT1 only supports INTEGER divide in MMCME4_BASE
        .CLKOUT1_DIVIDE(10),     
        
        .CLKOUT0_DUTY_CYCLE(0.5), .CLKOUT0_PHASE(0.0),
        .CLKOUT1_DUTY_CYCLE(0.5), .CLKOUT1_PHASE(0.0)
    ) u_mmcm_gen (
        .CLKIN1(clk_100m_raw),
        .CLKFBIN(clk_mmcm_feedback_in), 
        
        .CLKOUT0(init_clk_unbuffered),  // 125M
        .CLKOUT1(sys_clk_unbuffered),   // 100M
        .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(), 
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        
        .CLKFBOUT(clk_mmcm_feedback_out),
        .CLKFBOUTB(),
        .LOCKED(mmcm_locked),
        .RST(sys_rst),
        .PWRDWN(1'b0)
    );

    // 1.2 Global Buffers
    BUFG u_bufg_init (.I(init_clk_unbuffered), .O(init_clk));
    BUFG u_bufg_sys  (.I(sys_clk_unbuffered),  .O(sys_clk));  // Use THIS for all user logic
    BUFG u_bufg_fb   (.I(clk_mmcm_feedback_out), .O(clk_mmcm_feedback_in));
   
    //12.27 19:12 Update
    reg [9:0] gt_rst_cnt = 0;
    reg       gt_reset_pulse = 0;
    
    always @(posedge sys_clk) begin
        if (sys_rst || !mmcm_locked) begin
            gt_rst_cnt <= 0;
            gt_reset_pulse <= 0;
        end else begin
            if (gt_rst_cnt < 10'h3FF) begin
                gt_rst_cnt <= gt_rst_cnt + 1;
                if (gt_rst_cnt > 100 && gt_rst_cnt < 200) 
                    gt_reset_pulse <= 1'b1;
                else 
                    gt_reset_pulse <= 1'b0;
            end
        end
    end
    
    // ========================================================================
    // 2. CMAC Signals Declaration (Needed for Startup Logic)
    // ========================================================================
    wire gt0_usr_tx_reset, gt0_usr_rx_reset;
    wire stat0_rx_aligned;
    wire gt1_usr_tx_reset, gt1_usr_rx_reset;
    wire stat1_rx_aligned;

    // ========================================================================
    // 3. Safe Startup Logic & CDC Synchronization
    // ========================================================================
    
    // A. Synchronize CMAC status signals to sys_clk domain
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

    // B. Logic based on synchronized signals
    // sync_status mapping: [5]=gt0_tx, [4]=gt0_rx, [3]=st0, [2]=gt1_tx, [1]=gt1_rx, [0]=st1
    wire system_ready;
//    assign system_ready = mmcm_locked &&
//                          (!sync_status[5]) && (!sync_status[4]) && sync_status[3] &&
//                          (!sync_status[2]) && (!sync_status[1]) && sync_status[0];
    assign system_ready = mmcm_locked;

//    assign system_ready = mmcm_locked && sync_status[3] && sync_status[0];

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
        .DATA_WIDTH(DATA_WIDTH),
        .PKT_LEN_BYTES(PKT_LEN_BYTES),
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
        .DATA_WIDTH(DATA_WIDTH),
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
    // 6. CDC FIFOs with Logic Fixes (Fixed Synth Error 8-6058)
    // ========================================================================
    // Change Log: Added .USE_ADV_FEATURES("0000") to disable unused Programmable Flags.

    // CMAC Common Widths
    localparam CMAC_DATA_WIDTH = 512;
    localparam CMAC_KEEP_WIDTH = 64;

    // CMAC Wires declared here for visibility
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
    // CDC 1: Injector (64b) -> CMAC 0 TX (512b)
    // ------------------------------------------------------------------------
    wire [79:0]  fifo1_din;
    wire [639:0] fifo1_dout;
    wire         fifo1_wr_en, fifo1_full;
    wire         fifo1_rd_en, fifo1_empty;

    assign fifo1_din   = {7'd0, inj_tlast, inj_tkeep, inj_tdata};
    assign fifo1_wr_en = inj_tvalid;
    assign inj_tready  = ~fifo1_full;

    xpm_fifo_async #(
        .FIFO_MEMORY_TYPE("auto"),
        .FIFO_WRITE_DEPTH(2048),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(80),
        .READ_DATA_WIDTH(640),
        .USE_ADV_FEATURES("0000") // [FIX] Disable Prog Full/Empty to avoid Threshold errors
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

    genvar i;
    generate
        for (i=0; i<8; i=i+1) begin : unpack_tx_c0
            assign c0_tx_tdata[i*64 +: 64] = fifo1_dout[i*80 + 0  +: 64];
            assign c0_tx_tkeep[i*8  +: 8]  = fifo1_dout[i*80 + 64 +: 8];
        end
    endgenerate

    wire [7:0] fifo1_last_vec;
    generate
        for (i=0; i<8; i=i+1) assign fifo1_last_vec[i] = fifo1_dout[i*80 + 72];
    endgenerate
    assign c0_tx_tlast  = |fifo1_last_vec; 
    assign c0_tx_tvalid = ~fifo1_empty; 
    assign fifo1_rd_en  = c0_tx_tready & ~fifo1_empty;


    // ------------------------------------------------------------------------
    // CDC 2: CMAC 0 RX (512b) -> Unpacker (128b)
    // ------------------------------------------------------------------------
    wire [FCP_AXIS_WIDTH-1:0] fifo2_din;
    wire                      fifo2_empty;
    
    assign fifo2_din = c0_rx_tdata[FCP_AXIS_WIDTH-1:0]; 

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(2048),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(FCP_AXIS_WIDTH),
        .READ_DATA_WIDTH(FCP_AXIS_WIDTH),
        .USE_ADV_FEATURES("0000") // [FIX] Disable Prog Full/Empty
    ) u_fifo_c0_to_unpacker (
        .rst(sys_rst || gt0_usr_rx_reset),
        .wr_clk(gt0_txusrclk2),// update
        .din(fifo2_din),
        .wr_en(c0_rx_tvalid),
        .full(),
        .rd_clk(sys_clk),
        .dout(unpacker_tdata),
        .rd_en(unpacker_tready),
        .empty(fifo2_empty),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );
    
    assign unpacker_tvalid = ~fifo2_empty;

    // ------------------------------------------------------------------------
    // CDC 3: Packer (128b) -> CMAC 1 TX (512b)
    // ------------------------------------------------------------------------
    wire [127:0] fifo3_dout_direct;
    wire         fifo3_empty_direct;
    wire         fifo3_rd_en;
    
    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(2048),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(128),
        .READ_DATA_WIDTH(128),
        .USE_ADV_FEATURES("0000") // [FIX] Disable Prog Full/Empty
    ) u_fifo_packer_to_c1_direct (
        .rst(sys_rst),
        .wr_clk(sys_clk),
        .din(packer_tdata),
        .wr_en(packer_tvalid),
        .full(),
        .rd_clk(gt1_txusrclk2),
        .dout(fifo3_dout_direct),
        .rd_en(fifo3_rd_en),
        .empty(fifo3_empty_direct),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );

    assign c1_tx_tdata  = {384'd0, fifo3_dout_direct};
    assign c1_tx_tkeep  = {{48{1'b0}}, {16{1'b1}}};
    assign c1_tx_tlast  = 1'b1;
    assign c1_tx_tvalid = ~fifo3_empty_direct;
    assign fifo3_rd_en  = c1_tx_tready & ~fifo3_empty_direct;

    // ------------------------------------------------------------------------
    // CDC 4: CMAC 1 RX (512b) -> Switch (64b)
    // ------------------------------------------------------------------------
    wire [639:0] fifo4_din;
    wire [79:0]  fifo4_dout;
    wire         fifo4_rd_en, fifo4_empty;

    wire [7:0] chunk_valid_mask;
    genvar k;
    generate
        for (k=0; k<8; k=k+1) begin : gen_chunk_valid
            assign chunk_valid_mask[k] = |c1_rx_tkeep[k*8 +: 8];
        end
    endgenerate

    wire [7:0] chunk_is_last;
    assign chunk_is_last[7] = c1_rx_tlast && chunk_valid_mask[7]; 
    assign chunk_is_last[6] = c1_rx_tlast && chunk_valid_mask[6] && (!chunk_valid_mask[7]);
    assign chunk_is_last[5] = c1_rx_tlast && chunk_valid_mask[5] && (!chunk_valid_mask[6]);
    assign chunk_is_last[4] = c1_rx_tlast && chunk_valid_mask[4] && (!chunk_valid_mask[5]);
    assign chunk_is_last[3] = c1_rx_tlast && chunk_valid_mask[3] && (!chunk_valid_mask[4]);
    assign chunk_is_last[2] = c1_rx_tlast && chunk_valid_mask[2] && (!chunk_valid_mask[3]);
    assign chunk_is_last[1] = c1_rx_tlast && chunk_valid_mask[1] && (!chunk_valid_mask[2]);
    assign chunk_is_last[0] = c1_rx_tlast && chunk_valid_mask[0] && (!chunk_valid_mask[1]);

    genvar j;
    generate
        for (j=0; j<8; j=j+1) begin : pack_rx_c1
            assign fifo4_din[j*80 + 0  +: 64] = c1_rx_tdata[j*64 +: 64];
            assign fifo4_din[j*80 + 64 +: 8]  = c1_rx_tkeep[j*8  +: 8];
            assign fifo4_din[j*80 + 72]       = chunk_is_last[j]; 
            assign fifo4_din[j*80 + 73 +: 7]  = 7'd0; // Padding
        end
    endgenerate

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(2048),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(640),
        .READ_DATA_WIDTH(80),
        .USE_ADV_FEATURES("0000") // [FIX] Disable Prog Full/Empty
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

    assign sw_tdata  = fifo4_dout[63:0];
    assign sw_tkeep  = fifo4_dout[71:64];
    assign sw_tlast  = fifo4_dout[72];
    
    assign sw_tvalid = ~fifo4_empty;
    assign fifo4_rd_en = sw_tready & ~fifo4_empty;
    wire [7:0] gt_powergoodout;
    // ========================================================================
    // 7. CMAC Instantiations
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
//        .gtwiz_reset_tx_datapath(gt_reset_pulse), .gtwiz_reset_rx_datapath(gt_reset_pulse),
        .gtwiz_reset_tx_datapath(1'b0), .gtwiz_reset_rx_datapath(1'b0),
        .usr_tx_reset(gt0_usr_tx_reset), .usr_rx_reset(gt0_usr_rx_reset),
        .core_tx_reset(1'b0), .core_rx_reset(1'b0), .core_drp_reset(1'b0),
        
        .ctl_tx_enable(1'b1), .ctl_rx_enable(1'b1),
        .ctl_tx_send_idle(1'b0), .ctl_tx_send_rfi(1'b0), .ctl_tx_send_lfi(1'b0),
        .ctl_tx_test_pattern(1'b0), .ctl_rx_test_pattern(1'b0), .ctl_rx_force_resync(1'b0),
//        .gt_loopback_in(12'h249),
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
//        .gtwiz_reset_tx_datapath(gt_reset_pulse), .gtwiz_reset_rx_datapath(gt_reset_pulse),
        .gtwiz_reset_tx_datapath(1'b0), .gtwiz_reset_rx_datapath(1'b0),
        .usr_tx_reset(gt1_usr_tx_reset), .usr_rx_reset(gt1_usr_rx_reset),
        .core_tx_reset(1'b0), .core_rx_reset(1'b0), .core_drp_reset(1'b0),
        
        .ctl_tx_enable(1'b1), .ctl_rx_enable(1'b1),
        .ctl_tx_send_idle(1'b0), .ctl_tx_send_rfi(1'b0), .ctl_tx_send_lfi(1'b0),
        .ctl_tx_test_pattern(1'b0), .ctl_rx_test_pattern(1'b0), .ctl_rx_force_resync(1'b0),
//        .gt_loopback_in(12'h249),//debug used
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
        
        .drp_clk(init_clk), .drp_addr(10'b0), .drp_di(16'b0), .drp_en(1'b0), .drp_we(1'b0), .drp_do(), .drp_rdy()
    );


    // ========================================================================
    // 8. ILA Debug Core (System Clock Domain)
    // ========================================================================
    
//     8-1. Used for Final Link
    ila_0 u_ila (
        .clk(sys_clk), 

        // Group 1: Data Path
        .probe0 (inj_tdata), .probe1 (inj_tkeep), .probe2 (inj_tlast), .probe3 (inj_tvalid), .probe4 (inj_tready),
        .probe5 (sw_tdata),  .probe6 (sw_tkeep),  .probe7 (sw_tlast),  .probe8 (sw_tvalid),  .probe9 (sw_tready),

        // Group 2: FCP Downstream
        .probe10(ds_fcp_valid), .probe11(ds_fcp_vc), .probe12(ds_fcp_qlen), .probe13(ds_fcp_fccl), .probe14(ds_fcp_fccr),

        // Group 3: FCP Upstream
        .probe15(us_fcp_valid), .probe16(us_fcp_vc), .probe17(us_fcp_qlen), .probe18(us_fcp_fccl), .probe19(us_fcp_fccr),

        // Group 4: Adapters
        .probe20(packer_tvalid), .probe21(packer_tready), .probe22(unpacker_tvalid), .probe23(enable_internal),

        // Group 5: Link Status
        .probe24(stat0_rx_aligned), .probe25(stat1_rx_aligned), .probe26(m_axis_tx_pkt_count), .probe27(dbg_buffer_free_count)
    );

//    // 8-2. Injector->FIFO1->CMAC0->FIFO2->Unpacker->Injector
//    ila_1 u_ila (
//        .clk(sys_clk), 

//        // -----------------------------------------------------------
//        // Path 1: Injector -> FIFO 1 (User Domain 64-bit)
//        // -----------------------------------------------------------
//        .probe0 (inj_tdata),       // [63:0]  Injector Data Out
//        .probe1 (inj_tkeep),       // [7:0]   Injector Keep Out
//        .probe2 (inj_tlast),       // [0:0]   Injector Last Out
//        .probe3 (inj_tvalid),      // [0:0]   Injector Valid Out
//        .probe4 (inj_tready),      // [0:0]   Injector Ready In (from FIFO1)

//        // FIFO 1 Status (Critical to see if data is stuck)
//        .probe5 (fifo1_full),      // [0:0]   FIFO 1 Full Flag
//        .probe6 (fifo1_empty),     // [0:0]   FIFO 1 Empty Flag

//        // -----------------------------------------------------------
//        // Path 2: FIFO 1 -> CMAC 0 TX (PHY Domain 512-bit - Probing sys_clk side only)
//        // -----------------------------------------------------------
//        // Note: Direct probing of c0_tx (512b) requires a separate ILA on gt0_txusrclk2.
//        // We only probe the FIFO write side status here (covered above).

//        // -----------------------------------------------------------
//        // Path 3: CMAC 0 RX -> FIFO 2 (PHY Domain 512-bit -> User Domain 128-bit)
//        // -----------------------------------------------------------
//        // Probing FIFO 2 Read Side (Unpacker Input)
//        .probe7 (fifo2_empty),     // [0:0]   FIFO 2 Empty Flag
//        // Note: packer/unpacker signals are below

//        // -----------------------------------------------------------
//        // Path 4: FIFO 2 -> Unpacker (User Domain 128-bit)
//        // -----------------------------------------------------------
//        .probe8 (unpacker_tdata),  // [127:0] Unpacker Data In (from FIFO2)
//        .probe9 (unpacker_tvalid), // [0:0]   Unpacker Valid In
//        .probe10(unpacker_tready), // [0:0]   Unpacker Ready Out

//        // -----------------------------------------------------------
//        // Path 5: Packer -> FIFO 3 (User Domain 128-bit)
//        // -----------------------------------------------------------
//        .probe11(packer_tdata),    // [127:0] Packer Data Out (to FIFO3)
//        .probe12(packer_tvalid),   // [0:0]   Packer Valid Out
//        .probe13(packer_tready),   // [0:0]   Packer Ready In

//        // -----------------------------------------------------------
//        // Path 6: CMAC 1 RX -> FIFO 4 (PHY Domain 512-bit -> User Domain 64-bit)
//        // -----------------------------------------------------------
//        // Probing FIFO 4 Read Side (Switch Input)
//        .probe14(fifo4_empty),     // [0:0]   FIFO 4 Empty Flag

//        // -----------------------------------------------------------
//        // Path 7: FIFO 4 -> Injector/Switch (Loopback Data - User Domain 64-bit)
//        // -----------------------------------------------------------
//        .probe15(sw_tdata),        // [63:0]  Switch Data In (from FIFO4)
//        .probe16(sw_tkeep),        // [7:0]   Switch Keep In
//        .probe17(sw_tlast),        // [0:0]   Switch Last In
//        .probe18(sw_tvalid),       // [0:0]   Switch Valid In
//        .probe19(sw_tready),       // [0:0]   Switch Ready Out

//        // -----------------------------------------------------------
//        // System Status & Counts
//        // -----------------------------------------------------------
//        .probe20(sync_status),     // [5:0]   CMAC Status {gt0_tx_rst, gt0_rx_rst, stat0_aligned, ...}
//        .probe21(gt_reset_pulse),  // [0:0]   GT Reset Pulse Status
//        .probe22(mmcm_locked),     // [0:0]   Clock Locked Status
//        .probe23(enable_internal), // [0:0]   System Enable Signal
//        .probe24(m_axis_tx_pkt_count), // [63:0] Total Packets Sent Counter
//        .probe25(gt_powergoodout)  // [7:0]   GT Power Good Status (Bit 0-3: CMAC0)
//    );

endmodule
`resetall