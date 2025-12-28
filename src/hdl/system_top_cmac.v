// SPDX-License-Identifier: BSD-2-Clause-Views
// Top Level Integration Wrapper with Dual CMAC and CDC
// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none

module system_top_cmac #
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
    parameter QMIN              = 3
)
(
    // System Clock (User Logic)
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
    // 1. Clock & Reset Management & Clock Generation
    // ========================================================================
    wire sys_clk;     // 100MHz (User Logic)
    wire sys_rst;     // Active High Reset
    wire init_clk;    // 125MHz (Generated from sys_clk)
    wire mmcm_locked; // Status of the clock generator

    // 1.1 Input Buffer for Differential System Clock
    IBUFDS sys_clk_ibufds (.I(sys_clk_p), .IB(sys_clk_n), .O(sys_clk));
    
    // 1.2 Reset Inversion
    assign sys_rst = ~sys_rst_n;

    // 1.3 MMCM Primitive: 100MHz -> 125MHz
    // ------------------------------------------------------------------------
    wire clk_mmcm_feedback;
    wire init_clk_unbuffered;

    MMCME4_BASE #(
        .BANDWIDTH("OPTIMIZED"),
        .CLKOUT4_CASCADE("FALSE"),
        .STARTUP_WAIT("FALSE"),
        .CLKIN1_PERIOD(10.0),    // Input: 100 MHz (10ns)
        .CLKFBOUT_MULT_F(10.0),  // VCO = 100 MHz * 10 = 1000 MHz
        .DIVCLK_DIVIDE(1),
        .CLKOUT0_DIVIDE_F(8.0),  // Output: 1000 MHz / 8 = 125 MHz
        .CLKOUT0_DUTY_CYCLE(0.5),
        .CLKOUT0_PHASE(0.0)
    ) u_mmcm_gen (
        .CLKIN1(sys_clk),        // 100 MHz Input
        .CLKFBIN(clk_mmcm_feedback), // Feedback Loop
        
        .CLKOUT0(init_clk_unbuffered), // 125 MHz Output
        .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(), 
        .CLKOUT3(), .CLKOUT3B(), .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
        
        .CLKFBOUT(clk_mmcm_feedback), // Feedback Output
        .CLKFBOUTB(),
        
        .LOCKED(mmcm_locked),    // 1 = Stable
        .RST(sys_rst),           // Reset MMCM if system reset is pressed
        .PWRDWN(1'b0)
    );

    // 1.4 Global Buffer for the generated 125MHz clock
    BUFG u_bufg_init_clk (
        .I(init_clk_unbuffered),
        .O(init_clk)
    );

    // ------------------------------------------------------------------------
    // 1.5 Safe Startup Logic (Updated with MMCM Locked check)
    // ------------------------------------------------------------------------
    reg  enable_reg;
    wire enable_internal;
    
    // Add MMCM Locked to the readiness check
    wire system_ready;
    assign system_ready = mmcm_locked &&           // Clock must be stable
                          (!gt0_usr_tx_reset) && (!gt0_usr_rx_reset) && stat0_rx_aligned &&
                          (!gt1_usr_tx_reset) && (!gt1_usr_rx_reset) && stat1_rx_aligned;

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
    // 2. User Logic Signal Declarations (System Clock Domain)
    // ========================================================================
    
    // Injector Output (Data)
    wire [DATA_WIDTH-1:0]   inj_tdata;
    wire                    inj_tvalid;
    wire                    inj_tlast;
    wire [DATA_WIDTH/8-1:0] inj_tkeep;
    wire                    inj_tready; // Backpressure from CDC FIFO

    // Switch Input (Data)
    wire [DATA_WIDTH-1:0]   sw_tdata;
    wire                    sw_tvalid;
    wire                    sw_tlast;
    wire [DATA_WIDTH/8-1:0] sw_tkeep;
    wire                    sw_tready;

    // Discrete FCP Signals (Switch <-> Adapters)
    // Downstream (Switch) Output
    wire                      ds_fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] ds_fcp_vc;
    wire [STAT_WIDTH-1:0]     ds_fcp_fccl;
    wire [STAT_WIDTH-1:0]     ds_fcp_qlen;
    wire [STAT_WIDTH-1:0]     ds_fcp_fccr;
    
    // Upstream (Injector) Input
    wire                      us_fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] us_fcp_vc;
    wire [STAT_WIDTH-1:0]     us_fcp_fccl;
    wire [STAT_WIDTH-1:0]     us_fcp_qlen;
    wire [STAT_WIDTH-1:0]     us_fcp_fccr;

    // Packer Output (AXIS FCP)
    wire [FCP_AXIS_WIDTH-1:0] packer_tdata;
    wire                      packer_tvalid;
    wire                      packer_tready; // Backpressure from CDC FIFO

    // Unpacker Input (AXIS FCP)
    wire [FCP_AXIS_WIDTH-1:0] unpacker_tdata;
    wire                      unpacker_tvalid;
    wire                      unpacker_tready;

    // Debug Signals
    wire [63:0] m_axis_tx_pkt_count;
    wire [STAT_WIDTH-1:0] dbg_buffer_free_count;
    wire [STAT_WIDTH-1:0] dbg_total_rx_count;

    // ========================================================================
    // 3. User Logic Instantiation
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
        .QMIN(QMIN)
    ) upstream_inst (
        .clk(sys_clk),
        .rst(sys_rst),
        .enable(enable_internal),

        // Data Output -> Goes to CMAC 0
        .m_axis_pkt_tdata(inj_tdata),
        .m_axis_pkt_tvalid(inj_tvalid),
        .m_axis_pkt_tlast(inj_tlast),
        .m_axis_pkt_tkeep(inj_tkeep),
        .m_axis_pkt_tready(inj_tready),

        // FCP Input <- Comes from CMAC 0
        .fcp_valid(us_fcp_valid),
        .fcp_vc(us_fcp_vc),
        .fcp_fccl(us_fcp_fccl),
        .fcp_fccr(us_fcp_fccr),
        .fcp_qlen(us_fcp_qlen),

        // Debug Used
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

        // Data Input <- Comes from CMAC 1
        .s_axis_pkt_tdata(sw_tdata),
        .s_axis_pkt_tvalid(sw_tvalid),
        .s_axis_pkt_tlast(sw_tlast),
        .s_axis_pkt_tkeep(sw_tkeep),
        .s_axis_pkt_tready(sw_tready),

        // FCP Output -> Goes to CMAC 1
        .fcp_valid(ds_fcp_valid),
        .fcp_vc(ds_fcp_vc),
        .fcp_fccl(ds_fcp_fccl),
        .fcp_qlen(ds_fcp_qlen),
        .fcp_fccr(ds_fcp_fccr),

        // m_axis_pkt_tdata...
        
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
        .m_axis_fcp_tready(packer_tready) // Driven by FIFO
    );

    fcp_sink_adapter #(.AXIS_WIDTH(FCP_AXIS_WIDTH)) fcp_unpacker (
        .s_axis_fcp_tdata(unpacker_tdata),
        .s_axis_fcp_tvalid(unpacker_tvalid),
        .s_axis_fcp_tready(unpacker_tready),
        .fcp_valid(us_fcp_valid),
        .fcp_vc(us_fcp_vc),
        // ... connect other FCP signals ...
        .fcp_fccl(us_fcp_fccl), .fcp_qlen(us_fcp_qlen), .fcp_fccr(us_fcp_fccr)
    );

    // ========================================================================
    // 4. CMAC Signals & Clock Domain Crossing Logic
    // ========================================================================
    
    // CMAC Common Widths
    localparam CMAC_DATA_WIDTH = 512;
    localparam CMAC_KEEP_WIDTH = 64;

    // --- CMAC 0 Signals (Upstream / Data TX / FCP RX) ---
    wire gt0_txusrclk2;
    wire gt0_rx_clk;
    wire gt0_usr_tx_reset, gt0_usr_rx_reset;
    wire stat0_rx_aligned;
    
    // CMAC0 TX Interface (Driven by Injector via FIFO)
    wire [CMAC_DATA_WIDTH-1:0] c0_tx_tdata;
    wire [CMAC_KEEP_WIDTH-1:0] c0_tx_tkeep;
    wire                       c0_tx_tvalid;
    wire                       c0_tx_tlast;
    wire                       c0_tx_tready;

    // CMAC0 RX Interface (Drives Unpacker via FIFO)
    wire [CMAC_DATA_WIDTH-1:0] c0_rx_tdata;
    wire [CMAC_KEEP_WIDTH-1:0] c0_rx_tkeep;
    wire                       c0_rx_tvalid;
    wire                       c0_rx_tlast;
    
    // --- CMAC 1 Signals (Downstream / FCP TX / Data RX) ---
    wire gt1_txusrclk2;
    wire gt1_rx_clk;
    wire gt1_usr_tx_reset, gt1_usr_rx_reset;
    wire stat1_rx_aligned;

    // CMAC1 TX Interface (Driven by Packer via FIFO)
    wire [CMAC_DATA_WIDTH-1:0] c1_tx_tdata;
    wire [CMAC_KEEP_WIDTH-1:0] c1_tx_tkeep;
    wire                       c1_tx_tvalid;
    wire                       c1_tx_tlast;
    wire                       c1_tx_tready;

    // CMAC1 RX Interface (Drives Switch via FIFO)
    wire [CMAC_DATA_WIDTH-1:0] c1_rx_tdata;
    wire [CMAC_KEEP_WIDTH-1:0] c1_rx_tkeep;
    wire                       c1_rx_tvalid;
    wire                       c1_rx_tlast;

    // ========================================================================
    // 5. CDC FIFOs with Width Adaptation (Logic -> CMAC Padding)
    // ========================================================================

    // ------------------------------------------------------------------------
    // CDC 1: Injector (64b) -> CMAC 0 TX (512b)
    // ------------------------------------------------------------------------
    wire [DATA_WIDTH-1:0]   fifo1_out_tdata;
    wire [DATA_WIDTH/8-1:0] fifo1_out_tkeep;
    wire                    fifo1_out_tvalid;
    wire                    fifo1_out_tlast;
    wire                    fifo1_out_tready;

    xpm_fifo_axis #(
        .CLOCKING_MODE("independent_clock"), 
        .FIFO_DEPTH(2048),
        .TDATA_WIDTH(DATA_WIDTH + DATA_WIDTH/8 + 1), // Data + Keep + Last
        .PROG_EMPTY_THRESH(10)
    ) u_fifo_inj_to_c0 (
        .s_aclk(sys_clk),
        .s_aresetn(~sys_rst),
        .s_axis_tdata({inj_tlast, inj_tkeep, inj_tdata}),
        .s_axis_tvalid(inj_tvalid),
        .s_axis_tready(inj_tready),
        
        .m_aclk(gt0_txusrclk2),
        .m_axis_tdata({fifo1_out_tlast, fifo1_out_tkeep, fifo1_out_tdata}),
        .m_axis_tvalid(fifo1_out_tvalid),
        .m_axis_tready(fifo1_out_tready) // Controlled by CMAC logic below
    );

    // [Padding Logic] 64b -> 512b
    assign c0_tx_tdata  = { {(CMAC_DATA_WIDTH-DATA_WIDTH){1'b0}}, fifo1_out_tdata };
    assign c0_tx_tkeep  = { {(CMAC_KEEP_WIDTH-(DATA_WIDTH/8)){1'b0}}, fifo1_out_tkeep };
    assign c0_tx_tlast  = fifo1_out_tlast;
    assign c0_tx_tvalid = fifo1_out_tvalid;
    assign fifo1_out_tready = c0_tx_tready;


    // ------------------------------------------------------------------------
    // CDC 2: CMAC 0 RX (512b) -> Unpacker (128b) [Received FCP]
    // ------------------------------------------------------------------------
    // Note: FCP comes from CMAC 1 via loopback.
    // [Slicing Logic] 512b -> 128b happens BEFORE writing to FIFO to save resources
    wire [FCP_AXIS_WIDTH-1:0] c0_rx_tdata_sliced = c0_rx_tdata[FCP_AXIS_WIDTH-1:0];

    xpm_fifo_axis #(
        .CLOCKING_MODE("independent_clock"),
        .FIFO_DEPTH(2048),
        .TDATA_WIDTH(FCP_AXIS_WIDTH) // FCP doesn't need keep/last usually, but good practice
    ) u_fifo_c0_to_unpacker (
        .s_aclk(gt0_rx_clk),
        .s_aresetn(~gt0_usr_rx_reset),
        .s_axis_tdata(c0_rx_tdata_sliced), // Sliced
        .s_axis_tvalid(c0_rx_tvalid),
        .s_axis_tready(), // CMAC RX has no tready, we must be fast enough!
        
        .m_aclk(sys_clk),
        .m_axis_tdata(unpacker_tdata),
        .m_axis_tvalid(unpacker_tvalid),
        .m_axis_tready(unpacker_tready)
    );

    // ------------------------------------------------------------------------
    // CDC 3: Packer (128b) -> CMAC 1 TX (512b) [Sending FCP]
    // ------------------------------------------------------------------------
    wire [FCP_AXIS_WIDTH-1:0] fifo3_out_tdata;
    wire                      fifo3_out_tvalid;
    wire                      fifo3_out_tready;

    xpm_fifo_axis #(
        .CLOCKING_MODE("independent_clock"),
        .FIFO_DEPTH(2048),
        .TDATA_WIDTH(FCP_AXIS_WIDTH)
    ) u_fifo_packer_to_c1 (
        .s_aclk(sys_clk),
        .s_aresetn(~sys_rst),
        .s_axis_tdata(packer_tdata),
        .s_axis_tvalid(packer_tvalid),
        .s_axis_tready(packer_tready),
        
        .m_aclk(gt1_txusrclk2),
        .m_axis_tdata(fifo3_out_tdata),
        .m_axis_tvalid(fifo3_out_tvalid),
        .m_axis_tready(fifo3_out_tready)
    );

    // [Padding Logic] 128b -> 512b
    assign c1_tx_tdata  = { {(CMAC_DATA_WIDTH-FCP_AXIS_WIDTH){1'b0}}, fifo3_out_tdata };
//    assign c1_tx_tkeep  = { (CMAC_KEEP_WIDTH){1'b1} }; // Keep all valid for FCP? Or strictly based on width. 
    assign c1_tx_tkeep = { {(CMAC_KEEP_WIDTH - 16){1'b0}}, {16{1'b1}} };
    // Simplified: Assuming FCP always valid fully.
    assign c1_tx_tlast  = 1'b1; // Single flit packet for FCP usually
    assign c1_tx_tvalid = fifo3_out_tvalid;
    assign fifo3_out_tready = c1_tx_tready;

    // ------------------------------------------------------------------------
    // CDC 4: CMAC 1 RX (512b) -> Switch (64b) [Received Data]
    // ------------------------------------------------------------------------
    // [Slicing Logic] 512b -> 64b
    wire [DATA_WIDTH-1:0]   c1_rx_tdata_sliced = c1_rx_tdata[DATA_WIDTH-1:0];
    wire [DATA_WIDTH/8-1:0] c1_rx_tkeep_sliced = c1_rx_tkeep[DATA_WIDTH/8-1:0];

    xpm_fifo_axis #(
        .CLOCKING_MODE("independent_clock"),
        .FIFO_DEPTH(2048),
        .TDATA_WIDTH(DATA_WIDTH + DATA_WIDTH/8 + 1)
    ) u_fifo_c1_to_sw (
        .s_aclk(gt1_rx_clk),
        .s_aresetn(~gt1_usr_rx_reset),
        .s_axis_tdata({c1_rx_tlast, c1_rx_tkeep_sliced, c1_rx_tdata_sliced}),
        .s_axis_tvalid(c1_rx_tvalid),
        .s_axis_tready(), // No backpressure to CMAC
        
        .m_aclk(sys_clk),
        .m_axis_tdata({sw_tlast, sw_tkeep, sw_tdata}),
        .m_axis_tvalid(sw_tvalid),
        .m_axis_tready(sw_tready)
    );

    // ========================================================================
    // 6. CMAC 0 Instantiation (Upstream Node)
    // ========================================================================
    cmac_usplus_0 u_cmac_0 (
        .gt_rxp_in(gt0_rxp_in), .gt_rxn_in(gt0_rxn_in),
        .gt_txp_out(gt0_txp_out), .gt_txn_out(gt0_txn_out),
        .gt_ref_clk_p(gt0_ref_clk_p), .gt_ref_clk_n(gt0_ref_clk_n),
        .init_clk(init_clk),
        .gt_txusrclk2(gt0_txusrclk2),
        .rx_clk(gt0_rx_clk),
        .gt_ref_clk_out(),
        
        .sys_reset(sys_rst), // Active High
        .gtwiz_reset_tx_datapath(1'b0), .gtwiz_reset_rx_datapath(1'b0),
        .usr_tx_reset(gt0_usr_tx_reset), .usr_rx_reset(gt0_usr_rx_reset),
        .core_tx_reset(1'b0), .core_rx_reset(1'b0), .core_drp_reset(1'b0),
        
        .ctl_tx_enable(1'b1), .ctl_rx_enable(1'b1),
        .ctl_tx_send_idle(1'b0), .ctl_tx_send_rfi(1'b0), .ctl_tx_send_lfi(1'b0),
        .ctl_tx_test_pattern(1'b0), .ctl_rx_test_pattern(1'b0), .ctl_rx_force_resync(1'b0),
        .gt_loopback_in(12'b0),
        
        // TX Interface (From Injector/FIFO)
        .tx_axis_tready(c0_tx_tready),
        .tx_axis_tvalid(c0_tx_tvalid),
        .tx_axis_tdata(c0_tx_tdata),
        .tx_axis_tlast(c0_tx_tlast),
        .tx_axis_tkeep(c0_tx_tkeep),
        .tx_axis_tuser(1'b0), .tx_ovfout(), .tx_unfout(), .tx_preamblein(56'd0),
        
        // RX Interface (To Unpacker/FIFO)
        .rx_axis_tvalid(c0_rx_tvalid),
        .rx_axis_tdata(c0_rx_tdata),
        .rx_axis_tlast(c0_rx_tlast),
        .rx_axis_tkeep(c0_rx_tkeep),
        .rx_axis_tuser(), .rx_preambleout(),
        
        .stat_rx_aligned(stat0_rx_aligned),
        .stat_rx_status(),
        
        .drp_clk(init_clk), .drp_addr(10'b0), .drp_di(16'b0), .drp_en(1'b0), .drp_we(1'b0), .drp_do(), .drp_rdy()
    );

    // ========================================================================
    // 7. CMAC 1 Instantiation (Downstream Node)
    // ========================================================================
    cmac_usplus_1 u_cmac_1 (
        .gt_rxp_in(gt1_rxp_in), .gt_rxn_in(gt1_rxn_in),
        .gt_txp_out(gt1_txp_out), .gt_txn_out(gt1_txn_out),
        .gt_ref_clk_p(gt1_ref_clk_p), .gt_ref_clk_n(gt1_ref_clk_n),
        .init_clk(init_clk),
        .gt_txusrclk2(gt1_txusrclk2),
        .rx_clk(gt1_rx_clk),
        .gt_ref_clk_out(),
        
        .sys_reset(sys_rst),
        .gtwiz_reset_tx_datapath(1'b0), .gtwiz_reset_rx_datapath(1'b0),
        .usr_tx_reset(gt1_usr_tx_reset), .usr_rx_reset(gt1_usr_rx_reset),
        .core_tx_reset(1'b0), .core_rx_reset(1'b0), .core_drp_reset(1'b0),
        
        .ctl_tx_enable(1'b1), .ctl_rx_enable(1'b1),
        .ctl_tx_send_idle(1'b0), .ctl_tx_send_rfi(1'b0), .ctl_tx_send_lfi(1'b0),
        .ctl_tx_test_pattern(1'b0), .ctl_rx_test_pattern(1'b0), .ctl_rx_force_resync(1'b0),
        .gt_loopback_in(12'b0),
        
        // TX Interface (From Packer/FIFO)
        .tx_axis_tready(c1_tx_tready),
        .tx_axis_tvalid(c1_tx_tvalid),
        .tx_axis_tdata(c1_tx_tdata),
        .tx_axis_tlast(c1_tx_tlast),
        .tx_axis_tkeep(c1_tx_tkeep),
        .tx_axis_tuser(1'b0), .tx_ovfout(), .tx_unfout(), .tx_preamblein(56'd0),
        
        // RX Interface (To Switch/FIFO)
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
    
    ila_0 u_ila (
        .clk(sys_clk), // Sampling Clock: System User Clock

        // -----------------------------------------------------------
        // Group 1: Forward Data Path (Injector -> CMAC -> Switch)
        // -----------------------------------------------------------
        // Source Side (Injector Output)
        .probe0 (inj_tdata),        // [63:0] Data Payload
        .probe1 (inj_tkeep),        // [7:0]  Keep Strobe (Important for sparse packing check)
        .probe2 (inj_tlast),        // [0:0]  Packet Boundary
        .probe3 (inj_tvalid),       // [0:0]  Valid
        .probe4 (inj_tready),       // [0:0]  Backpressure from TX FIFO/CMAC

        // Sink Side (Switch Input) - Compare with Source to verify transmission
        .probe5 (sw_tdata),         // [63:0] Data Payload
        .probe6 (sw_tkeep),         // [7:0]  Keep Strobe
        .probe7 (sw_tlast),         // [0:0]  Packet Boundary
        .probe8 (sw_tvalid),        // [0:0]  Valid
        .probe9 (sw_tready),        // [0:0]  Backpressure from Switch Buffer

        // -----------------------------------------------------------
        // Group 2: Downstream FCP (Generated by Switch -> To Packer)
        // -----------------------------------------------------------
        // This group shows what the Switch *wants* to tell the Injector
        .probe10(ds_fcp_valid),     // [0:0]  FCP Transaction Valid
        .probe11(ds_fcp_vc),        // [15:0] Target Queue/VC Index
        .probe12(ds_fcp_qlen),      // [31:0] Current Queue Length (Congestion Level)
        .probe13(ds_fcp_fccl),      // [31:0] Flow Control Credit Limit (CRITICAL)
        .probe14(ds_fcp_fccr),      // [31:0] Flow Control Credit Return/Request (CRITICAL)

        // -----------------------------------------------------------
        // Group 3: Upstream FCP (Received by Injector <- From Unpacker)
        // -----------------------------------------------------------
        // This group shows what the Injector *actually received* after the loop
        // Compare Probe 13 vs Probe 18 to check credit integrity
        .probe15(us_fcp_valid),     // [0:0]  FCP Transaction Valid
        .probe16(us_fcp_vc),        // [15:0] Target Queue/VC Index
        .probe17(us_fcp_qlen),      // [31:0] Current Queue Length
        .probe18(us_fcp_fccl),      // [31:0] Flow Control Credit Limit (CRITICAL)
        .probe19(us_fcp_fccr),      // [31:0] Flow Control Credit Return/Request (CRITICAL)

        // -----------------------------------------------------------
        // Group 4: Adapter & FIFO Status (Intermediate Links)
        // -----------------------------------------------------------
        .probe20(packer_tvalid),    // [0:0]  Adapter Output Valid
        .probe21(packer_tready),    // [0:0]  Adapter Output Ready (FIFO Pressure)
        .probe22(unpacker_tvalid),  // [0:0]  Adapter Input Valid
        .probe23(enable_internal),  // [0:0]  System Global Enable

        // -----------------------------------------------------------
        // Group 5: Physical Link & Statistics
        // -----------------------------------------------------------
        .probe24(stat0_rx_aligned),    // [0:0]  CMAC 0 Link Status (Must be 1)
        .probe25(stat1_rx_aligned),    // [0:0]  CMAC 1 Link Status (Must be 1)
        .probe26(m_axis_tx_pkt_count), // [63:0] Total Tx Packets (Heartbeat)
        .probe27(dbg_buffer_free_count)// [31:0] Switch Buffer Free Cells
    );

endmodule
`resetall