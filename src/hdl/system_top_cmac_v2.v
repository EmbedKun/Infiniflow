// SPDX-License-Identifier: BSD-2-Clause-Views
// Top Level Integration Wrapper with Dual CMAC and CDC
// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none

module system_top_cmac_v2 #
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
    // 5. CDC FIFOs using Generic Async FIFO (Correct Logic Fix)
    // ========================================================================

    // ------------------------------------------------------------------------
    // CDC 1: Injector (64b) -> CMAC 0 TX (512b)
    // 逻辑检查：通过。TLAST 在 512b 侧是 OR 关系，CMAC 通过 TKEEP 识别结尾。
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
        .PROG_EMPTY_THRESH(10)
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

    // Injector侧每拍都有Last信息，聚合后如果任意一拍是Last，则CMAC侧该周期为Last
    wire [7:0] fifo1_last_vec;
    generate
        for (i=0; i<8; i=i+1) assign fifo1_last_vec[i] = fifo1_dout[i*80 + 72];
    endgenerate
    assign c0_tx_tlast  = |fifo1_last_vec; 
    assign c0_tx_tvalid = ~fifo1_empty; 
    assign fifo1_rd_en  = c0_tx_tready & ~fifo1_empty;


    // ------------------------------------------------------------------------
    // CDC 2: CMAC 0 RX (512b) -> Unpacker (128b)
    // 逻辑检查：修复了上一版遗漏的 Valid 信号赋值
    // ------------------------------------------------------------------------
    wire [FCP_AXIS_WIDTH-1:0] fifo2_din;
    wire                      fifo2_empty;
    
    assign fifo2_din = c0_rx_tdata[FCP_AXIS_WIDTH-1:0]; 

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(2048),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(FCP_AXIS_WIDTH),
        .READ_DATA_WIDTH(FCP_AXIS_WIDTH)
    ) u_fifo_c0_to_unpacker (
        .rst(gt0_usr_rx_reset),
        .wr_clk(gt0_rx_clk),
        .din(fifo2_din),
        .wr_en(c0_rx_tvalid),
        .full(),
        .rd_clk(sys_clk),
        .dout(unpacker_tdata),
        .rd_en(unpacker_tready),
        .empty(fifo2_empty),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );
    
    // [FIX] 正确生成 Valid 信号
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
        .READ_DATA_WIDTH(128)
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
    // 逻辑检查：[重点修正] TLAST 逻辑重写，防止碎包
    // ------------------------------------------------------------------------
    wire [639:0] fifo4_din;
    wire [79:0]  fifo4_dout;
    wire         fifo4_rd_en, fifo4_empty;

    // --- TLAST 定位逻辑 (Priority Logic) ---
    // 我们必须找出哪个 Chunk 是最后一个有效的 Chunk，并将 c1_rx_tlast 仅赋给它。
    // 如果 c1_rx_tlast 为 0，则所有 chunks 的 last 都是 0。
    
    wire [7:0] chunk_valid_mask;
    genvar k;
    generate
        for (k=0; k<8; k=k+1) begin : gen_chunk_valid
            // 如果该 Chunk 的 Keep 不全为 0，则认为该 Chunk 有效
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

    // 打包逻辑
    genvar j;
    generate
        for (j=0; j<8; j=j+1) begin : pack_rx_c1
            assign fifo4_din[j*80 + 0  +: 64] = c1_rx_tdata[j*64 +: 64];
            assign fifo4_din[j*80 + 64 +: 8]  = c1_rx_tkeep[j*8  +: 8];
            // [FIXED] 只有被判定为 Last 的那个 Chunk，第 72 位才置 1
            assign fifo4_din[j*80 + 72]       = chunk_is_last[j]; 
            assign fifo4_din[j*80 + 73 +: 7]  = 7'd0; // Padding
        end
    endgenerate

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH(2048),
        .READ_MODE("fwft"),
        .WRITE_DATA_WIDTH(640),
        .READ_DATA_WIDTH(80),
        .PROG_EMPTY_THRESH(10)
    ) u_fifo_c1_to_sw (
        .rst(gt1_usr_rx_reset),
        .wr_clk(gt1_rx_clk),
        .din(fifo4_din),
        .wr_en(c1_rx_tvalid),
        .full(),
        .rd_clk(sys_clk),
        .dout(fifo4_dout),
        .rd_en(fifo4_rd_en),
        .empty(fifo4_empty),
        .sleep(1'b0), .injectsbiterr(1'b0), .injectdbiterr(1'b0)
    );

    // Unpack to Switch
    assign sw_tdata  = fifo4_dout[63:0];
    assign sw_tkeep  = fifo4_dout[71:64];
    assign sw_tlast  = fifo4_dout[72]; // 现在这个 TLAST 是准确的了
    
    assign sw_tvalid = ~fifo4_empty;
    assign fifo4_rd_en = sw_tready & ~fifo4_empty;

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