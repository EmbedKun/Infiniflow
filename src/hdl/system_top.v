// SPDX-License-Identifier: BSD-2-Clause-Views
// Top Level Integration Wrapper
// Connects Massive Traffic Injector -> Downstream Switch Model
// Exposes Debug Interfaces for Observation.
// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none

module system_top #
(
    // ---------------------------------------------------------
    // Global Configuration
    // ---------------------------------------------------------
    parameter QUEUE_INDEX_WIDTH = 5, // 262,144 Queues
    parameter DATA_WIDTH        = 64, // 64-bit Data Bus

    // ---------------------------------------------------------
    // Injector Specific
    // ---------------------------------------------------------
    parameter REQ_TAG_WIDTH     = 8,
    parameter LEN_WIDTH         = 16,
    parameter OP_TABLE_SIZE     = 16,
    parameter PKT_LEN_BYTES     = 64,
    parameter PIPELINE = 3+(QUEUE_INDEX_WIDTH > 12 ? QUEUE_INDEX_WIDTH-12 : 0),

    // ---------------------------------------------------------
    // Switch Specific
    // ---------------------------------------------------------
    parameter ACTIVE_FIFO_DEPTH = 1024,
    //Drain from active FIFO
    parameter BUFFER_ADDR_WIDTH = 10, // 16K Cells
    parameter STAT_WIDTH        = 32,
    parameter DRAIN_RATIO_M     = 8,  // Drain Rate 1:4
    parameter QMAX = 6,
    parameter QMIN = 3
)
(
    input  wire                          clk_p,   
    input  wire                          clk_n,   
    input  wire                          rst_n,

    // ---------------------------------------------------------
    // 1. Control Interface (To Injector)
    // ---------------------------------------------------------
    input  wire                          enable,          // Start Injection
    input  wire [QUEUE_INDEX_WIDTH-1:0]  stop_queue_idx,  // Target Queue to Kill
    input  wire                          stop_cmd_valid,  // Trigger Kill

    // ---------------------------------------------------------
    // 2. FCP Output Interface (From Switch)
    //    Flow Control Packets / Statistics
    // ---------------------------------------------------------
    output wire                          dbg_link_fcp_valid,
    output wire [QUEUE_INDEX_WIDTH-1:0]  dbg_link_fcp_vc,
    output wire [STAT_WIDTH-1:0]         dbg_link_fcp_fccl, // Remaining Buffer + Total Rx
    output wire [STAT_WIDTH-1:0]         dbg_link_fcp_qlen, // Queue Length
    output wire [STAT_WIDTH-1:0]         dbg_link_fcp_fccr, // Tx Count

    // ---------------------------------------------------------
    // 3. DEBUG Interface (Monitoring the Link)
    //    These signals allow you to see what is passing between
    //    Injector and Switch without affecting logic.
    // ---------------------------------------------------------
    output wire [DATA_WIDTH-1:0]         dbg_link_tdata,
    output wire                          dbg_link_tvalid,
    output wire                          dbg_link_tlast,
    output wire [DATA_WIDTH/8-1:0]       dbg_link_tkeep,
    output wire                          dbg_link_tready, // Critical: observe backpressure

    // Internal Status
    output wire                          dbg_injector_active,

    // Output link: Debug used
    output wire  [DATA_WIDTH-1:0]        out_link_tdata,
    output wire                          out_link_tvalid,

    // Statistics Outputs
    output wire  [63:0]                   m_axis_tx_pkt_count,
    output wire  [31:0]                   m_axis_vc_tx_count,
    output wire  [STAT_WIDTH-1:0]         dbg_buffer_free_count,
    output wire  [STAT_WIDTH-1:0]         dbg_total_rx_count
);

    // ========================================================================
    // Internal Interconnect Signals (The Link)
    // ========================================================================
    wire [DATA_WIDTH-1:0]         link_tdata;
    wire                          link_tvalid;
    wire                          link_tlast;
    wire [DATA_WIDTH/8-1:0]       link_tkeep;
    wire                          link_tready;
    wire                          fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0]  fcp_vc;
    wire [STAT_WIDTH-1:0]         fcp_fccl; 
    wire [STAT_WIDTH-1:0]         fcp_qlen; 
    wire [STAT_WIDTH-1:0]         fcp_fccr; 
    wire                          rst;
    assign rst = ~rst_n;
    // ========================================================================
    // 1. Instantiate Upstream (Massive Traffic Injector)
    // ========================================================================
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
        .clk(clk_p),
        .rst(rst),

        // Control
        .enable(enable),
        .stop_queue_idx(stop_queue_idx),
        .stop_cmd_valid(stop_cmd_valid),

        // Output (Source) -> Connects to internal link wires
        .m_axis_pkt_tdata(link_tdata),
        .m_axis_pkt_tvalid(link_tvalid),
        .m_axis_pkt_tlast(link_tlast),
        .m_axis_pkt_tkeep(link_tkeep),
        .m_axis_pkt_tready(link_tready), // Input from downstream

        // Status
        .scheduler_active(dbg_injector_active),

        // Statistics Outputs
        .m_axis_tx_pkt_count(m_axis_tx_pkt_count),
        .m_axis_vc_tx_count (m_axis_vc_tx_count),

        // FCP Input
//        .fcp_valid(fcp_valid),
//        .fcp_vc(fcp_vc),
//        .fcp_fccl(fcp_fccl),
//        .fcp_qlen(fcp_qlen),
//        .fcp_fccr(fcp_fccr)
        .fcp_valid(us_fcp_valid),
        .fcp_vc(us_fcp_vc),
        .fcp_fccl(us_fcp_fccl),
        .fcp_qlen(us_fcp_qlen),
        .fcp_fccr(us_fcp_fccr)
    );

    // ========================================================================
    // 2. Instantiate Downstream (Switch Model)
    // ========================================================================
//    downstream_switch_model_bram #(
    downstream_switch_model_v2 #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .STAT_WIDTH(STAT_WIDTH),
        .DRAIN_RATIO_M(DRAIN_RATIO_M),
        .ACTIVE_FIFO_DEPTH(ACTIVE_FIFO_DEPTH)
    ) downstream_inst (
        .clk(clk_p),
        .rst(rst),

        // Input (Sink) -> Connects to internal link wires
        .s_axis_pkt_tdata(link_tdata),
        .s_axis_pkt_tvalid(link_tvalid),
        .s_axis_pkt_tlast(link_tlast),
        .s_axis_pkt_tkeep(link_tkeep),
        .s_axis_pkt_tready(link_tready), // Output backpressure

        // Output : Debug used
        .m_axis_pkt_tdata(out_link_tdata),
        .m_axis_pkt_tvalid(out_link_tvalid),

        // FCP Output
//        .fcp_valid(fcp_valid),
//        .fcp_vc(fcp_vc),
//        .fcp_fccl(fcp_fccl),
//        .fcp_qlen(fcp_qlen),
//        .fcp_fccr(fcp_fccr),
        .fcp_valid(ds_fcp_valid),
        .fcp_vc(ds_fcp_vc),
        .fcp_fccl(ds_fcp_fccl),
        .fcp_qlen(ds_fcp_qlen),
        .fcp_fccr(ds_fcp_fccr),

        .dbg_buffer_free_count(dbg_buffer_free_count),
        .dbg_total_rx_count(dbg_total_rx_count)
    );


// --- Internal Discrete FCP Signals (Unchanged) ---
    // From Downstream (Source)
    wire                         ds_fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] ds_fcp_vc;
    wire [STAT_WIDTH-1:0]        ds_fcp_fccl;
    wire [STAT_WIDTH-1:0]        ds_fcp_qlen;
    wire [STAT_WIDTH-1:0]        ds_fcp_fccr;

    // To Upstream (Sink)
    wire                         us_fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] us_fcp_vc;
    wire [STAT_WIDTH-1:0]        us_fcp_fccl;
    wire [STAT_WIDTH-1:0]        us_fcp_qlen;
    wire [STAT_WIDTH-1:0]        us_fcp_fccr;

    // --- New AXIS FCP Link ---
    wire [127:0]                 axis_fcp_tdata;
    wire                         axis_fcp_tvalid;
    wire                         axis_fcp_tready;
// 2. Adapter: Discrete -> AXIS
    fcp_source_adapter #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .AXIS_WIDTH(128)
    ) fcp_packer (
        .fcp_valid(ds_fcp_valid),
        .fcp_vc(ds_fcp_vc),
        .fcp_fccl(ds_fcp_fccl),
        .fcp_qlen(ds_fcp_qlen),
        .fcp_fccr(ds_fcp_fccr),
        
        .m_axis_fcp_tdata(axis_fcp_tdata),
        .m_axis_fcp_tvalid(axis_fcp_tvalid),
        .m_axis_fcp_tready(axis_fcp_tready)
    );

    // 3. Adapter: AXIS -> Discrete
    fcp_sink_adapter #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .AXIS_WIDTH(128)
    ) fcp_unpacker (
        .s_axis_fcp_tdata(axis_fcp_tdata),
        .s_axis_fcp_tvalid(axis_fcp_tvalid),
        .s_axis_fcp_tready(axis_fcp_tready),
        
        .fcp_valid(us_fcp_valid),
        .fcp_vc(us_fcp_vc),
        .fcp_fccl(us_fcp_fccl),
        .fcp_qlen(us_fcp_qlen),
        .fcp_fccr(us_fcp_fccr)
    );
    // ========================================================================
    // 3. Drive Debug Outputs
    // ========================================================================
    // Simply tap the wires
    assign dbg_link_tdata  = link_tdata;
    assign dbg_link_tvalid = link_tvalid;
    assign dbg_link_tlast  = link_tlast;
    assign dbg_link_tkeep  = link_tkeep;
    assign dbg_link_tready = link_tready;
    assign dbg_link_fcp_valid = us_fcp_valid; 
    assign dbg_link_fcp_vc    = us_fcp_vc;    
    assign dbg_link_fcp_fccl  = us_fcp_fccl;  
    assign dbg_link_fcp_qlen  = us_fcp_qlen;  
    assign dbg_link_fcp_fccr  = us_fcp_fccr;  

endmodule

`resetall