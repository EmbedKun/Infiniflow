// SPDX-License-Identifier: BSD-2-Clause-Views
// Author: mkxue-FNIL
`timescale 1ns / 1ps

module massive_traffic_injector #
(
    // Queue Configuration
    parameter QUEUE_INDEX_WIDTH = 16, // 2^18 = 262,144 Queues
    parameter REQ_TAG_WIDTH = 8,
    parameter LEN_WIDTH = 16,
    parameter OP_TABLE_SIZE = 16,
    parameter PIPELINE = 3+(QUEUE_INDEX_WIDTH > 12 ? QUEUE_INDEX_WIDTH-12 : 0),
    parameter BUFFER_ADDR_WIDTH = 14, 
    // Packet Configuration
    parameter DATA_WIDTH = 512,        // Output Packet Width
    parameter PKT_LEN_BYTES = 1536,      // Simulated Packet Length
    parameter QMAX = 6,
    parameter QMIN = 3,
    parameter IGNORE_FCP_MODE = 1
)
(
    input  wire                          clk,
    input  wire                          rst,

    // ---------------------------------------------------------
    // Control Interface
    // ---------------------------------------------------------
    input  wire                          enable,          // Scheduler Enable
    input  wire [QUEUE_INDEX_WIDTH-1:0]  stop_queue_idx,  // Control: Queue to stop
    input  wire                          stop_cmd_valid,  // Control: Execute stop

    // ---------------------------------------------------------
    // Packet Output Interface (AXI Stream)
    // ---------------------------------------------------------
    output wire [DATA_WIDTH-1:0]         m_axis_pkt_tdata,
    output wire                          m_axis_pkt_tvalid,
    output wire                          m_axis_pkt_tlast,
    output wire [DATA_WIDTH/8-1:0]       m_axis_pkt_tkeep,
    input  wire                          m_axis_pkt_tready,

    // ---------------------------------------------------------
    // Debug / Status
    // ---------------------------------------------------------
    output wire                          scheduler_active,

    // Statistics Outputs
    output wire  [63:0]                   m_axis_tx_pkt_count,
    output wire  [31:0]                   m_axis_vc_tx_count, 

    // FCP Input Interface
    input  wire                          fcp_valid,
    input  wire [QUEUE_INDEX_WIDTH-1:0]  fcp_vc,
    input  wire [31:0]                   fcp_fccl,
    input  wire [31:0]                   fcp_qlen,
    input  wire [31:0]                   fcp_fccr
);

    // ========================================================================
    // Internal Signals Wiring
    // ========================================================================

    // Request Interface (Scheduler -> Simulator)
    wire [QUEUE_INDEX_WIDTH-1:0]  axis_tx_req_queue;
    wire [REQ_TAG_WIDTH-1:0]      axis_tx_req_tag;
    wire                          axis_tx_req_valid;
    wire                          axis_tx_req_ready;

    // Status Interface (Simulator -> Scheduler)
    wire [LEN_WIDTH-1:0]          axis_tx_status_len;
    wire [REQ_TAG_WIDTH-1:0]      axis_tx_status_tag;
    wire                          axis_tx_status_valid;

    // Doorbell Interface (Simulator -> Scheduler)
    wire [QUEUE_INDEX_WIDTH-1:0]  axis_doorbell_queue;
    wire                          axis_doorbell_valid;

    // ========================================================================
    // 1. Instantiate Transmit Scheduler
    // ========================================================================
    tx_scheduler_rr #(
        .LEN_WIDTH(LEN_WIDTH),
        .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .OP_TABLE_SIZE(OP_TABLE_SIZE),
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .PIPELINE(PIPELINE)
    ) scheduler_inst (
        .clk(clk),
        .rst(rst),

        // Transmit Request Output
        .m_axis_tx_req_queue(axis_tx_req_queue),
        .m_axis_tx_req_tag(axis_tx_req_tag),
        .m_axis_tx_req_valid(axis_tx_req_valid),
        .m_axis_tx_req_ready(axis_tx_req_ready),

        // Transmit Status Input
        .s_axis_tx_req_status_len(axis_tx_status_len),
        .s_axis_tx_req_status_tag(axis_tx_status_tag),
        .s_axis_tx_req_status_valid(axis_tx_status_valid),

        // Doorbell Input
        .s_axis_doorbell_queue(axis_doorbell_queue),
        .s_axis_doorbell_valid(axis_doorbell_valid),

        // Control
        .enable(enable),
        .active(scheduler_active)
    );

    // ========================================================================
    // 2. Instantiate Packet Send Simulator
    // ========================================================================
    pkt_send_simulator #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .LEN_WIDTH(LEN_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .PKT_LEN_BYTES(PKT_LEN_BYTES),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .QMAX(QMAX),
        .QMIN(QMIN),
        .IGNORE_FCP_MODE(IGNORE_FCP_MODE)
    ) simulator_inst (
        .clk(clk),
        .rst(rst),

        // Request Input (From Scheduler)
        .s_axis_tx_req_queue(axis_tx_req_queue),
        .s_axis_tx_req_tag(axis_tx_req_tag),
        .s_axis_tx_req_valid(axis_tx_req_valid),
        .s_axis_tx_req_ready(axis_tx_req_ready),

        // Status Output (To Scheduler)
        .m_axis_tx_req_status_len(axis_tx_status_len),
        .m_axis_tx_req_status_tag(axis_tx_status_tag),
        .m_axis_tx_req_status_valid(axis_tx_status_valid),

        // Doorbell Output (To Scheduler)
        .m_axis_doorbell_queue(axis_doorbell_queue),
        .m_axis_doorbell_valid(axis_doorbell_valid),

        // Control Input (Stop Queue)
        .stop_queue_idx(stop_queue_idx),
        .stop_cmd_valid(stop_cmd_valid),

        // Packet Output (To External World / Monitor)
        .m_axis_pkt_tdata(m_axis_pkt_tdata),
        .m_axis_pkt_tvalid(m_axis_pkt_tvalid),
        .m_axis_pkt_tlast(m_axis_pkt_tlast),
        .m_axis_pkt_tkeep(m_axis_pkt_tkeep),
        .m_axis_pkt_tready(m_axis_pkt_tready),
        
        // Statistics Outputs
        .m_axis_tx_pkt_count(m_axis_tx_pkt_count),
        .m_axis_vc_tx_count (m_axis_vc_tx_count),

        // FCP Input
        .fcp_valid(fcp_valid),
        .fcp_vc(fcp_vc),
        .fcp_fccl(fcp_fccl),
        .fcp_qlen(fcp_qlen),
        .fcp_fccr(fcp_fccr)
    );

endmodule

`resetall