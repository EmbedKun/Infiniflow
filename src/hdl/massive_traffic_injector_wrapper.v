// SPDX-License-Identifier: BSD-2-Clause-Views
// Author: mkxue-FNIL
// Wrapper for timing analysis in Vivado

`timescale 1ns / 1ps
`default_nettype none

module massive_traffic_injector_wrapper #
(
    // Keep parameters same as original to test real logic
    parameter QUEUE_INDEX_WIDTH = 16, 
    parameter REQ_TAG_WIDTH = 8,
    parameter LEN_WIDTH = 16,
    parameter OP_TABLE_SIZE = 16,
    parameter PIPELINE = 3+(QUEUE_INDEX_WIDTH > 12 ? QUEUE_INDEX_WIDTH-12 : 0),
    parameter BUFFER_ADDR_WIDTH = 14, 
    parameter DATA_WIDTH = 512,        
    parameter PKT_LEN_BYTES = 1536,      
    parameter QMAX = 6,
    parameter QMIN = 3,
    parameter IGNORE_FCP_MODE = 1
)
(
    input  wire         clk,
    input  wire         rst,
    
    // Minimal Inputs to keep logic toggleable
    input  wire         enable,
    input  wire         m_axis_pkt_tready, // Allow backpressure test
    
    // Dummy Output to prevent logic optimization
    output reg          optimization_preventer
);

    // ========================================================================
    // 1. Signal Tie-offs (Fixed inputs)
    // ========================================================================
    wire [QUEUE_INDEX_WIDTH-1:0] stop_queue_idx = 0;
    wire                         stop_cmd_valid = 0;
    
    // FCP inputs tied to 0 (No flow control updates in this test)
    wire                         fcp_valid = 0;
    wire [QUEUE_INDEX_WIDTH-1:0] fcp_vc    = 0;
    wire [31:0]                  fcp_fccl  = 0;
    wire [31:0]                  fcp_qlen  = 0;
    wire [31:0]                  fcp_fccr  = 0;

    // ========================================================================
    // 2. Instantiate DUT (Device Under Test)
    // ========================================================================
    
    // DUT Outputs
    wire [DATA_WIDTH-1:0]   m_axis_pkt_tdata;
    wire                    m_axis_pkt_tvalid;
    wire                    m_axis_pkt_tlast;
    wire [DATA_WIDTH/8-1:0] m_axis_pkt_tkeep;
    
    wire                    scheduler_active;
    wire [63:0]             m_axis_tx_pkt_count;
    wire [31:0]             m_axis_vc_tx_count;

    massive_traffic_injector #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .LEN_WIDTH(LEN_WIDTH),
        .OP_TABLE_SIZE(OP_TABLE_SIZE),
        .PIPELINE(PIPELINE),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .PKT_LEN_BYTES(PKT_LEN_BYTES),
        .QMAX(QMAX),
        .QMIN(QMIN),
        .IGNORE_FCP_MODE(IGNORE_FCP_MODE)
    ) dut (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .stop_queue_idx(stop_queue_idx),
        .stop_cmd_valid(stop_cmd_valid),
        
        .m_axis_pkt_tdata(m_axis_pkt_tdata),
        .m_axis_pkt_tvalid(m_axis_pkt_tvalid),
        .m_axis_pkt_tlast(m_axis_pkt_tlast),
        .m_axis_pkt_tkeep(m_axis_pkt_tkeep),
        .m_axis_pkt_tready(m_axis_pkt_tready),
        
        .scheduler_active(scheduler_active),
        .m_axis_tx_pkt_count(m_axis_tx_pkt_count),
        .m_axis_vc_tx_count(m_axis_vc_tx_count),
        
        .fcp_valid(fcp_valid),
        .fcp_vc(fcp_vc),
        .fcp_fccl(fcp_fccl),
        .fcp_qlen(fcp_qlen),
        .fcp_fccr(fcp_fccr)
    );

    // ========================================================================
    // 3. Logic Preservation Chain (Reduction XOR)
    // ========================================================================
    // We XOR all output bits together. 
    // If any logic cone contributing to any output bit is removed, the result changes.
    // Vivado is smart enough to know this and will preserve the logic cones.
    
    always @(posedge clk) begin
        if (rst) begin
            optimization_preventer <= 0;
        end else begin
            // Registering the result adds a Flip-Flop endpoint for timing analysis
            optimization_preventer <= ^m_axis_pkt_tdata ^ 
                                      m_axis_pkt_tvalid ^ 
                                      m_axis_pkt_tlast ^ 
                                      ^m_axis_pkt_tkeep ^ 
                                      scheduler_active ^ 
                                      ^m_axis_tx_pkt_count ^ 
                                      ^m_axis_vc_tx_count;
        end
    end

endmodule