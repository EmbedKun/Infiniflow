`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////

// fcp_adapters.v
module fcp_source_adapter #
(
    parameter QUEUE_INDEX_WIDTH = 15,
    parameter STAT_WIDTH = 32,
    parameter AXIS_WIDTH = 512
)
(
    // Discrete FCP Inputs
    input  wire                          fcp_valid,
    input  wire [QUEUE_INDEX_WIDTH-1:0]  fcp_vc,
    input  wire [STAT_WIDTH-1:0]         fcp_fccl,
    input  wire [STAT_WIDTH-1:0]         fcp_qlen,
    input  wire [STAT_WIDTH-1:0]         fcp_fccr,

    // AXIS Output
    output wire [AXIS_WIDTH-1:0]         m_axis_fcp_tdata,
    output wire                          m_axis_fcp_tvalid,
    input  wire                          m_axis_fcp_tready // Optional support
);
    // Packing format: {Padding, VC, FCCR, QLEN, FCCL} 
    // Adjust order as preferred. LSB is usually consumer-specific.
    // Let's use: [VC, FCCR, QLEN, FCCL]
    // FCCL [31:0]
    // QLEN [63:32]
    // FCCR [95:64]
    // VC   [96+QUEUE_INDEX_WIDTH-1 : 96]
    
    assign m_axis_fcp_tvalid = fcp_valid;
    
    assign m_axis_fcp_tdata[31:0] = fcp_fccl;
    assign m_axis_fcp_tdata[63:32] = fcp_qlen;
    assign m_axis_fcp_tdata[95:64] = fcp_fccr;
    assign m_axis_fcp_tdata[96 +: QUEUE_INDEX_WIDTH] = fcp_vc;
    assign m_axis_fcp_tdata[AXIS_WIDTH-1 : 96+QUEUE_INDEX_WIDTH] = 0; // Padding

endmodule