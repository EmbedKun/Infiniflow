`timescale 1ns / 1ps
module fcp_sink_adapter #
(
    parameter QUEUE_INDEX_WIDTH = 15,
    parameter STAT_WIDTH = 32,
    parameter AXIS_WIDTH = 512
)
(
    // AXIS Input
    input  wire [AXIS_WIDTH-1:0]         s_axis_fcp_tdata,
    input  wire                          s_axis_fcp_tvalid,
    output wire                          s_axis_fcp_tready,

    // Discrete FCP Outputs
    output wire                          fcp_valid,
    output wire [QUEUE_INDEX_WIDTH-1:0]  fcp_vc,
    output wire [STAT_WIDTH-1:0]         fcp_fccl,
    output wire [STAT_WIDTH-1:0]         fcp_qlen,
    output wire [STAT_WIDTH-1:0]         fcp_fccr
);

    assign s_axis_fcp_tready = 1'b1; // Always ready to receive updates

    assign fcp_valid = s_axis_fcp_tvalid;
    assign fcp_fccl  = s_axis_fcp_tdata[31:0];
    assign fcp_qlen  = s_axis_fcp_tdata[63:32];
    assign fcp_fccr  = s_axis_fcp_tdata[95:64];
    assign fcp_vc    = s_axis_fcp_tdata[96 +: QUEUE_INDEX_WIDTH];

endmodule