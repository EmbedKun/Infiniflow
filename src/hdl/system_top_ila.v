// SPDX-License-Identifier: BSD-2-Clause-Views
// Top Level Integration Wrapper with ILA and CMAC
// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none

module system_top_ila #
(
    // ---------------------------------------------------------
    // Global Configuration
    // ---------------------------------------------------------
    parameter QUEUE_INDEX_WIDTH = 16, // 262,144 Queues
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
    parameter BUFFER_ADDR_WIDTH = 10, // 16K Cells
    parameter STAT_WIDTH        = 32,
    parameter DRAIN_RATIO_M     = 8,  // Drain Rate 1:1.5
    parameter QMAX = 6,
    parameter QMIN = 3
)
(
    input  wire                          clk_p,    
    input  wire                          clk_n,    
    input  wire                          rst_n
//    // Optional: External Stop Control (Can be tied to VIO if needed, or keeping input pins)
//    input  wire [QUEUE_INDEX_WIDTH-1:0]  stop_queue_idx, 
//    input  wire                          stop_cmd_valid
);

    localparam QUEUE_COUNT = 2**QUEUE_INDEX_WIDTH;

    // ========================================================================
    // 1. Clock & Reset Management
    // ========================================================================
    wire clk;
    wire rst;

    // Differential Clock Buffer
    IBUFDS clk_ibufds (
        .I (clk_p),
        .IB(clk_n),
        .O (clk)
    );

    // Active High Reset
    assign rst = ~rst_n;

    // ========================================================================
    // 2. Internal Enable Generation
    // ========================================================================
    reg [31:0] enable_cnt;
    reg        enable_reg;
    wire       enable_internal;

    always @(posedge clk) begin
        if (rst) begin
            enable_cnt <= 0;
            enable_reg <= 0;
        end else begin
            // Wait for QUEUE_COUNT + 100 cycles before enabling
            if (enable_cnt < (QUEUE_COUNT + 100)) begin
                enable_cnt <= enable_cnt + 1;
                enable_reg <= 0;
            end else begin
                // Latch High permanently
                enable_reg <= 1;
            end
        end
    end
    assign enable_internal = enable_reg;

    // ========================================================================
    // 3. Internal Interconnect Signals
    // ========================================================================
    wire [DATA_WIDTH-1:0]        link_tdata;
    wire                         link_tvalid;
    wire                         link_tlast;
    wire [DATA_WIDTH/8-1:0]      link_tkeep;
    wire                         link_tready;
    
    wire                         fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] fcp_vc;
    wire [STAT_WIDTH-1:0]        fcp_fccl; 
    wire [STAT_WIDTH-1:0]        fcp_qlen; 
    wire [STAT_WIDTH-1:0]        fcp_fccr; 

    // Status Signals (Formerly Outputs)
    wire                         dbg_injector_active;
    wire [63:0]                  m_axis_tx_pkt_count;
    wire [31:0]                  m_axis_vc_tx_count;
    wire [DATA_WIDTH-1:0]        out_link_tdata;
    wire                         out_link_tvalid;
    wire [STAT_WIDTH-1:0]        dbg_buffer_free_count;
    wire [STAT_WIDTH-1:0]        dbg_total_rx_count;

    // ========================================================================
    // 4. Instantiate Modules
    // ========================================================================
    
    // Upstream: Traffic Injector
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
        .clk(clk),
        .rst(rst),
        .enable(enable_internal), // Using internal enable
        
        .m_axis_pkt_tdata(link_tdata),
        .m_axis_pkt_tvalid(link_tvalid),
        .m_axis_pkt_tlast(link_tlast),
        .m_axis_pkt_tkeep(link_tkeep),
        .m_axis_pkt_tready(link_tready),
        
        .scheduler_active(dbg_injector_active),
        .m_axis_tx_pkt_count(m_axis_tx_pkt_count),
        .m_axis_vc_tx_count (m_axis_vc_tx_count),
        
        .fcp_valid(us_fcp_valid),
        .fcp_vc(us_fcp_vc),
        .fcp_fccl(us_fcp_fccl),
        .fcp_qlen(us_fcp_qlen),
        .fcp_fccr(us_fcp_fccr)
    );

    // Downstream: Switch Model
    downstream_switch_model_v2 #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .STAT_WIDTH(STAT_WIDTH),
        .DRAIN_RATIO_M(DRAIN_RATIO_M)
    ) downstream_inst (
        .clk(clk),
        .rst(rst),
        
        .s_axis_pkt_tdata(link_tdata),
        .s_axis_pkt_tvalid(link_tvalid),
        .s_axis_pkt_tlast(link_tlast),
        .s_axis_pkt_tkeep(link_tkeep),
        .s_axis_pkt_tready(link_tready),
        
        .m_axis_pkt_tdata(out_link_tdata),
        .m_axis_pkt_tvalid(out_link_tvalid),
        
        .fcp_valid(ds_fcp_valid),
        .fcp_vc(ds_fcp_vc),
        .fcp_fccl(ds_fcp_fccl),
        .fcp_qlen(ds_fcp_qlen),
        .fcp_fccr(ds_fcp_fccr),
        
        .dbg_buffer_free_count(dbg_buffer_free_count),
        .dbg_total_rx_count(dbg_total_rx_count)
    );

    // ========================================================================
    // 5. ILA Instantiation (Debug)
    // ========================================================================
    // NOTE: You must generate an ILA IP in Vivado named 'ila_0'
    // Recommended Probe Configuration:
    // Probe 0: [63:0] Link Data
    // Probe 1: [0:0]  Link Valid
    // Probe 2: [0:0]  Link Ready
    // Probe 3: [0:0]  FCP Valid
    // Probe 4: [17:0] FCP VC (QUEUE_INDEX_WIDTH)
    // Probe 5: [31:0] FCP QLEN
    // Probe 6: [63:0] Tx Pkt Count
    // Probe 7: [31:0] Rx Pkt Count
    // Probe 8: [31:0] Buffer Free
    // Probe 9: [0:0]  Enable Internal

    ila_0 u_ila (
        .clk(clk),
        .probe0(link_tdata),           // 64-bit
        .probe1(link_tvalid),          // 1-bit
        .probe2(link_tready),          // 1-bit
        .probe3(ds_fcp_valid),            // 1-bit
        .probe4(ds_fcp_vc),               // 18-bit (Resize if QUEUE_INDEX_WIDTH changes)
        .probe5(ds_fcp_qlen),             // 32-bit
        .probe6(m_axis_tx_pkt_count),  // 64-bit
        .probe7(dbg_total_rx_count),   // 32-bit
        .probe8(dbg_buffer_free_count),// 32-bit
        .probe9(enable_internal),      // 1-bit
        .probe10(clk),
        .probe11(rst),
        .probe12(ds_fcp_fccl),
        .probe13(ds_fcp_fccr),
        .probe14(us_fcp_valid),            // 1-bit
        .probe15(us_fcp_vc),               // 18-bit (Resize if QUEUE_INDEX_WIDTH changes)
        .probe16(us_fcp_qlen),             // 32-bit
        .probe17(us_fcp_fccl),               // 18-bit (Resize if QUEUE_INDEX_WIDTH changes)
        .probe18(us_fcp_fccr)              // 32-bit
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

endmodule
`resetall