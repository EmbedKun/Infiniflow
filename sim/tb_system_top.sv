`timescale 1ns / 1ps

module tb_system_top;

    // ========================================================================
    // 1. Parameter Definitions
    // ========================================================================
    parameter QUEUE_INDEX_WIDTH = 10;  // 64 Queues (2^18 is actually much more, keeping your param)
    parameter DATA_WIDTH        = 64;
    parameter BUFFER_ADDR_WIDTH = 10; 
    parameter DRAIN_RATIO_M     = 8;  
    parameter STAT_WIDTH        = 32;

    parameter REQ_TAG_WIDTH = 8;
    parameter LEN_WIDTH     = 16;
    parameter OP_TABLE_SIZE = 16;
    parameter PKT_LEN_BYTES = 64;
    
    parameter ACTIVE_FIFO_DEPTH = 1024;
    parameter QMAX = 6;
    parameter QMIN = 3;

    localparam QUEUE_COUNT = 2**QUEUE_INDEX_WIDTH;

    // ========================================================================
    // 2. Signals & DUT Instantiation
    // ========================================================================
    // --- Clock and Reset Changes ---
    reg  clk_p;                 // Differential Clock P
    wire clk_n;                 // Differential Clock N
    reg  rst_n;                 // Active-Low Reset
    
    reg  enable;
    
    // Unused control inputs tied to 0
    wire [QUEUE_INDEX_WIDTH-1:0] stop_queue_idx = 0;
    wire                         stop_cmd_valid = 0;

    // FCP Signals
    wire                         fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] fcp_vc;
    wire [31:0]                  fcp_fccl;
    wire [31:0]                  fcp_qlen;
    wire [31:0]                  fcp_fccr;

    // Link Interface Debug
    wire [DATA_WIDTH-1:0]        dbg_link_tdata;
    wire                         dbg_link_tvalid;
    wire                         dbg_link_tlast;
    wire [DATA_WIDTH/8-1:0]      dbg_link_tkeep;
    wire                         dbg_link_tready;
    wire                         dbg_injector_active;
    
    // Output Link
    wire [DATA_WIDTH-1:0]        out_link_tdata;
    wire                         out_link_tvalid;
    
    // Stats
    wire [STAT_WIDTH-1:0]        dbg_buffer_free_count;
    wire [STAT_WIDTH-1:0]        dbg_total_rx_count;
    wire [63:0]                  m_axis_tx_pkt_count;
    wire [31:0]                  m_axis_vc_tx_count;

    system_top #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .DRAIN_RATIO_M(DRAIN_RATIO_M),
        .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .LEN_WIDTH(LEN_WIDTH),
        .OP_TABLE_SIZE(OP_TABLE_SIZE),
        .PKT_LEN_BYTES(PKT_LEN_BYTES),
        .ACTIVE_FIFO_DEPTH(ACTIVE_FIFO_DEPTH),
        .QMAX(QMAX),
        .QMIN(QMIN)
    ) dut (
        .clk_p(clk_p),           // Updated
        .clk_n(clk_n),           // Updated
        .rst_n(rst_n),           // Updated
        .enable(enable),
        .stop_queue_idx(stop_queue_idx),
        .stop_cmd_valid(stop_cmd_valid),
        
        .dbg_link_fcp_valid(fcp_valid),
        .dbg_link_fcp_vc(fcp_vc),
        .dbg_link_fcp_fccl(fcp_fccl),
        .dbg_link_fcp_qlen(fcp_qlen),
        .dbg_link_fcp_fccr(fcp_fccr),

        .dbg_link_tdata(dbg_link_tdata),
        .dbg_link_tvalid(dbg_link_tvalid),
        .dbg_link_tlast(dbg_link_tlast),
        .dbg_link_tkeep(dbg_link_tkeep),
        .dbg_link_tready(dbg_link_tready),
        .dbg_injector_active(dbg_injector_active),
        
        .out_link_tdata(out_link_tdata),
        .out_link_tvalid(out_link_tvalid),
        .dbg_buffer_free_count(dbg_buffer_free_count),
        .dbg_total_rx_count(dbg_total_rx_count),

        .m_axis_tx_pkt_count(m_axis_tx_pkt_count),
        .m_axis_vc_tx_count(m_axis_vc_tx_count)
    );

    // ========================================================================
    // 3. Differential Clock Generation
    // ========================================================================
    initial begin
        clk_p = 0;
        forever #5 clk_p = ~clk_p; // 100MHz
    end

    // clk_n is always the inverse of clk_p
    assign clk_n = ~clk_p;

    // ========================================================================
    // 4. Main Test Procedure
    // ========================================================================
    initial begin
        // --- Init ---
        rst_n = 0;             // Reset asserted (Low)
        enable = 0;

        // --- Reset Sequence ---
        repeat (20) @(posedge clk_p);
        rst_n = 1;             // Reset released (High)
        $display("[%0t] Reset Released. Waiting for Auto-Activation...", $time);

        // --- Wait for Initialization ---
        repeat (QUEUE_COUNT + 100) @(posedge clk_p);

        // --- Start Injection ---
        @(posedge clk_p);
        enable = 1;
        $display("[%0t] Traffic Injection Enabled.", $time);

        // --- Run Simulation ---
        repeat (5000) @(posedge clk_p);

        // --- Final Stats ---
        $display("---------------------------------------------------");
        $display("[%0t] Simulation Finished.", $time);
        $display("Total Packets Sent (Injector): %0d", m_axis_tx_pkt_count);
        $display("Total Packets Rcvd (Switch)  : %0d", dbg_total_rx_count);
        $display("Switch Buffer Free           : %0d", dbg_buffer_free_count);
        $display("---------------------------------------------------");
        
        $finish;
    end

    // ========================================================================
    // 5. Simple Monitors (Updated to clk_p)
    // ========================================================================

    always @(posedge clk_p) begin
        if (fcp_valid) begin
            $display("[%0t] FCP > VC:%0d | QLen:%0d | FCCL:%0d | FCCR:%0d", 
                     $time, fcp_vc, fcp_qlen, fcp_fccl, fcp_fccr);
        end
    end

    reg bp_warned = 0;
    always @(posedge clk_p) begin
        if (dbg_link_tvalid && !dbg_link_tready && !bp_warned) begin
            $display("[%0t] NOTICE: Backpressure Detected (Switch Buffer Full)", $time);
            bp_warned = 1;
        end
    end

    always @(posedge clk_p) begin
        if (m_axis_tx_pkt_count > 0 && m_axis_tx_pkt_count % 100 == 0 && 
            dbg_link_tvalid && dbg_link_tready && dbg_link_tlast) begin
            // Heartbeat can be added here
        end
    end

endmodule