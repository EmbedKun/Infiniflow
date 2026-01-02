`timescale 1ns / 1ps

module tb_tx_scheduler_rr;

    // Parameters
    parameter QUEUE_INDEX_WIDTH = 8; // 13-bit = 8192 Queues
    parameter REQ_TAG_WIDTH = 8;
    parameter OP_TABLE_SIZE = 16;
    parameter LEN_WIDTH = 16;
//    parameter PIPELINE = 3 + (QUEUE_INDEX_WIDTH > 12 ? QUEUE_INDEX_WIDTH - 12 : 0);
    parameter PIPELINE = 2;
    // =========================================================
    // 1. Signal Declaration (Added status-related regs)
    // =========================================================
    reg clk;
    reg rst;
    reg enable;
    
    // Doorbell Interface
    reg [QUEUE_INDEX_WIDTH-1:0] s_axis_doorbell_queue;
    reg s_axis_doorbell_valid;
    
    // [Added] Status Feedback Interface (Driven by TB, hence reg)
    reg [LEN_WIDTH-1:0]     s_axis_tx_req_status_len;
    reg [REQ_TAG_WIDTH-1:0] s_axis_tx_req_status_tag;
    reg                     s_axis_tx_req_status_valid;

    // Tx Request Interface (DUT Output, hence wire)
    wire [QUEUE_INDEX_WIDTH-1:0] m_axis_tx_req_queue;
    wire [REQ_TAG_WIDTH-1:0]     m_axis_tx_req_tag;
    wire                         m_axis_tx_req_valid;
    reg                          m_axis_tx_req_ready; // Driven by TB
    
    wire active;

    integer i; // Loop counter

    // =========================================================
    // 2. Module Instantiation (Completed port connections)
    // =========================================================
    tx_scheduler_rr #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .OP_TABLE_SIZE(OP_TABLE_SIZE),
        .LEN_WIDTH(LEN_WIDTH),
        .PIPELINE(PIPELINE)
    ) uut (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        
        // Doorbell
        .s_axis_doorbell_queue(s_axis_doorbell_queue),
        .s_axis_doorbell_valid(s_axis_doorbell_valid),
        
        // Tx Request
        .m_axis_tx_req_queue(m_axis_tx_req_queue),
        .m_axis_tx_req_tag(m_axis_tx_req_tag),
        .m_axis_tx_req_valid(m_axis_tx_req_valid),
        .m_axis_tx_req_ready(m_axis_tx_req_ready),
        
        // [Added] Status Loopback Interface
        .s_axis_tx_req_status_len(s_axis_tx_req_status_len),
        .s_axis_tx_req_status_tag(s_axis_tx_req_status_tag),
        .s_axis_tx_req_status_valid(s_axis_tx_req_status_valid),
        
        .active(active)
    );

    // =========================================================
    // 3. Clock Generation
    // =========================================================
    always begin
        #5 clk = ~clk;  // 100 MHz clock
    end

    // =========================================================
    // 4. [Independent Always Block] Status Loopback Logic
    //    Simulates PHY transmission completion, returns Tag 
    //    immediately to release scheduler resources
    // =========================================================
    always @(posedge clk) begin
        if (rst) begin
            s_axis_tx_req_status_valid <= 0;
            s_axis_tx_req_status_tag   <= 0;
            s_axis_tx_req_status_len   <= 0;
        end else begin
            // Default low, generate pulse
            s_axis_tx_req_status_valid <= 0;

            // If scheduler issued a packet and we accepted it
            if (m_axis_tx_req_valid && m_axis_tx_req_ready) begin
                s_axis_tx_req_status_valid <= 1;
                s_axis_tx_req_status_tag   <= m_axis_tx_req_tag; // Return Tag
                s_axis_tx_req_status_len   <= 64; // Assume packet length 64
            end
        end
    end

    // =========================================================
    // 5. Main Stimulus Logic (Initial Block)
    // =========================================================
    initial begin
        // Initialize signals
        clk = 0;
        rst = 1;
        enable = 0;
        s_axis_doorbell_valid = 0;
        s_axis_doorbell_queue = 0;
        m_axis_tx_req_ready = 1; // Receiver always Ready, to test max throughput

        // Status signals are already driven in the always block, 
        // no initialization needed here (or covered by reset)
        
        // Reset sequence
        #100;
        rst = 0;
        #100;
        enable = 1;

        // -----------------------------------------------------
        // Critical Wait: Wait for RTL internal RAM initialization to complete
        // 13-bit index = 8192 cycles. 
        // We wait 200,000ns (20,000 cycles) to ensure safety
        // -----------------------------------------------------
        $display("Time %t: Waiting for Scheduler Hardware Init...", $time);
        #200000; 
        $display("Time %t: Init done, starting doorbells...", $time);

        // Send 6000 doorbells, activate 6000 different queues
        for (i = 0; i < 6000; i = i + 1) begin
            s_axis_doorbell_valid = 1;
            s_axis_doorbell_queue = i; // Q0, Q1 ... Q5999
            @(posedge clk);
        end
        
        // Stop sending doorbells
        s_axis_doorbell_valid = 0;
        
        // At this point, you should see m_axis_tx_req_valid becoming 101010... on the waveform
        
        // Run for a while then finish
        #20000;
        $finish;
    end

endmodule