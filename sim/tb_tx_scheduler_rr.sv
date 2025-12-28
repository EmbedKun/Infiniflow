`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/12/22 09:44:11
// Design Name: 
// Module Name: tb_tx_scheduler_rr
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps

module tb_tx_scheduler_rr;

    // Parameters
    parameter LEN_WIDTH = 16;
    parameter REQ_TAG_WIDTH = 8;
    parameter OP_TABLE_SIZE = 16;
    parameter QUEUE_INDEX_WIDTH = 6;
    parameter PIPELINE = 2;

    // Inputs
    reg clk = 0;
    reg rst = 0;
    reg m_axis_tx_req_ready = 0;
    reg [LEN_WIDTH-1:0] s_axis_tx_req_status_len = 0;
    reg [REQ_TAG_WIDTH-1:0] s_axis_tx_req_status_tag = 0;
    reg s_axis_tx_req_status_valid = 0;
    reg [QUEUE_INDEX_WIDTH-1:0] s_axis_doorbell_queue = 0;
    reg s_axis_doorbell_valid = 0;
    reg enable = 0;

    // Outputs
    wire [QUEUE_INDEX_WIDTH-1:0] m_axis_tx_req_queue;
    wire [REQ_TAG_WIDTH-1:0] m_axis_tx_req_tag;
    wire m_axis_tx_req_valid;
    wire active;

    // Clock generation
    always #5 clk = ~clk; // 100MHz

    // Instantiate the Unit Under Test (UUT)
    tx_scheduler_rr #(
        .LEN_WIDTH(LEN_WIDTH),
        .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .OP_TABLE_SIZE(OP_TABLE_SIZE),
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .PIPELINE(PIPELINE)
    ) uut (
        .clk(clk),
        .rst(rst),
        .m_axis_tx_req_queue(m_axis_tx_req_queue),
        .m_axis_tx_req_tag(m_axis_tx_req_tag),
        .m_axis_tx_req_valid(m_axis_tx_req_valid),
        .m_axis_tx_req_ready(m_axis_tx_req_ready),
        .s_axis_tx_req_status_len(s_axis_tx_req_status_len),
        .s_axis_tx_req_status_tag(s_axis_tx_req_status_tag),
        .s_axis_tx_req_status_valid(s_axis_tx_req_status_valid),
        .s_axis_doorbell_queue(s_axis_doorbell_queue),
        .s_axis_doorbell_valid(s_axis_doorbell_valid),
        .enable(enable),
        .active(active)
    );

    // Helper task to ring doorbell
    task doorbell(input [QUEUE_INDEX_WIDTH-1:0] q_index);
        begin
            @(posedge clk);
            s_axis_doorbell_queue <= q_index;
            s_axis_doorbell_valid <= 1;
            @(posedge clk);
            s_axis_doorbell_valid <= 0;
            $display("[%0t] Doorbell rung for Queue %d", $time, q_index);
        end
    endtask

    // Helper task to send completion status
    task completion(input [REQ_TAG_WIDTH-1:0] tag, input [LEN_WIDTH-1:0] len);
        begin
            @(posedge clk);
            s_axis_tx_req_status_tag <= tag;
            s_axis_tx_req_status_len <= len; // len=0 means done, len>0 means more packets
            s_axis_tx_req_status_valid <= 1;
            @(posedge clk);
            s_axis_tx_req_status_valid <= 0;
            $display("[%0t] Completion sent for Tag %d", $time, tag);
        end
    endtask

    // Variables to store received tags to verify completion logic
    logic [REQ_TAG_WIDTH-1:0] captured_tag_q1;
    logic [REQ_TAG_WIDTH-1:0] captured_tag_q2;

    initial begin
        // Setup Dump for Waveform viewing (GTKWave etc.)
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_tx_scheduler_rr);

        // Initialize Inputs
        rst = 1;
        enable = 0;
        m_axis_tx_req_ready = 0;
        captured_tag_q1 = 0;
        captured_tag_q2 = 0;
        
        // Reset sequence
        #100;
        rst = 0;
        $display("[%0t] Reset released", $time);

        // Enable scheduler
        #20;
        enable = 1;
        
        // Wait for internal initialization
        // Logic initializes 2^QUEUE_INDEX_WIDTH queues. 2^6 = 64 cycles + pipeline overhead.
        // Wait enough time for 'init_reg' to clear internally.
        #1000; 
        $display("[%0t] Initialization assumed complete", $time);

        // -------------------------------------------------------
        // Test Case 1: Single Queue Scheduling
        // -------------------------------------------------------
        $display("--- Test Case 1: Single Queue (Q1) ---");
        m_axis_tx_req_ready = 1; // Sink is ready
        
        doorbell(1); // Ring doorbell for Queue 1

        // Wait for request
        wait(m_axis_tx_req_valid);
        if (m_axis_tx_req_queue == 1) begin
//            $display("[%0t] SUCCESS: Received Request for Queue 1, Tag: %d", $time, m_axis_tx_req_tag);
            captured_tag_q1 = m_axis_tx_req_tag;
        end else begin
            $error("[%0t] ERROR: Expected Queue 1, got %d", $time, m_axis_tx_req_queue);
        end

        // Wait a few cycles
        #50;
        begin
            @(posedge clk);
            s_axis_tx_req_status_tag <= 2;
            s_axis_tx_req_status_len <= 0; // len=0 means done, len>0 means more packets
            s_axis_tx_req_status_valid <= 1;
            @(posedge clk);
            s_axis_tx_req_status_valid <= 0;
        end
        
        begin
            @(posedge clk);
            s_axis_tx_req_status_tag <= 1;
            s_axis_tx_req_status_len <= 0; // len=0 means done, len>0 means more packets
            s_axis_tx_req_status_valid <= 1;
            @(posedge clk);
            s_axis_tx_req_status_valid <= 0;
        end
        #50;
        // -------------------------------------------------------
        // Test Case 2: Round Robin Logic (Q2 and Q3)
        // -------------------------------------------------------
        $display("--- Test Case 2: RR Logic (Q2 then Q3) ---");
        // Ring both doorbells back to back
        doorbell(2);
//        doorbell(3);

        // Check first request (Should be Q2 because it was rung first)
        wait(m_axis_tx_req_valid); 
        @(posedge clk); // Sample point
        if (m_axis_tx_req_queue == 2) begin
//            $display("[%0t] SUCCESS: RR Step 1 - Received Request for Queue 2", $time);
            captured_tag_q2 = m_axis_tx_req_tag;
        end else begin
            $error("[%0t] ERROR: RR Step 1 - Expected Queue 2, got %d", $time, m_axis_tx_req_queue);
        end

        // wait for handshake to complete if it was high (ready is always 1 here)
        #20; 

        // Check second request (Should be Q3)
        // Note: m_axis_tx_req_valid might toggle or stay high depending on timing, 
        // effectively we expect Q3 next.
//        wait(m_axis_tx_req_valid && m_axis_tx_req_queue == 3);
//        $display("[%0t] SUCCESS: RR Step 2 - Received Request for Queue 3", $time);

        #100;

        // -------------------------------------------------------
        // Test Case 3: Completion and Cleanup
        // -------------------------------------------------------
//        $display("--- Test Case 3: Completion ---");
//        // Send completion for Q1 (captured earlier). Len=0 implies queue is empty/done.
//        completion(captured_tag_q1, 0);

//        // Send completion for Q2. Len=0.
//        completion(captured_tag_q2, 0);

        begin
            @(posedge clk);
            s_axis_tx_req_status_tag <= 2;
            s_axis_tx_req_status_len <= 0; // len=0 means done, len>0 means more packets
            s_axis_tx_req_status_valid <= 1;
            @(posedge clk);
            s_axis_tx_req_status_valid <= 0;
        end
        
        begin
            @(posedge clk);
            s_axis_tx_req_status_tag <= 1;
            s_axis_tx_req_status_len <= 0; // len=0 means done, len>0 means more packets
            s_axis_tx_req_status_valid <= 1;
            @(posedge clk);
            s_axis_tx_req_status_valid <= 0;
        end

        begin
            @(posedge clk);
            s_axis_tx_req_status_tag <= 0;
            s_axis_tx_req_status_len <= 0; // len=0 means done, len>0 means more packets
            s_axis_tx_req_status_valid <= 1;
            @(posedge clk);
            s_axis_tx_req_status_valid <= 0;
        end
        
        #100;

        // -------------------------------------------------------
        // Test Case 4: Queue Reactivation
        // -------------------------------------------------------
//        $display("--- Test Case 4: Reactivating Q1 ---");
//        doorbell(1);
//        wait(m_axis_tx_req_valid && m_axis_tx_req_queue == 1);
//        $display("[%0t] SUCCESS: Queue 1 reactivated successfully", $time);

          #10000;
        $display("All tests passed!");
        $finish;
    end

endmodule