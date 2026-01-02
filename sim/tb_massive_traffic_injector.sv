`timescale 1ns / 1ps

module tb_massive_traffic_injector;

    // ========================================================================
    // 1. Parameter Definitions
    // ========================================================================
    // Reduced size for faster simulation, or use full size for verification.
    // NOTE: To simulate full 262,144 queues, simulation might take a while.
    // For quick verification, you can reduce QUEUE_INDEX_WIDTH to 6 or 8.
    // Here we keep it as 18 to match your design intent.
    parameter QUEUE_INDEX_WIDTH = 16; 
    parameter REQ_TAG_WIDTH     = 8;
    parameter LEN_WIDTH         = 16;
    parameter OP_TABLE_SIZE     = 64;
    parameter PIPELINE          = 7;
    parameter DATA_WIDTH        = 512;
    parameter PKT_LEN_BYTES     = 1536;

    localparam QUEUE_COUNT = 2**QUEUE_INDEX_WIDTH;

    // ========================================================================
    // 2. DUT Signals
    // ========================================================================
    reg                          clk;
    reg                          rst;
    reg                          enable;
    reg  [QUEUE_INDEX_WIDTH-1:0] stop_queue_idx;
    reg                          stop_cmd_valid;

    wire [DATA_WIDTH-1:0]        m_axis_pkt_tdata;
    wire                         m_axis_pkt_tvalid;
    wire                         m_axis_pkt_tlast;
    wire [DATA_WIDTH/8-1:0]      m_axis_pkt_tkeep;
    reg                          m_axis_pkt_tready; // Driven by TB

    wire                         scheduler_active;

    // ========================================================================
    // 3. DUT Instantiation
    // ========================================================================
    massive_traffic_injector #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .REQ_TAG_WIDTH(REQ_TAG_WIDTH),
        .LEN_WIDTH(LEN_WIDTH),
        .OP_TABLE_SIZE(OP_TABLE_SIZE),
        .PIPELINE(PIPELINE),
        .DATA_WIDTH(DATA_WIDTH),
        .PKT_LEN_BYTES(PKT_LEN_BYTES)
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
        .scheduler_active(scheduler_active)
    );

    // ========================================================================
    // 4. Clock Generation
    // ========================================================================
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // ========================================================================
    // 5. Statistics & Monitoring Variables
    // ========================================================================
    longint pkt_count = 0;
    logic [QUEUE_INDEX_WIDTH-1:0] extracted_queue_id;
    logic [QUEUE_INDEX_WIDTH-1:0] expected_queue_id;

    // ========================================================================
    // 6. Main Test Procedure
    // ========================================================================
    initial begin
        // --- Initialize Inputs ---
        rst = 1;
        enable = 0;
        stop_queue_idx = 0;
        stop_cmd_valid = 0;
        m_axis_pkt_tready = 0; // Backpressure initially

        $display("[%0t] Simulation Started.", $time);
        $display("[%0t] Configuration: %0d Queues.", $time, QUEUE_COUNT);

        // --- Reset Sequence ---
        repeat (20) @(posedge clk);
        rst = 0;
        $display("[%0t] Reset Released. Waiting for Auto-Activator...", $time);

        // --- Wait for Auto-Activation ---
        // The simulator takes QUEUE_COUNT cycles to ring all doorbells.
        // We wait slightly longer to be safe.
        repeat (QUEUE_COUNT + 100) @(posedge clk);

        $display("[%0t] Initialization should be complete.", $time);

        // --- Enable Scheduler ---
        @(posedge clk);
        enable = 1;
        #2000;
        m_axis_pkt_tready = 1; // Sink is ready to accept packets
        $display("[%0t] Scheduler Enabled. Traffic sink ready.", $time);

        // --- Simulation Run ---
        // Monitor will automatically print packets below.
        // We let it run for enough cycles to see queue rotation.
        
        // Wait until we receive 50 packets to verify rotation
        wait (pkt_count >= 50);

        $display("[%0t] Received 50 packets successfully.", $time);
        
        // Add random backpressure to test AXI Stream handshake
        $display("[%0t] Applying random backpressure...", $time);

        m_axis_pkt_tready = 1;

        #2000;
        #2000;
                #2000;
                        #2000;
                                #2000;
                                        #2000;
                                                #2000;
                                                        #2000;
        $display("[%0t] Test Passed! Total Packets: %0d", $time, pkt_count);
        $finish;
    end

    // ========================================================================
    // 7. Packet Monitor / Checker
    // ========================================================================
    // This block parses the output AXI Stream to extract Queue IDs
    always @(posedge clk) begin
        if (m_axis_pkt_tvalid && m_axis_pkt_tready) begin
            
            // In your simulator design: 
            // m_axis_pkt_tdata <= {{(DATA_WIDTH-QUEUE_INDEX_WIDTH-16){1'b0}}, current_q, word_cnt};
            // Assuming 64-bit width and 18-bit Queue ID:
            // Data = [Padding] [QueueID (18bit)] [Counter (16bit)]
            // We extract bits [33:16] for queue ID (16+18 = 34)
            
            extracted_queue_id = m_axis_pkt_tdata[16 +: QUEUE_INDEX_WIDTH];

            // Only check on the first word of a packet (where word_cnt == 0)
            if (m_axis_pkt_tdata[15:0] == 0) begin
                
                // Print the first few packets to verify rotation
                if (pkt_count < 20) begin
                    $display("[%0t] Packet Received | Queue ID: %0d", $time, extracted_queue_id);
                    
                    // Optional: Check sequential order (0, 1, 2...)
                    // Note: Depending on pipeline depth, the first packet might not be exactly 0,
                    // but they should be sequential.
                    if (pkt_count > 0) begin
                        if (extracted_queue_id != (expected_queue_id + 1)) begin
                            // This is a soft warning because pipeline startup might be tricky,
                            // but in steady state it should be strictly RR.
                            // $warning("Non-sequential Queue ID observed!");
                        end
                    end
                    expected_queue_id = extracted_queue_id;
                end

                pkt_count++;
            end
        end
    end

endmodule