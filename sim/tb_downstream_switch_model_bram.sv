`timescale 1ns / 1ps

module tb_downstream_switch_model_bram;

    // ========================================================================
    // 1. Parameters & Signals
    // ========================================================================
    parameter QUEUE_INDEX_WIDTH = 4; // Reduced for TB (16 VCs)
    parameter DATA_WIDTH        = 64;
    parameter BUFFER_ADDR_WIDTH = 8; // Reduced for TB (256 Cells)
    parameter STAT_WIDTH        = 32;
    parameter DRAIN_RATIO_M     = 15; // Fast drain for simulation

    reg                          clk;
    reg                          rst;

    // Input Interface
    reg  [DATA_WIDTH-1:0]        s_axis_pkt_tdata;
    reg                          s_axis_pkt_tvalid;
    reg                          s_axis_pkt_tlast;
    reg  [DATA_WIDTH/8-1:0]      s_axis_pkt_tkeep;
    wire                         s_axis_pkt_tready;

    // Output Interface
    wire [DATA_WIDTH-1:0]        m_axis_pkt_tdata;
    wire                         m_axis_pkt_tvalid;

    // FCP Interface
    wire                         fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] fcp_vc;
    wire [STAT_WIDTH-1:0]        fcp_fccl;
    wire [STAT_WIDTH-1:0]        fcp_qlen;
    wire [STAT_WIDTH-1:0]        fcp_fccr;

    // Debug Stats
    wire [STAT_WIDTH-1:0]        dbg_buffer_free_count;
    wire [STAT_WIDTH-1:0]        dbg_total_rx_count;

    // Simulation Stats
    integer                      tb_total_sent_count = 0;
    integer                      tb_total_rcvd_count = 0;
    integer                      vc_sent_count [2**QUEUE_INDEX_WIDTH-1:0];
    
    // Expected Data Queue (Queue of Queues for each VC is overkill, simplifying)
    // Using a single large FIFO for expected data verification since we can't 
    // easily predict exact drain order across VCs in this simple TB without a scoreboard.
    // BUT, the DUT outputs data, so we can verify if the data looks "valid" based on our pattern.
    // Pattern: {Zero_Padding, VC_ID, Seq_Num}
    
    // ========================================================================
    // 2. DUT Instantiation
    // ========================================================================
    downstream_switch_model_bram #(
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .STAT_WIDTH(STAT_WIDTH),
        .DRAIN_RATIO_M(DRAIN_RATIO_M)
    ) dut (
        .clk(clk),
        .rst(rst),
        .s_axis_pkt_tdata(s_axis_pkt_tdata),
        .s_axis_pkt_tvalid(s_axis_pkt_tvalid),
        .s_axis_pkt_tlast(s_axis_pkt_tlast),
        .s_axis_pkt_tkeep(s_axis_pkt_tkeep),
        .s_axis_pkt_tready(s_axis_pkt_tready),
        .m_axis_pkt_tdata(m_axis_pkt_tdata),
        .m_axis_pkt_tvalid(m_axis_pkt_tvalid),
        .fcp_valid(fcp_valid),
        .fcp_vc(fcp_vc),
        .fcp_fccl(fcp_fccl),
        .fcp_qlen(fcp_qlen),
        .fcp_fccr(fcp_fccr),
        .dbg_buffer_free_count(dbg_buffer_free_count),
        .dbg_total_rx_count(dbg_total_rx_count)
    );

    // ========================================================================
    // 3. Clock Generation
    // ========================================================================
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // ========================================================================
    // 4. Tasks
    // ========================================================================
    
    // Task: Reset System
    task sys_reset();
        begin
            rst = 1;
            s_axis_pkt_tvalid = 0;
            s_axis_pkt_tdata  = 0;
            s_axis_pkt_tlast  = 0;
            s_axis_pkt_tkeep  = 0;
            #100;
            rst = 0;
            #100;
        end
    endtask

    // Task: Send Single Packet (Single Flit for simplicity in this model)
    // Data Pattern: [63:32]=Zero, [31:16]=VC_ID, [15:0]=Seq_Num
    task send_packet(input [QUEUE_INDEX_WIDTH-1:0] vc_id);
        reg [15:0] seq_num;
        begin
            seq_num = vc_sent_count[vc_id] & 16'hFFFF;
            
            // Wait for Ready
            wait(s_axis_pkt_tready);
            @(posedge clk);
            
            s_axis_pkt_tvalid <= 1;
            // Matches DUT extraction: parsed_vc_idx = data[16 +: QUEUE_INDEX_WIDTH]
            // We put VC_ID at bit 16.
            s_axis_pkt_tdata  <= {32'h0, {16-QUEUE_INDEX_WIDTH{1'b0}}, vc_id, seq_num}; 
            s_axis_pkt_tlast  <= 1; // Single beat packet
            s_axis_pkt_tkeep  <= {DATA_WIDTH/8{1'b1}};
            
            @(posedge clk);
            s_axis_pkt_tvalid <= 0;
            s_axis_pkt_tlast  <= 0;
            
            vc_sent_count[vc_id] = vc_sent_count[vc_id] + 1;
            tb_total_sent_count = tb_total_sent_count + 1;
        end
    endtask

    // ========================================================================
    // 5. Stimulus Generation
    // ========================================================================
    integer i;
    initial begin
        // Init Arrays
        for (i=0; i<2**QUEUE_INDEX_WIDTH; i=i+1) vc_sent_count[i] = 0;

        sys_reset();
        
        $display("=== Starting Test ===");

        // Phase 1: Send Burst to VC 0
        $display("[Time %0t] Sending 10 packets to VC 0...", $time);
        for (i=0; i<10; i=i+1) begin
            send_packet(0); 
            #10; // Small gap
        end

        // Phase 2: Send Interleaved to Random VCs
        $display("[Time %0t] Sending 50 random packets...", $time);
        for (i=0; i<50; i=i+1) begin
            send_packet($urandom_range(0, (2**QUEUE_INDEX_WIDTH)-1));
            #10;
        end

        // Wait for drain
        #5000;

        for (i=0; i<50; i=i+1) begin
            send_packet($urandom_range(0, (2**QUEUE_INDEX_WIDTH)-1));
            #10;
        end

       #5000;

        // Final Check
        $display("=== Test Completed ===");
        $display("Total Sent by TB: %0d", tb_total_sent_count);
        $display("Total Rcvd by TB: %0d", tb_total_rcvd_count);
        $display("DUT Total Rx Count: %0d", dbg_total_rx_count);
        $display("Buffer Free Count: %0d / %0d", dbg_buffer_free_count, 2**BUFFER_ADDR_WIDTH);
        
        if (tb_total_sent_count == dbg_total_rx_count && dbg_total_rx_count == tb_total_rcvd_count)
            $display("SUCCESS: All counts match!");
        else
            $display("ERROR: Count mismatch!");
            
        $stop;
    end

    // ========================================================================
    // 6. Monitor / Verification
    // ========================================================================
    
    // Monitor Output Data
    always @(posedge clk) begin
        if (m_axis_pkt_tvalid) begin
            tb_total_rcvd_count <= tb_total_rcvd_count + 1;
            // Extract info from received data to verify
            // Expected: [31:16] = VC, [15:0] = Seq (We only check structure here)
            $display("[Time %0t] [Monitor] Rcvd Pkt: Data=0x%h (VC=%0d, Seq=%0d)", 
                     $time, m_axis_pkt_tdata, m_axis_pkt_tdata[16 +: QUEUE_INDEX_WIDTH], m_axis_pkt_tdata[15:0]);
        end
    end

    // Monitor FCP Updates
    always @(posedge clk) begin
        if (fcp_valid) begin
            // $display("[Time %0t] [FCP] VC=%0d QLEN=%0d FCCR=%0d", $time, fcp_vc, fcp_qlen, fcp_fccr);
        end
    end

endmodule