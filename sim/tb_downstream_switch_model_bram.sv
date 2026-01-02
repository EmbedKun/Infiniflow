`timescale 1ns / 1ps

module tb_downstream_switch_model_bram;

    // ========================================================================
    // 1. Parameters (Must match DUT)
    // ========================================================================
    parameter QUEUE_INDEX_WIDTH = 13;
    parameter DATA_WIDTH        = 512;
    // Set to 8 (256 Cells) for faster simulation fill-up
    // Capacity = 256 / 24 ≈ 10.6 packets
    parameter BUFFER_ADDR_WIDTH = 8; 
    parameter STAT_WIDTH        = 32;
    parameter DRAIN_RATIO_M     = 80; // Drain 1 packet every 4 cycles (Fast drain for sim)
    parameter ACTIVE_FIFO_DEPTH = 1024;
    parameter PKT_LEN_BYTES     = 1536;

    // Derived
    localparam PKT_BEATS = PKT_LEN_BYTES / (DATA_WIDTH/8); // 24

    // ========================================================================
    // 2. Signals
    // ========================================================================
    reg                          clk;
    reg                          rst;

    // Data Plane Input
    reg  [DATA_WIDTH-1:0]        s_axis_pkt_tdata;
    reg                          s_axis_pkt_tvalid;
    reg                          s_axis_pkt_tlast;
    reg  [DATA_WIDTH/8-1:0]      s_axis_pkt_tkeep;
    wire                         s_axis_pkt_tready;

    // Data Plane Output
    wire [DATA_WIDTH-1:0]        m_axis_pkt_tdata;
    wire                         m_axis_pkt_tvalid;

    // FCP Output
    wire                         fcp_valid;
    wire [QUEUE_INDEX_WIDTH-1:0] fcp_vc;
    wire [STAT_WIDTH-1:0]        fcp_fccl;
    wire [STAT_WIDTH-1:0]        fcp_qlen;
    wire [STAT_WIDTH-1:0]        fcp_fccr;

    // Debug
    wire [BUFFER_ADDR_WIDTH-1:0]        dbg_buffer_free_count;
    
    reg flag;
    // ========================================================================
    // 3. DUT Instantiation
    // ========================================================================
    downstream_switch_model_v2 # (
        .QUEUE_INDEX_WIDTH(QUEUE_INDEX_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .BUFFER_ADDR_WIDTH(BUFFER_ADDR_WIDTH),
        .STAT_WIDTH(STAT_WIDTH),
        .DRAIN_RATIO_M(DRAIN_RATIO_M),
        .ACTIVE_FIFO_DEPTH(ACTIVE_FIFO_DEPTH)
    ) uut (
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
    // 4. Clock Generation (330MHz approx)
    // ========================================================================
    initial begin
        clk = 0;
        forever #1.5 clk = ~clk; // Period = 3ns
    end

    // ========================================================================
    // 5. Tasks
    // ========================================================================
    
    // Task to send a full 1536B packet
    task send_packet;
        input [QUEUE_INDEX_WIDTH-1:0] queue_id;
        input [31:0] packet_id; // For visual debug in waveform
        integer i;
        begin
            // Check backpressure before starting
            wait(s_axis_pkt_tready);
            
            @(posedge clk);
            s_axis_pkt_tvalid <= 1;
            s_axis_pkt_tkeep  <= {64{1'b1}}; // All ones
            
            for (i = 0; i < PKT_BEATS; i = i + 1) begin
                // Construct Data:
                // [Low Bits]: Beat Counter
                // [16 +: 13]: Queue ID (Crucial for parsing!)
                // [High Bits]: Packet ID for debug
                s_axis_pkt_tdata <= { {(512-32-19-13-16-16){1'b0}}, packet_id,19'b0000, queue_id, 16'b0000, 16'b0 + i[15:0] };
                
                // Last beat handling
                if (i == PKT_BEATS - 1) 
                    s_axis_pkt_tlast <= 1;
                else
                    s_axis_pkt_tlast <= 0;

                // Handle Backpressure MID-PACKET
                // If tready drops, we must hold the current data
                do begin
                    @(posedge clk);
                end while (s_axis_pkt_tready == 0);
            end
            
            // End of packet
            s_axis_pkt_tvalid <= 0;
            s_axis_pkt_tlast  <= 0;
            s_axis_pkt_tdata  <= 0;
        end
    endtask

    // ========================================================================
    // 6. Main Stimulus
    // ========================================================================
    integer k;
    
    initial begin
        // Initialize Inputs
        rst = 1;
        s_axis_pkt_tdata = 0;
        s_axis_pkt_tvalid = 0;
        s_axis_pkt_tlast = 0;
        s_axis_pkt_tkeep = 0;
        flag = 0;
        // Reset Sequence
        #100;
        @(posedge clk);
        rst = 0;
        #10000000;
        @(posedge clk);
        
        for (k = 0; k < 2**QUEUE_INDEX_WIDTH-1; k = k + 1) begin
            send_packet(13'b0+k, 32'h0000_0000);
            @(posedge clk);
        end
        flag = 1;
        // Wait for everything to drain
        #100000;
        flag = 0;
        #100;
        for (k = 0; k < 2**QUEUE_INDEX_WIDTH-1; k = k + 1) begin
            send_packet(13'b0+k, 32'h0000_0000);
            @(posedge clk);
        end
        flag = 1;
    end
endmodule