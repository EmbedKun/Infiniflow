`timescale 1ns / 1ps
`default_nettype none

module switch_fmax_wrapper (
//    input  wire       sys_clk_p, // Differential Clock Input (Common on UltraScale+)
//    input  wire       sys_clk_n,
    input  wire       clk_330,
    input  wire       sys_rst_n, // Reset (Active Low)
    output reg        done_led   // Single output to prevent logic optimization
);
    wire sys_rst = ~sys_rst_n;
    
    // ========================================================================
    // 1. Clock Management
    // ========================================================================
//    wire clk_330;
    
    // Use IBUFDS + BUFG for differential clock input on real hardware
    // If just for Implementation analysis, IBUFDS is sufficient
//    wire clk_ibuf;
//    IBUFDS u_ibufds (
//        .I (sys_clk_p),
//        .IB(sys_clk_n),
//        .O (clk_ibuf)
//    );
    
//    BUFG u_bufg (
//        .I (clk_ibuf),
//        .O (clk_330)
//    );

    // Internal High-Fanout Reset Tree
    (* keep = "true" *) reg rst_reg_1 = 1;
    (* keep = "true" *) reg rst_reg_2 = 1;
    always @(posedge clk_330) begin
        rst_reg_1 <= sys_rst;
        rst_reg_2 <= rst_reg_1; // Synchronized reset tree
    end
    wire internal_rst = rst_reg_2;

    // ========================================================================
    // 2. Stimulus Generator - Generates 512-bit Line Rate Data
    // ========================================================================
    // Uses LFSR or counter to ensure data toggling, preventing logic pruning
    
    reg [511:0] r_tdata;
    reg         r_tvalid;
    reg         r_tlast;
    reg [63:0]  r_tkeep;
    
    // Ready signal from DUT requires response
    wire        w_tready;
    
    // Simple counter to simulate packet length (e.g., 24 beats per packet)
    reg [7:0]   beat_cnt;
    
    always @(posedge clk_330) begin
        if (internal_rst) begin
            r_tdata  <= 512'd0;
            r_tvalid <= 1'b0;
            r_tlast  <= 1'b0;
            r_tkeep  <= 64'hFFFF_FFFF_FFFF_FFFF;
            beat_cnt <= 0;
        end else begin
            // Pump data if DUT is ready (Line Rate Stress Test)
            if (w_tready) begin
                r_tvalid <= 1'b1;
                // Toggle data bits to test dynamic power and timing
                r_tdata  <= {r_tdata[510:0], ~r_tdata[511]}; 
                
                // Construct TLAST
                if (beat_cnt == 23) begin
                    r_tlast  <= 1'b1;
                    beat_cnt <= 0;
                end else begin
                    r_tlast  <= 1'b0;
                    beat_cnt <= beat_cnt + 1;
                end
            end 
            // If !ready, hold valid and data (AXI Stream Spec)
        end
    end

    // ========================================================================
    // 3. Input Register Isolation
    // ========================================================================
    // Ensures Vivado analyzes Reg->DUT paths, not Pad->DUT
    (* keep = "true" *) reg [511:0] dut_s_tdata;
    (* keep = "true" *) reg         dut_s_tvalid;
    (* keep = "true" *) reg         dut_s_tlast;
    (* keep = "true" *) reg [63:0]  dut_s_tkeep;

    always @(posedge clk_330) begin
        dut_s_tdata  <= r_tdata;
        dut_s_tvalid <= r_tvalid;
        dut_s_tlast  <= r_tlast;
        dut_s_tkeep  <= r_tkeep;
    end

    // ========================================================================
    // 4. DUT Instantiation
    // ========================================================================
    // Output Wires
    wire [511:0] dut_m_tdata;
    wire         dut_m_tvalid;
    wire         dut_m_tlast;
    wire [63:0]  dut_m_tkeep;
    
    wire         fcp_valid;
    wire [15:0]  fcp_vc;
    wire [31:0]  fcp_fccl;
    wire [31:0]  fcp_qlen;
    wire [31:0]  fcp_fccr;
    
    wire [31:0]  dbg_free;
    wire [31:0]  dbg_total;

    // Parameters must match your module
    downstream_switch_model_v2 #(
        .QUEUE_INDEX_WIDTH(16),
        .DATA_WIDTH(512),
        .BUFFER_ADDR_WIDTH(8), // Keep small buffer to test high-pressure logic
        .STAT_WIDTH(32),
        .DRAIN_RATIO_M(45)     // Full speed drain
    ) inst_dut (
        .clk              (clk_330),
        .rst              (internal_rst),
        
        .s_axis_pkt_tdata (dut_s_tdata),
        .s_axis_pkt_tvalid(dut_s_tvalid),
        .s_axis_pkt_tlast (dut_s_tlast),
        .s_axis_pkt_tkeep (dut_s_tkeep),
        .s_axis_pkt_tready(w_tready),
        
        .m_axis_pkt_tdata (dut_m_tdata),
        .m_axis_pkt_tvalid(dut_m_tvalid),
        
        .fcp_valid        (fcp_valid),
        .fcp_vc           (fcp_vc),
        .fcp_fccl         (fcp_fccl),
        .fcp_qlen         (fcp_qlen),
        .fcp_fccr         (fcp_fccr),
        
        .dbg_buffer_free_count(dbg_free),
        .dbg_total_rx_count   (dbg_total)
    );

    // ========================================================================
    // 5. Output Register Isolation & XOR Reduction
    // ========================================================================
    // Ensures Vivado analyzes DUT->Reg paths and prevents optimization
    
    reg [511:0] cap_m_tdata;
    reg         cap_m_tvalid;
    reg         cap_m_tlast;
    reg [31:0]  cap_fcp_data; // Mix of all FCP data
    
    always @(posedge clk_330) begin
        // Stage 1: Capture all outputs
        cap_m_tdata  <= dut_m_tdata;
        cap_m_tvalid <= dut_m_tvalid;
        cap_m_tlast  <= dut_m_tlast;
        // XOR mix miscellaneous signals
        cap_fcp_data <= fcp_fccl ^ fcp_qlen ^ fcp_fccr ^ {19'b0, fcp_vc} ^ dbg_free ^ dbg_total;
    end

    // Stage 2: XOR Reduction Tree (Prevents congestion, pipelined XOR)
    reg reduce_1;
    reg reduce_2;
    
    always @(posedge clk_330) begin
        // Reduce 512-bit data to 1-bit
        reduce_1 <= ^cap_m_tdata; 
        // Reduce control signals to 1-bit
        reduce_2 <= cap_m_tvalid ^ cap_m_tlast ^ (^cap_fcp_data) ^ fcp_valid;
    end

    // Stage 3: Output to Pin
    always @(posedge clk_330) begin
        done_led <= reduce_1 ^ reduce_2;
    end

endmodule