// SPDX-License-Identifier: BSD-2-Clause-Views
/*
 * High-Performance Pipelined Switch Model
 * * Features:
 * - True Pipeline: 1 Packet/Cycle throughput (Ingress & Egress).
 * - Hazard Handling: Internal forwarding for back-to-back packets on same VC.
 * - Parallel Buffer Management.
 * - Clock-Driven Drain (Simulated Output Bandwidth).
 */

`timescale 1ns / 1ps
`default_nettype none

module downstream_switch_model_pipeline #
(
    parameter QUEUE_INDEX_WIDTH = 13, // 8K Queues
    parameter DATA_WIDTH        = 64,
    parameter BUFFER_ADDR_WIDTH = 12, // 4K Cells
    parameter STAT_WIDTH        = 32, 
    parameter DRAIN_RATIO_M     = 4,  // Egress Bandwidth Divider
    parameter ACTIVE_FIFO_DEPTH = 4096
)
(
    input  wire                          clk,
    input  wire                          rst,

    // Data Plane Input (Ingress)
    input  wire [DATA_WIDTH-1:0]         s_axis_pkt_tdata,
    input  wire                          s_axis_pkt_tvalid,
    input  wire                          s_axis_pkt_tlast, // Assumed 1-cell packets for model simplicity
    output wire                          s_axis_pkt_tready,

    // Data Plane Output (Egress)
    output reg  [DATA_WIDTH-1:0]         m_axis_pkt_tdata,
    output reg                           m_axis_pkt_tvalid,

    // FCP Output (Streaming Stats)
    output reg                           fcp_valid,
    output reg  [QUEUE_INDEX_WIDTH-1:0]  fcp_vc,
    output reg  [STAT_WIDTH-1:0]         fcp_fccl,
    output reg  [STAT_WIDTH-1:0]         fcp_qlen,
    output reg  [STAT_WIDTH-1:0]         fcp_fccr,
    
    // Debug
    output wire [STAT_WIDTH-1:0]         dbg_buffer_free_count
);

    localparam BUFFER_DEPTH = 2**BUFFER_ADDR_WIDTH;

    // ========================================================================
    // 1. RAM Definitions (Separate for Pipeline access)
    // ========================================================================
    
    // 指针 RAM：使用 Distributed RAM 以获得最佳的随机读写性能，避免 BRAM 的 R/W 冲突延迟
    (* ram_style = "distributed" *) reg [BUFFER_ADDR_WIDTH-1:0] ram_head [0:2**QUEUE_INDEX_WIDTH-1];
    (* ram_style = "distributed" *) reg [BUFFER_ADDR_WIDTH-1:0] ram_tail [0:2**QUEUE_INDEX_WIDTH-1];
    (* ram_style = "distributed" *) reg [0:0]                   ram_valid[0:2**QUEUE_INDEX_WIDTH-1];
    
    // 统计 RAM：也建议使用 Distributed 或 Quad-Port 模拟，这里用 Dist
    (* ram_style = "distributed" *) reg [STAT_WIDTH-1:0]        ram_rxcnt [0:2**QUEUE_INDEX_WIDTH-1];
    (* ram_style = "distributed" *) reg [STAT_WIDTH-1:0]        ram_txcnt [0:2**QUEUE_INDEX_WIDTH-1];

    // 数据 RAM 和 Next RAM：容量大，必须用 BRAM
    // Ingress 只写 Port A，Egress 只读 Port B -> 无冲突！
    (* ram_style = "block" *)       reg [DATA_WIDTH-1:0]        ram_data  [0:BUFFER_DEPTH-1];
    (* ram_style = "block" *)       reg [BUFFER_ADDR_WIDTH-1:0] ram_next  [0:BUFFER_DEPTH-1];

    // ========================================================================
    // 2. Resource Management (Free List) - Pre-fetch Logic
    // ========================================================================
    reg [BUFFER_ADDR_WIDTH-1:0] free_list_fifo [0:BUFFER_DEPTH-1];
    reg [BUFFER_ADDR_WIDTH:0]   fl_wr_ptr = 0, fl_rd_ptr = 0, fl_count = BUFFER_DEPTH;
    
    wire [BUFFER_ADDR_WIDTH-1:0] alloc_ptr; 
    reg  [BUFFER_ADDR_WIDTH-1:0] return_ptr;
    reg                          fl_pop_req;  // Ingress requests alloc
    reg                          fl_push_req; // Drain requests free

    // Free List Init
    integer i;
    initial begin
        for (i=0; i<BUFFER_DEPTH; i=i+1) free_list_fifo[i] = i;
    end

    // Alloc/Free Logic
    assign alloc_ptr = free_list_fifo[fl_rd_ptr[BUFFER_ADDR_WIDTH-1:0]];
    
    // Buffer Stats
    reg [STAT_WIDTH-1:0] buffer_free_cnt_reg = BUFFER_DEPTH;
    reg [STAT_WIDTH-1:0] total_rx_cnt_reg = 0;
    
    assign dbg_buffer_free_count = buffer_free_cnt_reg;
    assign s_axis_pkt_tready = (fl_count > 4); // Pipeline margin

    always @(posedge clk) begin
        if (rst) begin
            fl_wr_ptr <= 0; fl_rd_ptr <= 0; fl_count <= BUFFER_DEPTH;
            buffer_free_cnt_reg <= BUFFER_DEPTH; total_rx_cnt_reg <= 0;
        end else begin
            // Free
            if (fl_push_req) begin
                free_list_fifo[fl_wr_ptr[BUFFER_ADDR_WIDTH-1:0]] <= return_ptr;
                fl_wr_ptr <= fl_wr_ptr + 1;
            end
            // Alloc
            if (fl_pop_req) begin
                fl_rd_ptr <= fl_rd_ptr + 1;
            end
            // Count
            if (fl_pop_req && !fl_push_req) begin
                fl_count <= fl_count - 1; 
                buffer_free_cnt_reg <= buffer_free_cnt_reg - 1;
                total_rx_cnt_reg <= total_rx_cnt_reg + 1;
            end else if (!fl_pop_req && fl_push_req) begin
                fl_count <= fl_count + 1;
                buffer_free_cnt_reg <= buffer_free_cnt_reg + 1;
            end else if (fl_pop_req && fl_push_req) begin
                total_rx_cnt_reg <= total_rx_cnt_reg + 1;
            end
        end
    end

    // ========================================================================
    // 3. Scheduler (Round-Robin FIFO)
    // ========================================================================
    reg [QUEUE_INDEX_WIDTH-1:0] active_vc_fifo [0:ACTIVE_FIFO_DEPTH-1];
    reg [11:0] sch_wr_ptr = 0, sch_rd_ptr = 0;
    wire       sch_empty = (sch_wr_ptr == sch_rd_ptr);
    
    // Dual Push Signals
    reg        sch_push_ing;
    reg [QUEUE_INDEX_WIDTH-1:0] sch_vc_ing;
    reg        sch_push_drn;
    reg [QUEUE_INDEX_WIDTH-1:0] sch_vc_drn;
    reg        sch_pop;
    
    wire [QUEUE_INDEX_WIDTH-1:0] sch_next_vc = active_vc_fifo[sch_rd_ptr[11:0]];

    always @(posedge clk) begin
        if (rst) begin
            sch_wr_ptr <= 0; sch_rd_ptr <= 0;
        end else begin
            if (sch_pop && !sch_empty) sch_rd_ptr <= sch_rd_ptr + 1;

            if (sch_push_ing && sch_push_drn) begin
                active_vc_fifo[sch_wr_ptr[11:0]]         <= sch_vc_ing;
                active_vc_fifo[(sch_wr_ptr+1'b1) & 12'hFFF] <= sch_vc_drn;
                sch_wr_ptr <= sch_wr_ptr + 2;
            end else if (sch_push_ing) begin
                active_vc_fifo[sch_wr_ptr[11:0]] <= sch_vc_ing;
                sch_wr_ptr <= sch_wr_ptr + 1;
            end else if (sch_push_drn) begin
                active_vc_fifo[sch_wr_ptr[11:0]] <= sch_vc_drn;
                sch_wr_ptr <= sch_wr_ptr + 1;
            end
        end
    end

    // ========================================================================
    // 4. Ingress Pipeline (The Writer)
    // ========================================================================
    // Stage 0: Alloc & Input
    // Stage 1: RAM Read (Tail, Valid) & Data Write
    // Stage 2: RAM Write (Tail, Next, Head, Valid) + Forwarding

    // --- Pipeline Registers ---
    reg [QUEUE_INDEX_WIDTH-1:0] ing_s0_vc, ing_s1_vc, ing_s2_vc;
    reg [BUFFER_ADDR_WIDTH-1:0] ing_s0_cell, ing_s1_cell, ing_s2_cell;
    reg                         ing_s0_vld, ing_s1_vld, ing_s2_vld;
    
    // Forwarding Registers (From Stage 2 to Stage 1)
    reg [QUEUE_INDEX_WIDTH-1:0] last_updated_vc;
    reg [BUFFER_ADDR_WIDTH-1:0] last_updated_tail;
    reg                         last_updated_active_flag;
    reg                         last_was_write;

    // --- Stage 0: Input Latch & Alloc ---
    always @(posedge clk) begin
        if (rst) begin
            ing_s0_vld <= 0;
            fl_pop_req <= 0;
            last_was_write <= 0;
        end else begin
            // Default
            fl_pop_req <= 0;
            ing_s0_vld <= 0;

            if (s_axis_pkt_tvalid && s_axis_pkt_tready) begin
                ing_s0_vld  <= 1;
                ing_s0_vc   <= s_axis_pkt_tdata[16 +: QUEUE_INDEX_WIDTH]; // Parsing
                ing_s0_cell <= alloc_ptr; // Grab from Free List
                fl_pop_req  <= 1;         // Advance Free List
            end
        end
    end

    // --- Stage 1: Read State & Write Payload ---
    reg [BUFFER_ADDR_WIDTH-1:0] r_tail_ptr;
    reg                         r_q_valid;
    
    always @(posedge clk) begin
        if (rst) ing_s1_vld <= 0;
        else     ing_s1_vld <= ing_s0_vld;
        
        ing_s1_vc   <= ing_s0_vc;
        ing_s1_cell <= ing_s0_cell;

        // Write Payload Data (BRAM Port A) - No conflict with Egress (Port B)
        if (ing_s0_vld) begin
            ram_data[ing_s0_cell] <= s_axis_pkt_tdata; // Use delayed data if needed
        end

        // Read Pointers (Distributed RAM)
        // **Forwarding Logic**: If Stage 2 is writing to the same VC, use Stage 2's result
        if (ing_s0_vld) begin
            if (last_was_write && (last_updated_vc == ing_s0_vc)) begin
                r_tail_ptr <= last_updated_tail;
                r_q_valid  <= 1'b1; // It must be valid if we just wrote to it
            end else begin
                r_tail_ptr <= ram_tail[ing_s0_vc];
                r_q_valid  <= ram_valid[ing_s0_vc];
            end
        end
    end

    // --- Stage 2: State Update (Link List) ---
    always @(posedge clk) begin
        if (rst) begin
            ing_s2_vld <= 0;
            sch_push_ing <= 0;
            last_was_write <= 0;
        end else begin
            ing_s2_vld <= ing_s1_vld;
            ing_s2_vc  <= ing_s1_vc;
            sch_push_ing <= 0;
            last_was_write <= 0;

            if (ing_s1_vld) begin
                last_was_write <= 1;
                last_updated_vc <= ing_s1_vc;
                last_updated_tail <= ing_s1_cell; // New tail is current cell

                // Update RX Count
                ram_rxcnt[ing_s1_vc] <= ram_rxcnt[ing_s1_vc] + 1;

                if (!r_q_valid) begin
                    // Case: Queue was Empty
                    ram_head[ing_s1_vc]  <= ing_s1_cell; // New Head
                    ram_tail[ing_s1_vc]  <= ing_s1_cell; // New Tail
                    ram_valid[ing_s1_vc] <= 1;
                    
                    // Activate Scheduler
                    sch_push_ing <= 1;
                    sch_vc_ing   <= ing_s1_vc;
                end else begin
                    // Case: Queue Active
                    ram_next[r_tail_ptr] <= ing_s1_cell; // Link old tail to new cell (BRAM Port A)
                    ram_tail[ing_s1_vc]  <= ing_s1_cell; // Update Tail
                end
            end
        end
    end

    // ========================================================================
    // 5. Egress Pipeline (The Reader)
    // ========================================================================
    // Stage 0: Scheduler Pop
    // Stage 1: Read Head & Tail (Check if last)
    // Stage 2: Read Next & Data
    // Stage 3: Output, Free, Update Head

    reg [15:0] drn_ratio_cnt;
    wire       drn_tick = (drn_ratio_cnt == DRAIN_RATIO_M);
    
    always @(posedge clk) begin
        if (rst) drn_ratio_cnt <= 0;
        else if (drn_tick) drn_ratio_cnt <= 0;
        else drn_ratio_cnt <= drn_ratio_cnt + 1;
    end

    reg [QUEUE_INDEX_WIDTH-1:0] drn_s0_vc, drn_s1_vc, drn_s2_vc;
    reg                         drn_s0_vld, drn_s1_vld, drn_s2_vld;
    
    // --- Stage 0: Schedule ---
    always @(posedge clk) begin
        sch_pop    <= 0;
        drn_s0_vld <= 0;
        
        if (drn_tick && !sch_empty) begin
            sch_pop    <= 1;
            drn_s0_vld <= 1;
            drn_s0_vc  <= sch_next_vc;
        end
    end

    // --- Stage 1: Read Head ---
    reg [BUFFER_ADDR_WIDTH-1:0] r_head_ptr;
    reg [BUFFER_ADDR_WIDTH-1:0] r_tail_check;
    
    always @(posedge clk) begin
        drn_s1_vld <= drn_s0_vld;
        drn_s1_vc  <= drn_s0_vc;
        
        if (drn_s0_vld) begin
            r_head_ptr   <= ram_head[drn_s0_vc];
            r_tail_check <= ram_tail[drn_s0_vc]; 
        end
    end

    // --- Stage 2: Read Data & Next ---
    reg [DATA_WIDTH-1:0]        r_out_data;
    reg [BUFFER_ADDR_WIDTH-1:0] r_next_ptr;
    reg                         r_is_last;

    always @(posedge clk) begin
        drn_s2_vld <= drn_s1_vld;
        drn_s2_vc  <= drn_s1_vc;
        
        if (drn_s1_vld) begin
            r_out_data <= ram_data[r_head_ptr]; // BRAM Port B (Read)
            r_next_ptr <= ram_next[r_head_ptr]; // BRAM Port B (Read)
            r_is_last  <= (r_head_ptr == r_tail_check);
            
            // Prepare for Free
            return_ptr <= r_head_ptr;
        end
    end

    // --- Stage 3: Output & Update ---
    always @(posedge clk) begin
        if (rst) begin
            m_axis_pkt_tvalid <= 0;
            sch_push_drn <= 0;
            fcp_valid <= 0;
            fl_push_req <= 0;
        end else begin
            // Defaults
            m_axis_pkt_tvalid <= 0;
            sch_push_drn      <= 0;
            fcp_valid         <= 0;
            fl_push_req       <= 0;

            if (drn_s2_vld) begin
                // 1. Output Data
                m_axis_pkt_tvalid <= 1;
                m_axis_pkt_tdata  <= r_out_data;

                // 2. Free Cell
                fl_push_req <= 1;
                // return_ptr set in Stage 2

                // 3. Update Pointers & Scheduler
                if (r_is_last) begin
                    // Queue becomes empty
                    ram_valid[drn_s2_vc] <= 0;
                    // Do NOT push back to scheduler
                end else begin
                    // Queue still active
                    ram_head[drn_s2_vc] <= r_next_ptr;
                    
                    // Re-queue to Scheduler (Round Robin)
                    sch_push_drn <= 1;
                    sch_vc_drn   <= drn_s2_vc;
                end

                // 4. Update Stats & Output FCP
                ram_txcnt[drn_s2_vc] <= ram_txcnt[drn_s2_vc] + 1;
                
                fcp_valid <= 1;
                fcp_vc    <= drn_s2_vc;
                fcp_fccr  <= ram_txcnt[drn_s2_vc] + 1;
                fcp_qlen  <= ram_rxcnt[drn_s2_vc] - (ram_txcnt[drn_s2_vc] + 1);
                fcp_fccl  <= buffer_free_cnt_reg + total_rx_cnt_reg; // Approximate global snapshot
            end
        end
    end

endmodule
`resetall