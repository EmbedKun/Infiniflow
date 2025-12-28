// SPDX-License-Identifier: BSD-2-Clause-Views
/*
 * Downstream Switch Model (Fully Synthesizable BRAM Version) with Data Drain
 * - Added: Data output logic in Drain State Machine to verify data integrity.
 */
// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none


// ========================================================================
// Main Module
// ========================================================================
module downstream_switch_model_bram #
(
    parameter QUEUE_INDEX_WIDTH = 13, // 262,144 VCs
    parameter DATA_WIDTH        = 64,
    parameter BUFFER_ADDR_WIDTH = 12, // 16K Cells
    parameter STAT_WIDTH        = 32, 
    parameter DRAIN_RATIO_M     = 4,
    parameter ACTIVE_FIFO_DEPTH = 1024
)
(
    input  wire                          clk,
    input  wire                          rst,

    // Data Plane Input
    input  wire [DATA_WIDTH-1:0]         s_axis_pkt_tdata,
    input  wire                          s_axis_pkt_tvalid,
    input  wire                          s_axis_pkt_tlast,
    input  wire [DATA_WIDTH/8-1:0]       s_axis_pkt_tkeep,
    output reg                           s_axis_pkt_tready, 

    // <<< MODIFIED: Data Plane Output (For Verification) >>>
    output reg  [DATA_WIDTH-1:0]         m_axis_pkt_tdata,
    output reg                           m_axis_pkt_tvalid,

    // FCP Output
    output reg                           fcp_valid,
    output reg  [QUEUE_INDEX_WIDTH-1:0]  fcp_vc,
    output reg  [STAT_WIDTH-1:0]         fcp_fccl,
    output reg  [STAT_WIDTH-1:0]         fcp_qlen,
    output reg  [STAT_WIDTH-1:0]         fcp_fccr,
    
    // Debug used
    output wire [STAT_WIDTH-1:0]         dbg_buffer_free_count,
    output wire [STAT_WIDTH-1:0]         dbg_total_rx_count
);

    localparam BUFFER_DEPTH = 2**BUFFER_ADDR_WIDTH;

    // --------------------------------------------------------------------
    // 1. Signal Declarations
    // --------------------------------------------------------------------
    
    // Extracted VC Index
    wire [QUEUE_INDEX_WIDTH-1:0] parsed_vc_idx;
    assign parsed_vc_idx = s_axis_pkt_tdata[16 +: QUEUE_INDEX_WIDTH];

    // --- BRAM Signals ---
    
    // Head Pointer RAM (Stores Head Ptr for each VC)
    wire [BUFFER_ADDR_WIDTH-1:0] ram_head_douta, ram_head_doutb;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_head_dina,  ram_head_dinb;
    reg                          ram_head_wea,   ram_head_web;
    reg  [QUEUE_INDEX_WIDTH-1:0] ram_head_addra, ram_head_addrb;

    // Tail Pointer RAM (Stores Tail Ptr for each VC)
    wire [BUFFER_ADDR_WIDTH-1:0] ram_tail_douta, ram_tail_doutb;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_tail_dina;
    reg                          ram_tail_wea;
    reg  [QUEUE_INDEX_WIDTH-1:0] ram_tail_addra, ram_tail_addrb;

    // Valid Flag RAM (Stores is_active for each VC)
    wire [0:0] ram_valid_douta, ram_valid_doutb;
    reg  [0:0] ram_valid_dina,  ram_valid_dinb;
    reg        ram_valid_wea,   ram_valid_web;

    // RX Counter RAM
    wire [STAT_WIDTH-1:0] ram_rxcnt_douta, ram_rxcnt_doutb;
    reg  [STAT_WIDTH-1:0] ram_rxcnt_dina;
    reg                   ram_rxcnt_wea;

    // TX Counter RAM
    wire [STAT_WIDTH-1:0] ram_txcnt_doutb;
    reg  [STAT_WIDTH-1:0] ram_txcnt_dinb;
    reg                   ram_txcnt_web;

    // Next Pointer RAM (The Linked List)
    // Port A used by Ingress (Update Old Tail), Port B used by Drain (Read Head's Next)
    wire [BUFFER_ADDR_WIDTH-1:0] ram_next_doutb;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_next_dina;
    reg                          ram_next_wea;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_next_addra, ram_next_addrb;

    // Data RAM (Payload)
    // <<< MODIFIED: Added Port B signals for draining data >>>
    reg  [DATA_WIDTH-1:0]        ram_data_dina;
    reg                          ram_data_wea;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_data_addra;

    reg  [BUFFER_ADDR_WIDTH-1:0] ram_data_addrb; // For Drain Read Addr
    wire [DATA_WIDTH-1:0]        ram_data_doutb; // For Drain Read Data

    // --- Free List & Active FIFO ---
    reg [BUFFER_ADDR_WIDTH-1:0] free_list_ram [BUFFER_DEPTH-1:0];
    reg [BUFFER_ADDR_WIDTH:0]   fl_wr_ptr = BUFFER_DEPTH, fl_rd_ptr = 0, fl_count = BUFFER_DEPTH;
    
    // Need a register to hold the output of free list RAM
    reg [BUFFER_ADDR_WIDTH-1:0] alloc_cell_ptr_reg;
    wire [BUFFER_ADDR_WIDTH-1:0] alloc_cell_ptr_next;
    
    reg [BUFFER_ADDR_WIDTH-1:0] return_cell_ptr;
    
    // Centralized Resource Management
    reg ing_alloc_req;
    reg drn_free_req;
    
    // Active VC FIFO
    reg [QUEUE_INDEX_WIDTH-1:0] active_vc_fifo [ACTIVE_FIFO_DEPTH-1:0];
    reg [9:0] active_wr_ptr = 0, active_rd_ptr = 0;
    wire active_fifo_empty = (active_wr_ptr == active_rd_ptr);
    reg [QUEUE_INDEX_WIDTH-1:0] drain_candidate_vc;

    // Global Stats
    reg [STAT_WIDTH-1:0] total_rx_count = 0;
    reg [STAT_WIDTH-1:0] buffer_free_count = BUFFER_DEPTH;
    assign dbg_buffer_free_count = buffer_free_count;
    assign dbg_total_rx_count = total_rx_count;
    // --------------------------------------------------------------------
    // 2. Memory Instantiations
    // --------------------------------------------------------------------

    tdp_bram #(.DATA_WIDTH(BUFFER_ADDR_WIDTH), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_head (
        .clk(clk),
        .wea(ram_head_wea), .addra(ram_head_addra), .dina(ram_head_dina), .douta(ram_head_douta),
        .web(ram_head_web), .addrb(ram_head_addrb), .dinb(ram_head_dinb), .doutb(ram_head_doutb)
    );

    tdp_bram #(.DATA_WIDTH(BUFFER_ADDR_WIDTH), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_tail (
        .clk(clk),
        .wea(ram_tail_wea), .addra(ram_tail_addra), .dina(ram_tail_dina), .douta(ram_tail_douta),
        .web(1'b0),         .addrb(ram_tail_addrb), .dinb(0),             .doutb(ram_tail_doutb)
    );

    tdp_bram #(.DATA_WIDTH(1), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_valid (
        .clk(clk),
        .wea(ram_valid_wea), .addra(ram_head_addra), .dina(ram_valid_dina), .douta(ram_valid_douta),
        .web(ram_valid_web), .addrb(ram_head_addrb), .dinb(ram_valid_dinb), .doutb(ram_valid_doutb)
    );

    tdp_bram #(.DATA_WIDTH(STAT_WIDTH), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_rxcnt (
        .clk(clk),
        .wea(ram_rxcnt_wea), .addra(ram_head_addra), .dina(ram_rxcnt_dina), .douta(ram_rxcnt_douta),
        .web(1'b0),          .addrb(ram_head_addrb), .dinb(0),              .doutb(ram_rxcnt_doutb)
    );

    tdp_bram #(.DATA_WIDTH(STAT_WIDTH), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_txcnt (
        .clk(clk),
        .wea(1'b0),          .addra(0),              .dina(0),              .douta(),
        .web(ram_txcnt_web), .addrb(ram_head_addrb), .dinb(ram_txcnt_dinb), .doutb(ram_txcnt_doutb)
    );

    tdp_bram #(.DATA_WIDTH(BUFFER_ADDR_WIDTH), .ADDR_WIDTH(BUFFER_ADDR_WIDTH)) u_next (
        .clk(clk),
        .wea(ram_next_wea), .addra(ram_next_addra), .dina(ram_next_dina), .douta(),
        .web(1'b0),         .addrb(ram_next_addrb), .dinb(0),             .doutb(ram_next_doutb)
    );

    // <<< MODIFIED: Connected Port B of Data RAM >>>
    tdp_bram #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(BUFFER_ADDR_WIDTH)) u_data (
        .clk(clk),
        .wea(ram_data_wea), .addra(ram_data_addra), .dina(ram_data_dina), .douta(),
        .web(1'b0),         .addrb(ram_data_addrb), .dinb(0),             .doutb(ram_data_doutb)
    );

    // --------------------------------------------------------------------
    // 3. Centralized Resource Manager (Free List & Stats)
    // --------------------------------------------------------------------
    
    // Free List FIFO Logic (Standard FIFO logic mapped to BRAM)
    assign alloc_cell_ptr_next = free_list_ram[fl_rd_ptr[BUFFER_ADDR_WIDTH-1:0]];

    always @(posedge clk) begin
        // Pipeline the read
        alloc_cell_ptr_reg <= alloc_cell_ptr_next;
        
        if (rst) begin
            fl_wr_ptr <= 0;
            fl_rd_ptr <= 0;
            fl_count  <= BUFFER_DEPTH; 
            total_rx_count <= 0;
            buffer_free_count <= BUFFER_DEPTH;
        end else begin
            // Write (Free)
            if (drn_free_req) begin
                free_list_ram[fl_wr_ptr[BUFFER_ADDR_WIDTH-1:0]] <= return_cell_ptr;
                fl_wr_ptr <= fl_wr_ptr + 1;
            end
            
            // Read (Alloc)
            if (ing_alloc_req) begin
                fl_rd_ptr <= fl_rd_ptr + 1;
            end
            
            // Update Counts
            if (ing_alloc_req && !drn_free_req) begin
                fl_count <= fl_count - 1;
                buffer_free_count <= buffer_free_count - 1;
                total_rx_count <= total_rx_count + 1;
            end else if (!ing_alloc_req && drn_free_req) begin
                fl_count <= fl_count + 1;
                buffer_free_count <= buffer_free_count + 1;
            end else if (ing_alloc_req && drn_free_req) begin
                total_rx_count <= total_rx_count + 1;
            end
        end
    end
    
    // Init Free List
    integer k;
    initial begin
        for (k=0; k<BUFFER_DEPTH; k=k+1) free_list_ram[k] = k;
    end

    // --------------------------------------------------------------------
    // 4. Ingress State Machine (Unchanged)
    // --------------------------------------------------------------------
    localparam S_ING_IDLE  = 0;
    localparam S_ING_DATA  = 1;
    localparam S_ING_RMW_1 = 2; 
    localparam S_ING_RMW_2 = 3; 

    reg [2:0] ing_state = S_ING_IDLE;
    reg [QUEUE_INDEX_WIDTH-1:0] ing_current_vc;
    reg [BUFFER_ADDR_WIDTH-1:0] ing_current_cell;

    always @(posedge clk) begin
        // Reset single-cycle strobes
        ram_head_wea   <= 0;
        ram_tail_wea   <= 0;
        ram_valid_wea  <= 0;
        ram_rxcnt_wea  <= 0;
        ram_next_wea   <= 0;
        ram_data_wea   <= 0;
        ing_alloc_req  <= 0;
        
        if (rst) begin
            ing_state <= S_ING_IDLE;
            s_axis_pkt_tready <= 0;
        end else begin
            case (ing_state)
                S_ING_IDLE: begin
                    // Check if we have free space
                    if (fl_count > 0) begin
                        s_axis_pkt_tready <= 1;
                        
                        if (s_axis_pkt_tvalid) begin
                            // 1. Capture Alloc Ptr
                            ing_current_cell <= alloc_cell_ptr_reg; // Pre-fetched
                            ing_current_vc   <= parsed_vc_idx;
                            
                            // 2. Write Data
                            ram_data_wea   <= 1;
                            ram_data_addra <= alloc_cell_ptr_reg;
                            ram_data_dina  <= s_axis_pkt_tdata;
                            
                            // 3. Request Allocation
                            ing_alloc_req <= 1;
                            
                            // 4. Start Read for State (Head/Tail)
                            ram_head_addra <= parsed_vc_idx;
                            ram_tail_addra <= parsed_vc_idx;
                            
                            if (s_axis_pkt_tlast) begin
                                s_axis_pkt_tready <= 0;
                                ing_state <= S_ING_RMW_1;
                            end else begin
                                ing_state <= S_ING_DATA;
                            end
                        end
                    end else begin
                        s_axis_pkt_tready <= 0;
                    end
                end

                S_ING_DATA: begin
                    if (s_axis_pkt_tvalid) begin
                        if (s_axis_pkt_tlast) begin
                            s_axis_pkt_tready <= 0;
                            ing_state <= S_ING_RMW_1;
                        end
                    end
                end

                S_ING_RMW_1: begin
                    // Wait for BRAM Read (Head, Tail, Valid, RxCnt)
                    ing_state <= S_ING_RMW_2;
                end

                S_ING_RMW_2: begin
                    // Update Stats
                    ram_rxcnt_wea  <= 1;
                    ram_rxcnt_dina <= ram_rxcnt_douta + 1;

                    // Update Pointers
                    if (ram_valid_douta == 1'b0) begin
                        // Queue Was Empty: Init Head & Tail
                        ram_head_wea   <= 1;
                        ram_head_dina  <= ing_current_cell;
                        ram_tail_wea   <= 1;
                        ram_tail_dina  <= ing_current_cell;
                        ram_valid_wea  <= 1;
                        ram_valid_dina <= 1'b1;
                        
                        // Push to Active FIFO
                        active_vc_fifo[active_wr_ptr] <= ing_current_vc;
                        active_wr_ptr <= active_wr_ptr + 1;
                    end else begin
                        // Queue Was Active: Append to Tail
                        // 1. Update Old Tail's Next Ptr -> New Cell
                        ram_next_wea   <= 1;
                        ram_next_addra <= ram_tail_douta; // Old Tail
                        ram_next_dina  <= ing_current_cell;
                        
                        // 2. Update Tail Ptr -> New Cell
                        ram_tail_wea   <= 1;
                        ram_tail_dina  <= ing_current_cell;
                        
                        // Push to Active FIFO
                        active_vc_fifo[active_wr_ptr] <= ing_current_vc;
                        active_wr_ptr <= active_wr_ptr + 1;
                    end
                    
                    ing_state <= S_ING_IDLE;
                end
            endcase
        end
    end

    // --------------------------------------------------------------------
    // 5. Drain State Machine (Modified for Data Read)
    // --------------------------------------------------------------------
    // Ratio Logic
    reg [15:0] ratio_counter;
    reg        drain_trigger;
    always @(posedge clk) begin
        if (rst) begin
            ratio_counter <= 0; drain_trigger <= 0;
        end else begin
            drain_trigger <= 0;
            // Count every time we process a packet in ingress
            if (ing_state == S_ING_RMW_2) begin 
                if (ratio_counter >= DRAIN_RATIO_M - 1) begin
                    ratio_counter <= 0; drain_trigger <= 1;
                end else ratio_counter <= ratio_counter + 1;
            end
        end
    end

    localparam S_DRN_IDLE      = 0;
    localparam S_DRN_READ_HEAD = 1;
    localparam S_DRN_READ_NEXT = 2; // Wait for Head Data
    localparam S_DRN_UPDATE    = 3; // Got Next Ptr, now update

    reg [2:0] drn_state = S_DRN_IDLE;
    reg [QUEUE_INDEX_WIDTH-1:0] drn_vc;
    
    // Temp registers for Read-Modify-Write
    reg [BUFFER_ADDR_WIDTH-1:0] curr_head; 

    always @(posedge clk) begin
        // Reset Strobes
        ram_head_web  <= 0;
        ram_valid_web <= 0;
        ram_txcnt_web <= 0;
        fcp_valid     <= 0;
        drn_free_req  <= 0;
        m_axis_pkt_tvalid <= 0; // Reset output valid

        if (rst) begin
            drn_state <= S_DRN_IDLE;
            active_rd_ptr <= 0;
        end else begin
            // Fetch Active VC (Combinatorial)
            drain_candidate_vc = active_vc_fifo[active_rd_ptr];

            case (drn_state)
                S_DRN_IDLE: begin
                    if (drain_trigger && !active_fifo_empty) begin
                        drn_vc = drain_candidate_vc;
                        active_rd_ptr <= active_rd_ptr + 1;
                        
                        // 1. Read Head Ptr & Tail Ptr (to check empty)
                        ram_head_addrb <= drain_candidate_vc;
                        ram_tail_addrb <= drain_candidate_vc;
                        
                        drn_state <= S_DRN_READ_HEAD;
                    end
                end

                S_DRN_READ_HEAD: begin
                    // Wait for Head/Tail/Valid to appear on doutb
                    drn_state <= S_DRN_READ_NEXT;
                end

                S_DRN_READ_NEXT: begin
                    // Head Ptr is valid now. 
                    if (ram_valid_doutb) begin
                        curr_head = ram_head_doutb;
                        
                        // 2. Read Next Pointer of current Head
                        ram_next_addrb <= curr_head;

                        // <<< MODIFIED: Read Data Payload of current Head >>>
                        ram_data_addrb <= curr_head;
                        
                        drn_state <= S_DRN_UPDATE;
                    end else begin
                        // Should not happen if Active FIFO logic is correct
                        drn_state <= S_DRN_IDLE;
                    end
                end

                S_DRN_UPDATE: begin
                    // Next Ptr is valid now (ram_next_doutb)
                    // Data Payload is valid now (ram_data_doutb)
                    
                    // <<< MODIFIED: Output the drained data >>>
                    m_axis_pkt_tvalid <= 1;
                    m_axis_pkt_tdata  <= ram_data_doutb;

                    // 1. Return Old Head to Free List
                    return_cell_ptr <= curr_head; 
                    drn_free_req    <= 1;

                    // 2. Check if Head == Tail (Queue Becoming Empty)
                    if (curr_head == ram_tail_doutb) begin
                        // Queue Empty
                        ram_valid_web  <= 1;
                        ram_valid_dinb <= 0;
                    end else begin
                        // Update Head = Next
                        ram_head_web   <= 1;
                        ram_head_dinb  <= ram_next_doutb; 
                    end

                    // 3. Update TX Stats
                    ram_txcnt_web  <= 1;
                    ram_txcnt_dinb <= ram_txcnt_doutb + 1;

                    // 4. Output FCP
                    fcp_valid <= 1;
                    fcp_vc    <= drn_vc; 
                    fcp_fccr  <= ram_txcnt_doutb + 1;
                    fcp_qlen  <= ram_rxcnt_doutb - (ram_txcnt_doutb + 1); 
                    fcp_fccl  <= buffer_free_count + total_rx_count;

                    drn_state <= S_DRN_IDLE;
                end
            endcase
        end
    end

endmodule
`resetall