// SPDX-License-Identifier: BSD-2-Clause-Views
// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none

module downstream_switch_model_v2 #
(
    parameter QUEUE_INDEX_WIDTH = 16,
    parameter DATA_WIDTH        = 512, 
    parameter BUFFER_ADDR_WIDTH = 8, 
    parameter STAT_WIDTH        = 32, 
    parameter DRAIN_RATIO_M     = 20, 
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

    // Data Plane Output
    output reg  [DATA_WIDTH-1:0]         m_axis_pkt_tdata,
    output reg                           m_axis_pkt_tvalid,

    // FCP Output
    output reg                           fcp_valid,
    output reg  [QUEUE_INDEX_WIDTH-1:0]  fcp_vc,
    output reg  [STAT_WIDTH-1:0]         fcp_fccl,
    output reg  [STAT_WIDTH-1:0]         fcp_qlen,
    output reg  [STAT_WIDTH-1:0]         fcp_fccr,
    
    // Debug
    output wire [STAT_WIDTH-1:0]         dbg_buffer_free_count,
    output wire [STAT_WIDTH-1:0]         dbg_total_rx_count
);

    localparam BUFFER_DEPTH = 2**BUFFER_ADDR_WIDTH;

    // --------------------------------------------------------------------
    // 1. Signal Declarations
    // --------------------------------------------------------------------
    wire [QUEUE_INDEX_WIDTH-1:0] parsed_vc_idx;
    assign parsed_vc_idx = s_axis_pkt_tdata[32 +: QUEUE_INDEX_WIDTH];

    // --- BRAM Signals  ---
    wire [BUFFER_ADDR_WIDTH-1:0] ram_head_douta, ram_head_doutb;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_head_dina,  ram_head_dinb;
    reg                          ram_head_wea,   ram_head_web;
    reg  [QUEUE_INDEX_WIDTH-1:0] ram_head_addra, ram_head_addrb;

    wire [BUFFER_ADDR_WIDTH-1:0] ram_tail_douta, ram_tail_doutb;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_tail_dina;
    reg                          ram_tail_wea;
    reg  [QUEUE_INDEX_WIDTH-1:0] ram_tail_addra, ram_tail_addrb;

    wire [0:0] ram_valid_douta, ram_valid_doutb;
    reg  [0:0] ram_valid_dina,  ram_valid_dinb;
    reg        ram_valid_wea,   ram_valid_web;

    wire [STAT_WIDTH-1:0] ram_rxcnt_douta, ram_rxcnt_doutb;
    reg  [STAT_WIDTH-1:0] ram_rxcnt_dina;
    reg                       ram_rxcnt_wea;

    wire [STAT_WIDTH-1:0] ram_txcnt_doutb;
    reg  [STAT_WIDTH-1:0] ram_txcnt_dinb;
    reg                       ram_txcnt_web;

    wire [BUFFER_ADDR_WIDTH-1:0] ram_next_doutb;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_next_dina;
    reg                          ram_next_wea;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_next_addra, ram_next_addrb;

    reg  [DATA_WIDTH-1:0]        ram_data_dina;
    reg                          ram_data_wea;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_data_addra;
    reg  [BUFFER_ADDR_WIDTH-1:0] ram_data_addrb; 
    wire [DATA_WIDTH-1:0]        ram_data_doutb; 

    // --- Free List & Active FIFO ---
    reg [BUFFER_ADDR_WIDTH-1:0] free_list_ram [BUFFER_DEPTH-1:0];
    reg [BUFFER_ADDR_WIDTH:0]   fl_wr_ptr = BUFFER_DEPTH, fl_rd_ptr = 0, fl_count = BUFFER_DEPTH;
    
    reg [BUFFER_ADDR_WIDTH-1:0] alloc_cell_ptr_reg;
    wire [BUFFER_ADDR_WIDTH-1:0] alloc_cell_ptr_next;
    
    reg [BUFFER_ADDR_WIDTH-1:0] return_cell_ptr;
    
    reg ing_alloc_req;
    reg drn_free_req;
    
    (* ram_style = "distributed" *) reg [QUEUE_INDEX_WIDTH-1:0] active_vc_fifo [ACTIVE_FIFO_DEPTH-1:0];
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
        .web(1'b0),          .addrb(ram_head_addrb), .dinb(0),             .doutb(ram_rxcnt_doutb)
    );
    tdp_bram #(.DATA_WIDTH(STAT_WIDTH), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_txcnt (
        .clk(clk),
        .wea(1'b0),          .addra(0),              .dina(0),             .douta(),
        .web(ram_txcnt_web), .addrb(ram_head_addrb), .dinb(ram_txcnt_dinb), .doutb(ram_txcnt_doutb)
    );
    tdp_bram #(.DATA_WIDTH(BUFFER_ADDR_WIDTH), .ADDR_WIDTH(BUFFER_ADDR_WIDTH)) u_next (
        .clk(clk),
        .wea(ram_next_wea), .addra(ram_next_addra), .dina(ram_next_dina), .douta(),
        .web(1'b0),         .addrb(ram_next_addrb), .dinb(0),             .doutb(ram_next_doutb)
    );
    tdp_bram #(.DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(BUFFER_ADDR_WIDTH)) u_data (
        .clk(clk),
        .wea(ram_data_wea), .addra(ram_data_addra), .dina(ram_data_dina), .douta(),
        .web(1'b0),         .addrb(ram_data_addrb), .dinb(0),             .doutb(ram_data_doutb)
    );

    // --------------------------------------------------------------------
    // 3. Centralized Resource Manager
    // --------------------------------------------------------------------
    assign alloc_cell_ptr_next = free_list_ram[fl_rd_ptr[BUFFER_ADDR_WIDTH-1:0]];

    always @(posedge clk) begin
        alloc_cell_ptr_reg <= alloc_cell_ptr_next;
        
        if (rst) begin
            fl_wr_ptr <= 0; fl_rd_ptr <= 0; fl_count <= BUFFER_DEPTH; 
            total_rx_count <= 0; buffer_free_count <= BUFFER_DEPTH;
        end else begin
            if (drn_free_req) begin
                free_list_ram[fl_wr_ptr[BUFFER_ADDR_WIDTH-1:0]] <= return_cell_ptr;
                fl_wr_ptr <= fl_wr_ptr + 1;
            end
            
            if (ing_alloc_req) begin
                fl_rd_ptr <= fl_rd_ptr + 1;
            end
            
            // Stats updates
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
    
    integer k;
    initial begin
        for (k=0; k<BUFFER_DEPTH; k=k+1) free_list_ram[k] = k;
    end

    // --------------------------------------------------------------------
    // 4. Ingress State Machine 
    // --------------------------------------------------------------------
    localparam S_ING_IDLE      = 0;
    localparam S_ING_SINK_BODY = 1;
    localparam S_ING_RMW_1     = 2; 
    localparam S_ING_RMW_2     = 3; 

    reg [2:0] ing_state = S_ING_IDLE;
    reg [QUEUE_INDEX_WIDTH-1:0] ing_current_vc;
    reg [BUFFER_ADDR_WIDTH-1:0] ing_current_cell;

    always @(posedge clk) begin
        // Defaults
        ram_head_wea   <= 0; ram_tail_wea   <= 0; ram_valid_wea  <= 0;
        ram_rxcnt_wea  <= 0; ram_next_wea   <= 0; ram_data_wea   <= 0;
        ing_alloc_req  <= 0;
        
        if (rst) begin
            ing_state <= S_ING_IDLE;
            s_axis_pkt_tready <= 0;
            active_wr_ptr <= 0;
        end else begin
            case (ing_state)
                S_ING_IDLE: begin
                    if (fl_count > 0) begin
                        s_axis_pkt_tready <= 1;

                        if (s_axis_pkt_tvalid && s_axis_pkt_tready) begin
                            ing_current_cell <= alloc_cell_ptr_reg; 
                            ing_current_vc   <= parsed_vc_idx;
                            ram_data_wea    <= 1;
                            ram_data_addra  <= alloc_cell_ptr_reg;
                            ram_data_dina   <= s_axis_pkt_tdata;
                            ing_alloc_req   <= 1;
                            ram_head_addra  <= parsed_vc_idx;
                            ram_tail_addra  <= parsed_vc_idx;

                            if (s_axis_pkt_tlast) begin

                                s_axis_pkt_tready <= 0;
                                ing_state <= S_ING_RMW_1;
                            end else begin

                                ing_state <= S_ING_SINK_BODY;
                            end
                        end
                    end else begin
                        s_axis_pkt_tready <= 0;
                    end
                end

                S_ING_SINK_BODY: begin
                    s_axis_pkt_tready <= 1;
                    if (s_axis_pkt_tvalid) begin
                        if (s_axis_pkt_tlast) begin

                            s_axis_pkt_tready <= 0;
                            ing_state <= S_ING_RMW_1;
                        end
                    end
                end

                S_ING_RMW_1: begin
                    ing_state <= S_ING_RMW_2;
                end

                S_ING_RMW_2: begin
                    ram_rxcnt_wea  <= 1;
                    ram_rxcnt_dina <= ram_rxcnt_douta + 1;
                    if (ram_valid_douta == 1'b0) begin
                        ram_head_wea   <= 1; ram_head_dina  <= ing_current_cell;
                        ram_tail_wea   <= 1; ram_tail_dina  <= ing_current_cell;
                        ram_valid_wea  <= 1; ram_valid_dina <= 1'b1;
                    	active_vc_fifo[active_wr_ptr] <= ing_current_vc;
                    	active_wr_ptr <= active_wr_ptr + 1;
                    end else begin
                        ram_next_wea   <= 1; 
                        ram_next_addra <= ram_tail_douta; 
                        ram_next_dina  <= ing_current_cell;
                        
                        ram_tail_wea   <= 1; 
                        ram_tail_dina  <= ing_current_cell;
                    end
                    ing_state <= S_ING_IDLE;
                end
            endcase
        end
    end

    // --------------------------------------------------------------------
    // 5. Drain State Machine
    // --------------------------------------------------------------------
    reg [15:0] drain_timer;
    reg        drain_trigger;
    
    always @(posedge clk) begin
        if (rst) begin
            drain_timer <= 0; 
            drain_trigger <= 0;
        end else begin
            drain_trigger <= 0;
            if (drain_timer >= DRAIN_RATIO_M - 1) begin
                drain_timer <= 0;
                drain_trigger <= 1;
            end else begin
                drain_timer <= drain_timer + 1;
            end
        end
    end

    localparam S_DRN_IDLE      = 0;
    localparam S_DRN_READ_HEAD = 1;
    localparam S_DRN_READ_NEXT = 2; 
    localparam S_DRN_UPDATE    = 3; 

    reg [2:0] drn_state = S_DRN_IDLE;
    reg [QUEUE_INDEX_WIDTH-1:0] drn_vc;
    reg [BUFFER_ADDR_WIDTH-1:0] curr_head; 

    always @(posedge clk) begin
        ram_head_web  <= 0;
        ram_valid_web <= 0;
        ram_txcnt_web <= 0;
        fcp_valid     <= 0;
        drn_free_req  <= 0;
        m_axis_pkt_tvalid <= 0; 

        if (rst) begin
            drn_state <= S_DRN_IDLE;
            active_rd_ptr <= 0;
        end else begin
            drain_candidate_vc = active_vc_fifo[active_rd_ptr];

            case (drn_state)
                S_DRN_IDLE: begin
                    if (drain_trigger && !active_fifo_empty) begin
                        drn_vc = drain_candidate_vc;
                        active_rd_ptr <= active_rd_ptr + 1;
                        
                        ram_head_addrb <= drain_candidate_vc;
                        ram_tail_addrb <= drain_candidate_vc;
                        
                        drn_state <= S_DRN_READ_HEAD;
                    end
                end

                S_DRN_READ_HEAD: begin
                    drn_state <= S_DRN_READ_NEXT;
                end

                S_DRN_READ_NEXT: begin
                    if (ram_valid_doutb) begin
                        curr_head = ram_head_doutb;
                        ram_next_addrb <= curr_head;
                        ram_data_addrb <= curr_head;
                        drn_state <= S_DRN_UPDATE;
                    end else begin
                        drn_state <= S_DRN_IDLE;
                    end
                end

                S_DRN_UPDATE: begin

                    m_axis_pkt_tvalid <= 1;
                    m_axis_pkt_tdata  <= ram_data_doutb; 
                    return_cell_ptr <= curr_head; 
                    drn_free_req    <= 1;

                    if (curr_head == ram_tail_doutb) begin
                        ram_valid_web  <= 1;
                        ram_valid_dinb <= 0;
                    end else begin
                        ram_head_web   <= 1;
                        ram_head_dinb  <= ram_next_doutb; 
                    end
                    ram_txcnt_web  <= 1;
                    ram_txcnt_dinb <= ram_txcnt_doutb + 1;
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