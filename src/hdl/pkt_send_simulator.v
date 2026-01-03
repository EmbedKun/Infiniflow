// SPDX-License-Identifier: BSD-2-Clause-Views
/*
 * Packet Send Simulator (Enhanced with Credit & Threshold Flow Control)
 * - Fixed: Added "Resume" logic. When FCP updates allow a stopped queue to send,
 * a Doorbell is triggered to re-activate it in the scheduler.
 */
// Author: mkxue-FNIL

`resetall
`timescale 1ns / 1ps
`default_nettype none

module pkt_send_simulator #
(
    parameter QUEUE_INDEX_WIDTH = 16, 
    parameter REQ_TAG_WIDTH = 8,
    parameter LEN_WIDTH = 16,
    parameter DATA_WIDTH = 512,
    parameter PKT_LEN_BYTES = 1536,
    parameter BUFFER_ADDR_WIDTH = 14, 
    parameter QMAX = 6,
    parameter QMIN = 2,
    parameter IGNORE_FCP_MODE = 1
)
(
    input  wire                          clk,
    input  wire                          rst,

    // Scheduler Interface
    input  wire [QUEUE_INDEX_WIDTH-1:0]  s_axis_tx_req_queue,
    input  wire [REQ_TAG_WIDTH-1:0]      s_axis_tx_req_tag,
    input  wire                          s_axis_tx_req_valid,
    output reg                           s_axis_tx_req_ready,

    output reg  [LEN_WIDTH-1:0]          m_axis_tx_req_status_len,
    output reg  [REQ_TAG_WIDTH-1:0]      m_axis_tx_req_status_tag,
    output reg                           m_axis_tx_req_status_valid,

    output reg  [QUEUE_INDEX_WIDTH-1:0]  m_axis_doorbell_queue,
    output reg                           m_axis_doorbell_valid,

    // Control Interface
    input  wire [QUEUE_INDEX_WIDTH-1:0]  stop_queue_idx,
    input  wire                          stop_cmd_valid,

    // Packet Output
    output reg  [DATA_WIDTH-1:0]         m_axis_pkt_tdata,
    output reg                           m_axis_pkt_tvalid,
    output reg                           m_axis_pkt_tlast,
    output reg  [DATA_WIDTH/8-1:0]       m_axis_pkt_tkeep,
    input  wire                          m_axis_pkt_tready,
    
    // Statistics Outputs
    output reg  [63:0]                   m_axis_tx_pkt_count,
    output reg  [31:0]                   m_axis_vc_tx_count, 

    // FCP Input Interface
    input  wire                          fcp_valid,
    input  wire [QUEUE_INDEX_WIDTH-1:0]  fcp_vc,
    input  wire [31:0]                   fcp_fccl,
    input  wire [31:0]                   fcp_qlen,
    input  wire [31:0]                   fcp_fccr,
    
    output wire [31:0]                   dbg_global_credit
);

    localparam QUEUE_COUNT = 2**QUEUE_INDEX_WIDTH;
    localparam PKT_WORDS = (PKT_LEN_BYTES + (DATA_WIDTH/8) - 1) / (DATA_WIDTH/8);
//    localparam PKT_WORDS = 1;
    localparam INITIAL_CREDIT = 2**BUFFER_ADDR_WIDTH;

    // ========================================================================
    // 1. BRAM Definitions
    // ========================================================================

    // VC Tx Count RAM
    reg  [31:0] tx_cnt_dina;
    wire [31:0] tx_cnt_douta, tx_cnt_doutb;
    reg  [QUEUE_INDEX_WIDTH-1:0] tx_cnt_addra, tx_cnt_addrb;
    reg  tx_cnt_wea;

    tdp_bram #(.DATA_WIDTH(32), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_tx_cnt (
        .clk(clk),
        .wea(tx_cnt_wea), .addra(tx_cnt_addra), .dina(tx_cnt_dina), .douta(tx_cnt_douta),
        .web(1'b0),       .addrb(tx_cnt_addrb), .dinb(0),           .doutb(tx_cnt_doutb)
    );

    // FCCR Cache RAM
    reg  [31:0] fccr_dinb;
    wire [31:0] fccr_douta; 
    reg  [QUEUE_INDEX_WIDTH-1:0] fccr_addra, fccr_addrb;
    reg  fccr_web; 

    tdp_bram #(.DATA_WIDTH(32), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_fccr_cache (
        .clk(clk),
        .wea(1'b0),       .addra(fccr_addra), .dina(0),         .douta(fccr_douta),
        .web(fccr_web),   .addrb(fccr_addrb), .dinb(fccr_dinb), .doutb()
    );

    // Threshold RAM
    reg  [31:0] thresh_dinb;
    wire [31:0] thresh_douta, thresh_doutb; 
    reg  [QUEUE_INDEX_WIDTH-1:0] thresh_addra, thresh_addrb;
    reg  thresh_web;

    tdp_bram #(.DATA_WIDTH(32), .ADDR_WIDTH(QUEUE_INDEX_WIDTH), .INIT_VAL(INITIAL_CREDIT)) u_thresh (
        .clk(clk),
        .wea(1'b0),       .addra(thresh_addra), .dina(0),           .douta(thresh_douta),
        .web(thresh_web), .addrb(thresh_addrb), .dinb(thresh_dinb), .doutb(thresh_doutb)
    );

    // Host Stop Mask RAM
    wire [0:0] stop_douta;
    reg  stop_web;
    reg  [QUEUE_INDEX_WIDTH-1:0] stop_addrb;
    
    tdp_bram #(.DATA_WIDTH(1), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_stop (
        .clk(clk),
        .wea(1'b0),     .addra(s_axis_tx_req_queue), .dina(0),   .douta(stop_douta),
        .web(stop_web), .addrb(stop_addrb),          .dinb(1'b1),.doutb()
    );
    
    // <<< NEW: Flow Control Stop Mask RAM >>>
    // Records if a queue is stopped due to FC constraints.
    // Port A: Tx Engine (Write 1 to Stop)
    // Port B: FCP Engine (Read to Check, Write 0 to Resume)
    reg  fc_stop_wea, fc_stop_web;
    reg  [QUEUE_INDEX_WIDTH-1:0] fc_stop_addra, fc_stop_addrb;
    reg  [0:0] fc_stop_dina, fc_stop_dinb;
    wire [0:0] fc_stop_doutb;

    tdp_bram #(.DATA_WIDTH(1), .ADDR_WIDTH(QUEUE_INDEX_WIDTH)) u_fc_stop (
        .clk(clk),
        .wea(fc_stop_wea), .addra(fc_stop_addra), .dina(fc_stop_dina), .douta(),
        .web(fc_stop_web), .addrb(fc_stop_addrb), .dinb(fc_stop_dinb), .doutb(fc_stop_doutb)
    );

    // ========================================================================
    // 2. Global Logic
    // ========================================================================
    reg [31:0] global_credit;
    reg [QUEUE_INDEX_WIDTH:0] init_ptr;
    reg initializing;
    reg doorbell_req_fcp; // Doorbell trigger from FCP logic

    always @(posedge clk) begin
        // Host Stop Logic
        stop_web <= stop_cmd_valid;
        stop_addrb <= stop_queue_idx;

        if (rst) begin
            init_ptr <= 0;
            initializing <= 1;
            m_axis_doorbell_valid <= 0;
            m_axis_tx_pkt_count <= 0;
            global_credit <= INITIAL_CREDIT;
        end else begin
            // Priority Doorbell Logic
            if (initializing) begin
                m_axis_doorbell_valid <= 1;
                m_axis_doorbell_queue <= init_ptr[QUEUE_INDEX_WIDTH-1:0];
                if (init_ptr == QUEUE_COUNT-1) initializing <= 0;
                else init_ptr <= init_ptr + 1;
            end else if (doorbell_req_fcp) begin
                // Resume Doorbell from FCP Logic
                m_axis_doorbell_valid <= 1;
                m_axis_doorbell_queue <= lat_vc; // From FCP FSM
            end else begin
                m_axis_doorbell_valid <= 0;
            end

            // Credit Update
            if (fcp_valid) global_credit <= fcp_fccl - m_axis_tx_pkt_count[31:0];
            else if (m_axis_pkt_tvalid && m_axis_pkt_tlast && m_axis_pkt_tready) begin
                global_credit <= global_credit - 1;
                m_axis_tx_pkt_count <= m_axis_tx_pkt_count + 1;
            end
        end
    end

    // ========================================================================
    // 3. FCP Handler FSM (Updates & Resumes)
    // ========================================================================
    localparam FCP_IDLE = 0;
    localparam FCP_READ = 1;
    localparam FCP_CALC = 2;
    localparam FCP_RESUME_CHECK = 3;
    
    reg [1:0] fcp_state;
    reg [QUEUE_INDEX_WIDTH-1:0] lat_vc;
    reg [31:0] lat_qlen, lat_fccr;
    
    reg [31:0] fcp_calc_inflight;
    reg [31:0] fcp_calc_new_thresh;
    reg [31:0] fcp_calc_old_thresh;

    reg [15:0] fcp_calc_inflight_16;
    reg [15:0] fcp_calc_new_thresh_16;
    reg [15:0] fcp_calc_old_thresh_16;
    
    always @(posedge clk) begin
        fccr_web   <= 0;
        thresh_web <= 0;
        fc_stop_web <= 0;
        doorbell_req_fcp <= 0;
        
        fcp_calc_inflight = 0;
        fcp_calc_new_thresh = 0;
        fcp_calc_old_thresh = 0;
        
        if (rst) begin
            fcp_state <= FCP_IDLE;
        end else begin
            case (fcp_state)
                FCP_IDLE: begin
                    if (fcp_valid) begin
                        lat_vc   <= fcp_vc;
                        lat_qlen <= fcp_qlen;
                        lat_fccr <= fcp_fccr;
                        
                        // Start Read (Threshold, TxCount, FC_Stop_Status)
                        thresh_addrb  <= fcp_vc;
                        tx_cnt_addrb  <= fcp_vc; 
                        fc_stop_addrb <= fcp_vc; // Check if it was stopped
                        
                        fcp_state <= FCP_READ;
                    end
                end
                
                FCP_READ: begin
                    fcp_state <= FCP_CALC;
                end
                
                FCP_CALC: begin
                    // 1. Calc Inflight & Threshold (Same as before)
                    fccr_web  <= 1; fccr_addrb <= lat_vc; fccr_dinb  <= lat_fccr;
                    
//                    fcp_calc_inflight = tx_cnt_doutb - lat_fccr;
//                    fcp_calc_old_thresh = thresh_doutb;
                    fcp_calc_inflight_16 = tx_cnt_doutb[15:0] - lat_fccr[15:0];
                    fcp_calc_old_thresh_16 = thresh_doutb[15:0];

                    if (lat_qlen > QMAX) fcp_calc_new_thresh_16 = fcp_calc_inflight_16 - lat_qlen[15:0] + QMIN;
                    else if (lat_qlen < QMIN) fcp_calc_new_thresh_16 = fcp_calc_old_thresh_16 - lat_qlen[15:0] + QMIN;
                    else fcp_calc_new_thresh_16 = fcp_calc_old_thresh_16;
                    
                    if (fcp_calc_new_thresh_16 < 1) fcp_calc_new_thresh_16 = 1;

                    thresh_web   <= 1; thresh_addrb <= lat_vc; thresh_dinb  <= {16'd0,fcp_calc_new_thresh_16};
                    
                    // 2. CHECK RESUME CONDITION
                    // If queue was stopped AND now (Inflight < New_Threshold)
                    // Note: fc_stop_doutb is valid from READ state
                    if (fc_stop_doutb == 1'b1 && fcp_calc_inflight_16 < fcp_calc_new_thresh_16) begin
                        // Need to resume!
                        // Clear Stop Flag
                        fc_stop_web  <= 1;
                        fc_stop_addrb <= lat_vc;
                        fc_stop_dinb <= 0; 
                        
                        // Ring Doorbell
                        doorbell_req_fcp <= 1;
                    end
                    
                    fcp_state <= FCP_IDLE;
                end
            endcase
        end
    end

    // ========================================================================
    // 4. Tx Engine FSM (Main Logic)
    // ========================================================================
    localparam TX_IDLE      = 0;
    localparam TX_READ_INFO = 1;
    localparam TX_CHECK     = 2;
    localparam TX_SEND_PKT  = 3;
    localparam TX_UPDATE    = 4;
                 
    reg [2:0] tx_state;
    reg [QUEUE_INDEX_WIDTH-1:0] tx_q;
    reg [REQ_TAG_WIDTH-1:0]     tx_tag;
    reg [31:0]                  word_cnt;

    reg [15:0] tx_calc_inflight_16;
    reg doorbell_req_tx;

    always @(posedge clk) begin
        tx_cnt_wea <= 0; fc_stop_wea <= 0;
        m_axis_pkt_tvalid <= 0; m_axis_pkt_tlast <= 0; m_axis_tx_req_status_valid <= 0;
        tx_calc_inflight_16 = 0;
        
        if (rst) begin
            tx_state <= TX_IDLE; s_axis_tx_req_ready <= 0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    if (!initializing) begin
                        
                        if (s_axis_tx_req_valid) begin
                            tx_q   <= s_axis_tx_req_queue;
                            tx_tag <= s_axis_tx_req_tag;
                            tx_cnt_addra <= s_axis_tx_req_queue;
                            fccr_addra   <= s_axis_tx_req_queue;
                            thresh_addra <= s_axis_tx_req_queue;
                            
                            tx_state <= TX_READ_INFO;
                            s_axis_tx_req_ready <= 1;
                            
                        end
                    end
                end
                
                TX_READ_INFO: begin
                    tx_state <= TX_CHECK;
                    s_axis_tx_req_ready <= 0;
                end
                
                TX_CHECK: begin
                    tx_calc_inflight_16 = tx_cnt_douta[15:0] - fccr_douta[15:0];
                    if ( (IGNORE_FCP_MODE == 1'b1 || ((tx_calc_inflight_16 < thresh_douta[15:0]) && (global_credit > 2))) && (stop_douta == 1'b0) ) begin
//                    if ((tx_calc_inflight < thresh_douta) && (global_credit > 2) && (stop_douta == 1'b0)) begin
                        // Pass
                        word_cnt <= 0;
                        tx_state <= TX_SEND_PKT;
                    end else begin
                        // Fail -> Stop Queue (Temporarily)
                        m_axis_tx_req_status_len   <= 0; 
                        m_axis_tx_req_status_tag   <= tx_tag;
                        m_axis_tx_req_status_valid <= 1;
                        
                        // <<< Mark as FC Stopped >>>
                        fc_stop_wea   <= 1;
                        fc_stop_addra <= tx_q;
                        fc_stop_dina  <= 1;
                        
                        tx_state <= TX_IDLE;
                    end
                end
                
                TX_SEND_PKT: begin
                    if (m_axis_pkt_tready) begin
                        m_axis_pkt_tvalid <= 1;
                        m_axis_pkt_tdata  <= {{(DATA_WIDTH-QUEUE_INDEX_WIDTH-32){1'b0}}, tx_q, word_cnt};
                        m_axis_pkt_tkeep  <= {DATA_WIDTH/8{1'b1}};
                        if (word_cnt == PKT_WORDS-1) begin
                            m_axis_pkt_tlast <= 1;
                            tx_state <= TX_UPDATE;
                        end else begin
                            word_cnt <= word_cnt + 1;
                        end
                    end
                end
                
                TX_UPDATE: begin
                    tx_cnt_wea  <= 1;
                    tx_cnt_dina <= tx_cnt_douta + 1;
                    tx_cnt_addra<= tx_q; 
                    m_axis_tx_req_status_len   <= PKT_LEN_BYTES;
                    m_axis_tx_req_status_tag   <= tx_tag;
                    m_axis_tx_req_status_valid <= 1;
                    m_axis_vc_tx_count <= tx_cnt_douta + 1;
                    tx_state <= TX_IDLE;
                end
            endcase
        end
    end

endmodule
`resetall