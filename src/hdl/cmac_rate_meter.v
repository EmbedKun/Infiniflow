// SPDX-License-Identifier: BSD-2-Clause-Views
// Author: mkxue-FNIL

`timescale 1ns / 1ps

module cmac_rate_meter #
(
    // Default window: 1 second (Assuming clock is 322.265625 MHz)
    // 322,265,625 cycles
    parameter TIME_WINDOW_CYCLES = 322265625
)
(
    input  wire        clk,          // Connect to usr_tx_clk or usr_rx_clk
    input  wire        rst,
    input  wire        stat_pulse,   // Connect to pulse signals like stat_tx_total_packets
    
    // Signals for VIO monitoring
    output reg [63:0]  cnt_total,    // Accumulated total count (since reset)
    output reg [31:0]  cnt_rate,     // Real-time rate (count within the last time window)
    output reg         update_strobe // Indicates data update to VIO (Optional)
);

    // ------------------------------------------------------
    // 1. Total Counter (64-bit, practically impossible to overflow)
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            cnt_total <= 0;
        end else begin
            if (stat_pulse) begin
                cnt_total <= cnt_total + 1;
            end
        end
    end

    // ------------------------------------------------------
    // 2. Windowed Rate Counter (Calculates packet count per X cycles)
    // ------------------------------------------------------
    reg [31:0] timer_cnt;
    reg [31:0] accumulator; // Value currently accumulating in the current window

    always @(posedge clk) begin
        if (rst) begin
            timer_cnt     <= 0;
            accumulator   <= 0;
            cnt_rate      <= 0;
            update_strobe <= 0;
        end else begin
            update_strobe <= 0; // Default low

            // A. Statistics Logic
            if (stat_pulse) begin
                accumulator <= accumulator + 1;
            end

            // B. Timer Logic
            if (timer_cnt == TIME_WINDOW_CYCLES - 1) begin
                // Time window reached: Latch result, clear accumulator
                cnt_rate      <= accumulator; // Output current count value
                update_strobe <= 1;           // Generate a pulse to indicate update
                
                timer_cnt     <= 0;
                accumulator   <= 0;           // Clear, prepare for next round
            end else begin
                timer_cnt <= timer_cnt + 1;
            end
        end
    end

endmodule