// Author: mkxue-FNIL

`timescale 1ns / 1ps
`default_nettype none

// ========================================================================
// Module: True Dual-Port RAM with Init
// ========================================================================
module tdp_bram #
(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 18,
    parameter INIT_VAL   = 0
)
(
    input  wire                  clk,
    input  wire                  wea,
    input  wire [ADDR_WIDTH-1:0] addra,
    input  wire [DATA_WIDTH-1:0] dina,
    output reg  [DATA_WIDTH-1:0] douta,
    input  wire                  web,
    input  wire [ADDR_WIDTH-1:0] addrb,
    input  wire [DATA_WIDTH-1:0] dinb,
    output reg  [DATA_WIDTH-1:0] doutb
);
    (* ram_style = "block" *) 
    reg [DATA_WIDTH-1:0] ram [2**ADDR_WIDTH-1:0];

    always @(posedge clk) begin
        if (wea) ram[addra] <= dina;
        douta <= ram[addra];
    end

    always @(posedge clk) begin
        if (web) ram[addrb] <= dinb;
        doutb <= ram[addrb];
    end
    
    integer i;
    initial begin
        for (i=0; i<2**ADDR_WIDTH; i=i+1) ram[i] = INIT_VAL;
    end
    
endmodule