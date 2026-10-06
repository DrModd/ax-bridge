`timescale 1ns/1ps
// Input-rate classifier (local MCLK domain, 49.152 MHz).
// Counts FIFO items in a window of 2^WBITS MCLK and matches the count to
// 44.1k*2^k or 48k*2^k items per second (+-1/32 tolerance), k = 0..5.
//   PCM / DoP : one item per stereo frame  -> k = 0 (44.1/48) .. 4 (705.6/768)
//   native DSD: one item per 16 bit-clocks -> k = 2 (DSD64) .. 5 (DSD512)
// A class is reported stable after two equal windows; any other window
// result clears 'stable' at once. 'seq' toggles at every window end.
module rate_mon #(
    parameter WBITS = 18      // 2^18 MCLK = 5.33 ms
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       item,       // one strobe per FIFO item
    input  wire       item_dsd,   // tag of that item
    input  wire       dop,        // DoP detector state
    output reg        stable,
    output reg  [2:0] k,
    output reg        fam48,
    output reg        dsd,        // native DSD
    output reg        is_dop,
    output reg        seq
);
    // nominal items per window at 44.1k / 48k (k = 0)
    localparam integer N44 = ((1 << WBITS) * 441 + 245760) / 491520;
    localparam integer N48 = (1 << WBITS) / 1024;
    localparam integer CW = WBITS - 4;     // counter width (768k max)

    reg [WBITS-1:0] wcnt;
    reg [CW-1:0]    icnt;
    reg             saw_pcm, saw_dsd, dop_all;

    function [4:0] classify;   // {valid, fam48, k[2:0]}
        input [CW-1:0] c;
        integer j;
        integer n44, n48;
        reg [4:0] r;
        begin
            r = 5'd0;
            for (j = 0; j < 6; j = j + 1) begin
                n44 = N44 << j;
                n48 = N48 << j;
                if ((c >= n44 - (n44 >> 5)) && (c <= n44 + (n44 >> 5)))
                    r = {1'b1, 1'b0, j[2:0]};
                if ((c >= n48 - (n48 >> 5)) && (c <= n48 + (n48 >> 5)))
                    r = {1'b1, 1'b1, j[2:0]};
            end
            classify = r;
        end
    endfunction

    wire [4:0] cl = classify(icnt);
    // DSD items with PCM items in the same window -> mixed (invalid)
    wire        ok = cl[4] && !(saw_pcm && saw_dsd);
    wire [5:0]  key = {cl[3:0], saw_dsd, dop_all & ~saw_dsd};
    reg  [5:0]  last_key;
    reg         last_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wcnt <= {WBITS{1'b0}};
            icnt <= {CW{1'b0}};
            saw_pcm <= 1'b0; saw_dsd <= 1'b0; dop_all <= 1'b1;
            stable <= 1'b0; k <= 3'd0; fam48 <= 1'b0; dsd <= 1'b0;
            is_dop <= 1'b0; seq <= 1'b0;
            last_key <= 6'd0; last_ok <= 1'b0;
        end else begin
            wcnt <= wcnt + 1'b1;
            if (&wcnt) begin
                seq <= ~seq;
                last_key <= key;
                last_ok <= ok;
                if (ok && last_ok && (key == last_key)) begin
                    stable <= 1'b1;
                    {fam48, k} <= cl[3:0];
                    dsd <= saw_dsd;
                    is_dop <= dop_all & ~saw_dsd;
                end else begin
                    stable <= 1'b0;
                end
                icnt <= {{(CW-1){1'b0}}, item};
                saw_pcm <= item & ~item_dsd;
                saw_dsd <= item & item_dsd;
                dop_all <= dop;
            end else begin
                if (item) begin
                    if (~&icnt)
                        icnt <= icnt + 1'b1;
                    if (item_dsd) saw_dsd <= 1'b1;
                    else          saw_pcm <= 1'b1;
                end
                if (!dop)
                    dop_all <= 1'b0;
            end
        end
    end
endmodule
