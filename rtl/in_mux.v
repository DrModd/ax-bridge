`timescale 1ns/1ps
// 4-input I2S / DSD selector.
//   input 0: Fox (PCM, DoP, native DSD with dsd_on)
//   inputs 1..3: I2S PCM (DoP is recognised downstream on any input)
// 'sel' comes from the local MCLK domain and only changes while the
// controller holds the whole input front end (bclk_rx + FIFO write side)
// in reset, so a glitch on the muxed bit clock at the switch instant is
// harmless.
//   USE_DCS = 0: plain LUT multiplexer (portable; Gowin promotes the
//                result to a global clock net).
//   USE_DCS = 1: Gowin DCS dynamic clock selector in force mode for the
//                bit clock (check the primitive name/ports against the
//                Gowin primitive guide of your IDE version before use).
// Also: per-input activity flags (LRCK toggling within ~10.7 ms),
// measured on the local clock.
module in_mux #(
    parameter USE_DCS = 0
) (
    input  wire       clk,         // local MCLK
    input  wire       rst_n,
    input  wire [1:0] sel,
    input  wire [3:0] bclk_in,
    input  wire [3:0] lrck_in,
    input  wire [3:0] sdata_in,
    input  wire       dsd_on0,     // input 0 only
    output wire       bclk,
    output wire       lrck,
    output wire       sdata,
    output wire       dsd_on,
    output reg  [3:0] active
);
    generate
        if (USE_DCS) begin : g_dcs
            wire [3:0] onehot = 4'b0001 << sel;
            DCS #(.DCS_MODE("RISING")) u_dcs (
                .CLKIN0(bclk_in[0]), .CLKIN1(bclk_in[1]),
                .CLKIN2(bclk_in[2]), .CLKIN3(bclk_in[3]),
                .CLKSEL(onehot), .SELFORCE(1'b1), .CLKOUT(bclk)
            );
        end else begin : g_lut
            assign bclk = bclk_in[sel];
        end
    endgenerate
    assign lrck   = lrck_in[sel];
    assign sdata  = sdata_in[sel];
    assign dsd_on = (sel == 2'd0) & dsd_on0;

    // activity: any LRCK edge within 2^19 MCLK
    reg [3:0] s1, s2, s3;
    reg [18:0] tmo0, tmo1, tmo2, tmo3;
    wire [3:0] edge_ = s2 ^ s3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1 <= 4'd0; s2 <= 4'd0; s3 <= 4'd0; active <= 4'd0;
            tmo0 <= 19'd0; tmo1 <= 19'd0; tmo2 <= 19'd0; tmo3 <= 19'd0;
        end else begin
            s1 <= lrck_in; s2 <= s1; s3 <= s2;
            if (edge_[0]) begin tmo0 <= 19'h7FFFF; active[0] <= 1'b1; end
            else if (tmo0 != 0) tmo0 <= tmo0 - 1'b1; else active[0] <= 1'b0;
            if (edge_[1]) begin tmo1 <= 19'h7FFFF; active[1] <= 1'b1; end
            else if (tmo1 != 0) tmo1 <= tmo1 - 1'b1; else active[1] <= 1'b0;
            if (edge_[2]) begin tmo2 <= 19'h7FFFF; active[2] <= 1'b1; end
            else if (tmo2 != 0) tmo2 <= tmo2 - 1'b1; else active[2] <= 1'b0;
            if (edge_[3]) begin tmo3 <= 19'h7FFFF; active[3] <= 1'b1; end
            else if (tmo3 != 0) tmo3 <= tmo3 - 1'b1; else active[3] <= 1'b0;
        end
    end
endmodule
