`timescale 1ns/1fs
// Behavioural transport for the benches: own free-running clock with a
// ppm offset. MODE 0: I2S PCM (32-bit slots, sine FL / FR at AMP),
// MODE 1: native DSD (Amanero: LRCK pin = left, SDATA = right) from FILE,
// MODE 2: DoP from FILE. EN = 0 keeps every line low (no clock).
module tb_src #(
    parameter integer MODE = 0,
    parameter real    FS   = 44100.0,
    parameter real    PPM  = 0.0,
    parameter real    FL   = 997.0,
    parameter real    FR   = 3001.0,
    parameter real    AMP  = 0.891,
    parameter integer DSDN = 1,
    parameter         FILE = "build/dsd.hex",
    parameter real    T0   = 1234.5
) (
    input  wire en,
    output reg  bclk = 1'b0,
    output reg  lrck = 1'b0,
    output reg  sdata = 1'b0,
    output wire dsd_on
);
    assign dsd_on = (MODE == 1) && en;
    real half;
    initial begin
        if (MODE == 1) half = 1.0e9 / (FS * (1.0 + PPM * 1.0e-6)) / 2.0;
        else           half = 1.0e9 / (FS * 64.0 * (1.0 + PPM * 1.0e-6)) / 2.0;
        #(T0);
        forever begin
            #(half);
            if (en) bclk = ~bclk;
            else    bclk = 1'b0;
        end
    end

    reg [31:0] dsdw [0:DSDN-1];
    initial if (MODE != 0) $readmemh(FILE, dsdw);

    reg [5:0]  slot = 6'd0;
    reg [31:0] wl = 0, wr = 0;
    reg        wr_lsb = 1'b0;
    reg        dop_m = 1'b0;
    integer    nfr = 0, dsdi = 0;
    real       tt;
    function [31:0] pcm32;
        input real v;
        integer iv;
        begin
            iv = $rtoi(v * 8388607.0);
            pcm32 = {iv[23:0], 8'h00};
        end
    endfunction
    always @(negedge bclk) begin
        if (MODE == 1) begin
            lrck  <= dsdw[(dsdi / 16) % DSDN][31 - (dsdi % 16)];
            sdata <= dsdw[(dsdi / 16) % DSDN][15 - (dsdi % 16)];
            dsdi = dsdi + 1;
        end else begin
            if (slot == 6'd0) begin
                if (MODE == 0) begin
                    tt = nfr / FS;
                    wl = pcm32(AMP * $sin(2.0 * 3.14159265358979 * FL * tt));
                    wr = pcm32(AMP * $sin(2.0 * 3.14159265358979 * FR * tt));
                end else begin
                    wl = {(dop_m ? 8'hFA : 8'h05), dsdw[nfr % DSDN][31:16], 8'h00};
                    wr = {(dop_m ? 8'hFA : 8'h05), dsdw[nfr % DSDN][15:0], 8'h00};
                    dop_m = ~dop_m;
                end
                nfr = nfr + 1;
            end
            lrck <= slot[5];
            if (slot == 6'd0)       sdata <= wr_lsb;
            else if (slot <= 6'd32) sdata <= wl[32 - slot];
            else                    sdata <= wr[64 - slot];
            if (slot == 6'd63) wr_lsb <= wr[0];
            slot <= slot + 6'd1;
        end
    end
endmodule
