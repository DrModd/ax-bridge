`timescale 1ns/1fs
// System bench: behavioural transport (own clock with a ppm offset) ->
// top -> I2S output decoded from the pins and dumped to build/sys_out.txt
// while audio_on is high. Analysis: tb/run_sys.py.
//   MODE 0: PCM I2S, 32-bit slots, FS Hz, sine FL on L / FR on R, AMP dBFS
//   MODE 1: native DSD (Amanero), bit rate FS Hz, bits from build/dsd.hex
//   MODE 2: DoP, frame rate FS Hz, bits from build/dsd.hex
module tb_sys;
    parameter integer MODE = 0;
    parameter real    FS   = 44100.0;
    parameter real    PPM  = 37.0;
    parameter real    FL   = 997.0;
    parameter real    FR   = 3001.0;
    parameter real    AMP  = 0.891;          // -1 dBFS
    parameter integer NOUT = 8192;           // samples to capture
    parameter integer DSDN = 1;              // 16-bit words in dsd.hex
    parameter real    TMAX_MS = 400.0;
    parameter integer TB_WBITS = 16;         // 18 = silicon
    parameter integer TB_GSH   = 4;          // 0 = silicon
    parameter real    PHL      = 0.0;        // phase of the left tone (rad)
    parameter real    AMPR     = -1.0;       // right amplitude (<0: = AMP)
    parameter integer ATT      = 0;          // PCM headroom (0.5 dB steps)

    // ---------------- local 49.152 MHz ----------------
    reg mclk = 0;
    always #10.1725 mclk = ~mclk;

    // ---------------- transport ----------------
    reg bclk = 0, lrck = 0, sdata = 0, dsd_on = 0;
    real tb_half;        // half period of the transport bit clock (ns)
    initial begin
        if (MODE == 1)
            tb_half = 1.0e9 / (FS * (1.0 + PPM * 1.0e-6)) / 2.0;
        else
            tb_half = 1.0e9 / (FS * 64.0 * (1.0 + PPM * 1.0e-6)) / 2.0;
        dsd_on = (MODE == 1);
        #1234.5;
        forever begin
            #(tb_half) bclk = ~bclk;
        end
    end

    reg [31:0] dsdw [0:DSDN-1];
    initial if (MODE != 0) $readmemh("build/dsd.hex", dsdw);

    // PCM / DoP framing on falling edges; slot bit b = 0..63
    reg [5:0]  slot = 6'd0;
    reg [31:0] wl, wr;
    integer    nfr = 0;
    integer    dsdi = 0;     // DSD bit index
    reg        dop_m = 1'b0;
    reg        wr_prev_lsb = 1'b0;
    real       tt;
    function [31:0] pcm32;
        input real v;
        integer iv;
        begin
            iv = $rtoi(v * 8388607.0);
            if (iv > 8388607)  iv = 8388607;
            if (iv < -8388608) iv = -8388608;
            pcm32 = {iv[23:0], 8'h00};
        end
    endfunction
    always @(negedge bclk) begin
        if (MODE == 1) begin
            // native DSD: LRCK pin = left, SDATA = right (PureFox wiring)
            lrck  <= dsdw[(dsdi / 16) % DSDN][31 - (dsdi % 16)];
            sdata <= dsdw[(dsdi / 16) % DSDN][15 - (dsdi % 16)];
            dsdi = dsdi + 1;
        end else begin
            if (slot == 6'd0) begin
                if (MODE == 0) begin
                    tt = nfr / FS;
                    wl = pcm32(AMP * $sin(2.0 * 3.14159265358979 * FL * tt + PHL));
                    wr = pcm32(((AMPR < 0.0) ? AMP : AMPR) * $sin(2.0 * 3.14159265358979 * FR * tt));
                end else begin
                    wl = {(dop_m ? 8'hFA : 8'h05), dsdw[nfr % DSDN][31:16], 8'h00};
                    wr = {(dop_m ? 8'hFA : 8'h05), dsdw[nfr % DSDN][15:0], 8'h00};
                    dop_m = ~dop_m;
                end
                nfr = nfr + 1;
            end
            lrck <= slot[5];
            // Philips: MSB one BCLK after the LRCK edge
            if (slot == 6'd0)       sdata <= wr_prev_lsb;
            else if (slot <= 6'd32) sdata <= wl[32 - slot];
            else                    sdata <= wr[64 - slot];
            slot <= slot + 6'd1;
        end
    end
    always @(negedge bclk) if (MODE != 1 && slot == 6'd63) wr_prev_lsb <= wr[0];

    // ---------------- DUT ----------------
    wire obclk, olrck, osdata, audio_on;
    tri1 i2c_sda;
    top #(.WBITS(TB_WBITS), .GSH(TB_GSH), .ATT_DEFAULT(ATT)) dut (
        .mclk_in(mclk), .i2s_bclk_in(bclk), .i2s_lrck_in(lrck),
        .i2s_sdata_in(sdata), .dsd_on(dsd_on),
        .in1_bclk(1'b0), .in1_lrck(1'b0), .in1_sdata(1'b0),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0),
        .i2c_scl(1'b1), .i2c_sda(i2c_sda),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    // ---------------- output decoder (pins) ----------------
    reg        olr_q = 0;
    reg [31:0] osr = 0;
    reg [31:0] ow_l = 0;
    integer    ncap = 0;
    integer    fo;
    real       t_on = -1.0;
    initial fo = $fopen("build/sys_out.txt", "w");
    always @(posedge obclk) begin
        olr_q <= olrck;
        if (olrck != olr_q) begin
            if (olr_q == 1'b0) ow_l <= {osr[30:0], osdata};
            else if (audio_on) begin
                $fdisplay(fo, "%0d %0d", $signed(ow_l), $signed({osr[30:0], osdata}));
                ncap = ncap + 1;
            end
            osr <= 32'd0;
        end else begin
            osr <= {osr[30:0], osdata};
        end
    end
    always @(posedge audio_on) begin
        if (t_on < 0) t_on = $realtime;
        $display("audio_on at %0.2f ms (path %0d k %0d fam48 %0d)", $realtime / 1.0e6,
                 dut.path, dut.cfg_k, dut.cfg_fam48);
    end
    always @(negedge audio_on)
        if (t_on >= 0) $display("audio_off at %0.2f ms", $realtime / 1.0e6);

    initial begin
        #(TMAX_MS * 1.0e6);
        $display("TIMEOUT captured %0d, clip sticky %0d", ncap, dut.clip_sticky);
        $fclose(fo);
        $finish;
    end
    always @(posedge mclk) if (ncap >= NOUT) begin
        $display("CAPTURED %0d samples, gear %0d, clip sticky %0d", ncap, dut.a_gear, dut.clip_sticky);
        $fclose(fo);
        $finish;
    end
endmodule
