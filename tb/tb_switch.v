`timescale 1ns/1fs
// Format-switch bench: the transport changes rate / format on the fly
// (with and without a clock stop in between, DSD_ON toggling, DoP).
// Checks: the bridge drops audio_on, relocks on the new format with the
// expected path, and the output never exceeds the programme level (no
// clicks / garbage bursts while switching).
module tb_switch;
    parameter integer DSDN = 1;
    reg mclk = 0;
    always #10.1725 mclk = ~mclk;

    // segment table: mode (0 PCM, 1 DSD, 2 DoP), rate, length ms, gap ms
    integer seg = 0;
    integer smode [0:5];
    real    srate [0:5];
    real    slen  [0:5];
    real    sgap  [0:5];
    integer spath [0:5];
    initial begin
        smode[0] = 0; srate[0] = 44100.0;    slen[0] = 45; sgap[0] = 3;   spath[0] = 0;
        smode[1] = 0; srate[1] = 96000.0;    slen[1] = 45; sgap[1] = 0;   spath[1] = 1;
        smode[2] = 1; srate[2] = 2822400.0;  slen[2] = 45; sgap[2] = 0;   spath[2] = 5;
        smode[3] = 0; srate[3] = 768000.0;   slen[3] = 45; sgap[3] = 2;   spath[3] = 4;
        smode[4] = 2; srate[4] = 176400.0;   slen[4] = 45; sgap[4] = 0;   spath[4] = 5;
        smode[5] = 0; srate[5] = 192000.0;   slen[5] = 45; sgap[5] = 0;   spath[5] = 2;
    end

    reg  bclk = 0, lrck = 0, sdata = 0, dsd_on = 0;
    reg  run = 0;
    real half;
    integer mode;
    real rate;
    always begin
        if (run) begin
            #(half) bclk = ~bclk;
        end else begin
            #100;
        end
    end

    reg [31:0] dsdw [0:DSDN-1];
    initial $readmemh("build/dsd.hex", dsdw);

    reg [5:0]  slot = 0;
    reg [31:0] wl = 0, wr = 0;
    reg        wr_lsb = 0;
    reg        dop_m = 0;
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
        if (mode == 1) begin
            lrck  <= dsdw[(dsdi / 16) % DSDN][31 - (dsdi % 16)];
            sdata <= dsdw[(dsdi / 16) % DSDN][15 - (dsdi % 16)];
            dsdi = dsdi + 1;
        end else begin
            if (slot == 6'd0) begin
                if (mode == 0) begin
                    tt = nfr / rate;
                    wl = pcm32(0.891 * $sin(2.0 * 3.14159265358979 * 997.0 * tt));
                    wr = pcm32(0.891 * $sin(2.0 * 3.14159265358979 * 3001.0 * tt));
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

    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4)) dut (
        .mclk_in(mclk), .i2s_bclk_in(bclk), .i2s_lrck_in(lrck),
        .i2s_sdata_in(sdata), .dsd_on(dsd_on),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    // output peak monitor (pins)
    reg olr_q = 0;
    reg [31:0] osr = 0;
    real peak = 0.0, v;
    always @(posedge obclk) begin
        olr_q <= olrck;
        if (olrck != olr_q) begin
            v = $itor($signed({osr[30:0], osdata})) / 2147483648.0;
            if (v < 0) v = -v;
            if (v > peak) peak = v;
            osr <= 0;
        end else
            osr <= {osr[30:0], osdata};
    end

    integer locks = 0, errors = 0;
    integer got_path [0:5];
    always @(posedge audio_on) begin
        $display("%8.2f ms  audio_on  seg %0d path %0d (expect %0d)", $realtime / 1e6,
                 seg, dut.path, spath[seg]);
        got_path[seg] = dut.path;
        locks = locks + 1;
    end
    always @(negedge audio_on)
        $display("%8.2f ms  audio_off", $realtime / 1e6);

    integer i;
    initial begin
        for (i = 0; i < 6; i = i + 1) got_path[i] = -1;
        for (i = 0; i < 6; i = i + 1) begin
            seg = i;
            mode = smode[i];
            rate = srate[i];
            half = (mode == 1) ? 1.0e9 / rate / 2.0 : 1.0e9 / (rate * 64.0) / 2.0;
            dsd_on = (mode == 1);
            slot = 0; nfr = 0; dsdi = 0;
            run = 1;
            $display("%8.2f ms  segment %0d: mode %0d rate %0.1f", $realtime / 1e6, i, mode, rate);
            #(slen[i] * 1.0e6);
            if (sgap[i] > 0) begin
                run = 0;
                #(sgap[i] * 1.0e6);
            end
        end
        for (i = 0; i < 6; i = i + 1)
            if (got_path[i] != spath[i]) errors = errors + 1;
        $display("peak output %0.4f FS, locks %0d", peak, locks);
        if (peak > 0.90) errors = errors + 1;
        if (errors == 0) $display("PASS tb_switch");
        else             $display("FAIL tb_switch (%0d errors)", errors);
        $finish;
    end
endmodule
