`timescale 1ns/1fs
// Stress bench for spdif_rx alone (make spdif_stress).
// Four sources carry a frame counter (L = n, R = ~n), so every output frame
// can be checked bit-exactly: R must be ~L, and L must be the previous L
// (parity repeat) or previous + 1. Anything else that leaves the receiver is
// a corrupted sample = audible "digital distortion".
// Line select is switched many times at random moments under a ~40 us reset
// (as top.v does), and sources change rate on the fly (like a player
// changing tracks), with and without a gap.
module tb_spdif_cnt #(
    parameter integer SEED = 1,
    parameter real    T0   = 100.0
) (
    output reg line = 1'b0
);
    real    fs = 44100.0, jit = 0.0, duty = 0.0, half, j;
    integer seed, frame, gap = 0, bmode = 0, na = 0, bf = 0;
    reg [23:0] wl, wr;
    reg [23:0] n;
    reg [191:0] cs;

    task pulse(input integer k);
        begin
            j = (jit > 0.0) ? jit * (($random(seed) & 1023) / 1023.0) : 0.0;
            #(j);
            line = ~line;
            #(k * half - j + (line ? duty : -duty));
        end
    endtask
    task subframe(input integer pre, input [23:0] s, input c);
        integer i, ones; reg [27:0] d;
        begin
            pulse(3);
            if (pre == 1) begin pulse(1); pulse(1); pulse(3); end
            else if (pre == 2) begin pulse(3); pulse(1); pulse(1); end
            else begin pulse(2); pulse(1); pulse(2); end
            d = {1'b0, c, 2'b00, s};
            ones = 0;
            for (i = 0; i < 27; i = i + 1) ones = ones + d[i];
            d[27] = ones[0];
            for (i = 0; i < 28; i = i + 1)
                if (d[i]) begin pulse(1); pulse(1); end else pulse(2);
        end
    endtask
    initial begin
        seed = SEED; n = SEED * 24'h10101; frame = 0; cs = 192'd0; cs[2] = 1'b1;
        #(T0);
        forever begin
            if (gap > 0) begin line = 1'b0; #(gap); gap = 0; end
            half = 1.0e9 / (fs * 128.0);
            cs[1] = (na != 0);
            if (bmode) begin                      // IEC 61937: Pa/Pb every 1536 frames, data otherwise
                if (bf == 0) begin wl = 24'hF87200; wr = 24'h4E1F00; end
                else begin wl = $random(seed); wr = $random(seed); end
                bf = (bf == 1535) ? 0 : bf + 1;
            end else begin wl = n; wr = ~n; bf = 0; end
            subframe(frame == 0 ? 1 : 2, wl, cs[frame]);
            subframe(3, wr, cs[frame]);
            n = n + 24'd1;
            frame = (frame == 191) ? 0 : frame + 1;
        end
    end
endmodule

module tb_spdif_stress;
    reg clk = 0;
    always #10.0 clk = ~clk;              // 50 MHz (ax-bridge2 sys_clk); 49.152 MHz: CLKH=10.1725
    reg rst_n = 0;
    reg [1:0] sel = 0;
    wire [3:0] sp;
    tb_spdif_cnt #(.SEED(3), .T0(101.1)) s0 (.line(sp[0]));
    tb_spdif_cnt #(.SEED(5), .T0(202.2)) s1 (.line(sp[1]));
    tb_spdif_cnt #(.SEED(7), .T0(303.3)) s2 (.line(sp[2]));
    tb_spdif_cnt #(.SEED(9), .T0(404.4)) s3 (.line(sp[3]));

    wire valid, locked, nonaudio, err;
    wire [31:0] l, r;
    wire [7:0] cs0, cs3;
    spdif_rx dut (.clk(clk), .rst_n(rst_n), .line(sp), .sel(sel), .valid(valid), .l(l), .r(r),
                  .locked(locked), .nonaudio(nonaudio), .err(err), .cs0(cs0), .cs3(cs3));

    integer bad = 0, jumps = 0, frames = 0, errs = 0, nolock = 0, sw = 0, seg_bad;
    reg [23:0] pl; reg first;
    real t_lv = 0.0;
    always @(posedge clk) begin
        #1;
        if (err) errs = errs + 1;
        if (valid) begin
            if ($realtime - t_lv > 50000.0) first = 1;     // output resumed after a gap: new sequence
            t_lv = $realtime;
            frames = frames + 1;
            if (r[31:8] != ~l[31:8]) begin
                bad = bad + 1; seg_bad = seg_bad + 1;
                if (bad <= 20) $display("%10.3f us  CORRUPT sel %0d  L %06h R %06h (fine %0d T16 %0d)",
                                        $realtime / 1000.0, sel, l[31:8], r[31:8], dut.fine, dut.t16);
            end else if (!first && l[31:8] != pl && l[31:8] != pl + 24'd1) begin
                jumps = jumps + 1;
                if (jumps <= 20) $display("%10.3f us  jump sel %0d  %06h -> %06h", $realtime / 1000.0, sel, pl, l[31:8]);
            end
            first = 0; pl = l[31:8];
        end
    end

    real rates [0:6];
    integer i, k, sd, li;
    real tw, t_ev = 0.0, lat, lat_max = 0.0;
    reg was_locked = 0;
    always @(posedge clk) begin
        if (locked && !was_locked) begin
            lat = $realtime - t_ev;
            if (lat > lat_max) lat_max = lat;
        end
        was_locked = locked;
    end
    initial begin
        rates[0] = 32000.0; rates[1] = 44100.0; rates[2] = 48000.0; rates[3] = 88200.0;
        rates[4] = 96000.0; rates[5] = 176400.0; rates[6] = 192000.0;
        s0.fs = 192000.0; s0.jit = 4.0; s0.duty = 10.0;
        s1.fs = 44100.0;  s1.duty = -15.0;
        s2.fs = 96000.0;  s2.jit = 2.0;
        s3.fs = 48000.0;
        sd = 12345;
        first = 1; seg_bad = 0;
        #(5000.0); rst_n = 1;
        for (i = 0; i < `NSW; i = i + 1) begin
            // random wait, then either switch the line or change a source rate
            tw = 1.0e5 + (($random(sd) & 32767) / 32767.0) * 1.5e6;   // 0.1 .. 1.6 ms
            #(tw);
            if (!locked && $realtime - t_ev > 1.0e6) begin
                nolock = nolock + 1;
                $display("%10.3f us  NOT LOCKED sel %0d (fs %0.0f) fine %0d ref_ok %0d", $realtime / 1000.0, sel,
                         sel == 0 ? s0.fs : sel == 1 ? s1.fs : sel == 2 ? s2.fs : s3.fs, dut.fine, dut.ref_ok);
            end
            k = $random(sd) & 7;
            if (k < 5) begin                       // switch the line under reset (top.v: ~40 us)
                li = ($random(sd) & 3);
                rst_n = 0; #(20000.0); sel = li; #(20000.0); rst_n = 1;
                first = 1; sw = sw + 1; t_ev = $realtime;
                was_locked = 0;
                $display("%10.3f us  switch to line %0d (fs %0.0f)", $realtime / 1000.0, li,
                         li == 0 ? s0.fs : li == 1 ? s1.fs : li == 2 ? s2.fs : s3.fs);
            end else begin                         // the selected source changes rate on the fly
                li = $random(sd) % 7; if (li < 0) li = -li;
                if (k == 7) begin                  // with a gap (line idle 20 us)
                    case (sel) 0: s0.gap = 20000; 1: s1.gap = 20000; 2: s2.gap = 20000; default: s3.gap = 20000; endcase
                end
                case (sel)
                    0: s0.fs = rates[li]; 1: s1.fs = rates[li]; 2: s2.fs = rates[li]; default: s3.fs = rates[li];
                endcase
                first = 1; t_ev = $realtime; was_locked = 0;
                $display("%10.3f us  rate change on line %0d -> %0.0f%s", $realtime / 1000.0, sel, rates[li],
                         k == 7 ? " (gap)" : "");
            end
        end
        #(1.5e6);
        // ---- compressed audio ----
        // 1) the selected source turns into a Dolby/DTS burst stream without
        //    touching the channel status: must go silent from the first Pa/Pb
        k = frames;
        $display("%10.3f us  line %0d: IEC 61937 bursts, channel status still PCM", $realtime / 1000.0, sel);
        case (sel) 0: s0.bmode = 1; 1: s1.bmode = 1; 2: s2.bmode = 1; default: s3.bmode = 1; endcase
        #(40.0e6);
        $display("%10.3f us  frames during bursts: %0d (the 1st frame before the burst start may pass)", $realtime / 1000.0, frames - k);
        case (sel) 0: s0.bmode = 0; 1: s1.bmode = 0; 2: s2.bmode = 0; default: s3.bmode = 0; endcase
        k = frames;
        #(200.0e6);
        $display("%10.3f us  PCM again: %0d frames", $realtime / 1000.0, frames - k);
        if (frames - k < 100) begin $display("ERROR: no PCM after the bursts"); nolock = nolock + 1; end
        // 2) switch to a line that is non-audio from the start: no frame at all
        s1.bmode = 1; s1.na = 1;
        rst_n = 0; #(20000.0); sel = 1; #(20000.0); rst_n = 1;
        k = frames;
        #(30.0e6);
        $display("%10.3f us  non-audio line from the start: %0d frames", $realtime / 1000.0, frames - k);
        if (frames != k) begin $display("ERROR: non-audio line produced frames"); bad = bad + 1; end
        $display("max lock latency %0.0f us", lat_max / 1000.0);
        $display("switches %0d  frames %0d  corrupt %0d  jumps %0d  parity/biphase errors %0d  not-locked %0d",
                 sw, frames, bad, jumps, errs, nolock);
        if (bad == 0 && nolock == 0) $display("PASS tb_spdif_stress"); else $display("FAIL tb_spdif_stress");
        $finish;
    end
endmodule
