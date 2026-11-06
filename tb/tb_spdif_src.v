`timescale 1ns/1fs
// Behavioural S/PDIF transmitter for the benches: biphase-mark stream at
// FS (+PPM) with JIT ns of random edge jitter, 24-bit sines FL / FR at
// AMP, channel status byte 0 bit 1 = NONAUDIO, even parity. EN = 0 keeps
// the line low. DUTY ns stretches every high pulse and shortens every
// low one (pulse-width distortion of optical receivers).
module tb_spdif_src #(
    parameter real    FS       = 44100.0,
    parameter real    PPM      = 0.0,
    parameter real    FL       = 997.0,
    parameter real    FR       = 3001.0,
    parameter real    AMP      = 0.891,
    parameter real    JIT      = 0.0,
    parameter real    DUTY     = 0.0,
    parameter integer NONAUDIO = 0,
    parameter integer SEED     = 1,
    parameter real    T0       = 777.7
) (
    input  wire en,
    output reg  line = 1'b0
);
    real half, t, ph_l, ph_r, j;
    integer frame, k, seed;
    reg [191:0] cs;
    reg [27:0]  d;
    reg [23:0]  sl, sr_;

    task pulse(input integer n);       // toggle, then n half-cells
        begin
            j = (JIT > 0.0) ? JIT * (($random(seed) & 1023) / 1023.0) : 0.0;
            #(j);
            if (en) line = ~line; else line = 1'b0;
            #(n * half - j + (line ? DUTY : -DUTY));
        end
    endtask

    task subframe(input integer pre, input [23:0] s, input c);
        integer i, ones;
        begin
            // preamble: 1 = B (3-1-1-3), 2 = M (3-3-1-1), 3 = W (3-2-1-2)
            pulse(3);
            if (pre == 1) begin pulse(1); pulse(1); pulse(3); end
            else if (pre == 2) begin pulse(3); pulse(1); pulse(1); end
            else begin pulse(2); pulse(1); pulse(2); end
            d = {1'b0, c, 1'b0, 1'b0, s};      // P U.. placeholder: [27]=P [26]=C [25]=U [24]=V
            d[24] = 1'b0; d[25] = 1'b0; d[26] = c;
            ones = 0;
            for (i = 0; i < 27; i = i + 1) ones = ones + d[i];
            d[27] = ones[0];                    // even parity over bits 4..31
            for (i = 0; i < 28; i = i + 1) begin
                if (d[i]) begin pulse(1); pulse(1); end
                else pulse(2);
            end
        end
    endtask

    initial begin
        seed = SEED;
        half = 1.0e9 / (FS * (1.0 + PPM * 1.0e-6) * 128.0);
        cs = 192'd0;
        cs[1] = (NONAUDIO != 0);
        cs[2] = 1'b1;                           // copy permitted
        cs[27:24] = (FS == 48000.0) ? 4'b0100 : (FS == 96000.0) ? 4'b0101 :
                    (FS == 192000.0) ? 4'b0111 : 4'b0000;
        frame = 0;
        t = 0.0;
        #(T0);
        forever begin
            ph_l = 2.0 * 3.14159265358979 * FL * t;
            ph_r = 2.0 * 3.14159265358979 * FR * t;
            sl  = $rtoi($sin(ph_l) * AMP * 8388607.0);
            sr_ = $rtoi($sin(ph_r) * AMP * 8388607.0);
            subframe(frame == 0 ? 1 : 2, sl, cs[frame]);
            subframe(3, sr_, cs[frame]);
            frame = (frame == 191) ? 0 : frame + 1;
            t = t + 1.0 / FS;
        end
    end
endmodule
