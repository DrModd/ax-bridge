`timescale 1ns / 1ps
// Audio bench for jitter_fifo: a sine through the buffer with a clock
// offset, so the read side spends long stretches interpolating.
// Write side: wr_clk 49.152 MHz, strobe at Fs * (1 + PPM) (fractional
// accumulator, strobe jitter one wr_clk period, as from a real source).
// Read side: rd_clk 49.152 MHz (own oscillator), strobe every 256 clocks
// (192 kHz grid) or DIV clocks.
// Every output sample is dumped with the flags to build/jf_audio.txt;
// tb/run_jf.py measures THD+N in the bit-exact and interpolated stretches.
module tb_jf_audio;
    parameter real    FS    = 48000.0;
    parameter real    PPM   = 500.0;       // writer faster (+) / slower (-)
    parameter real    F0    = 997.0;
    parameter real    AMP   = 0.5;
    parameter integer AW    = 10;
    parameter integer NT    = 64;
    parameter integer MPH_SH = 7;
    parameter          COEF_FILE = "jitter_fifo/rtl/fir_ls64_m128.hex";
    parameter integer N_OUT = 60000;
    parameter integer STEP_SH = 10;

    reg wr_clk = 0, rd_clk = 0;
    always #10.1725 wr_clk = ~wr_clk;
    always #10.1726 rd_clk = ~rd_clk;          // independent oscillator
    reg wr_rst = 1, rd_rst = 1;

    // write strobe: phase accumulator at Fs * (1 + PPM) on wr_clk
    real    acc = 0.0, inc;
    reg     wr_stb = 0;
    reg [31:0] in_l = 0, in_r = 0;
    integer n_in = 0;
    real    ph;
    initial inc = FS * (1.0 + PPM * 1e-6) / 49.152e6;
    always @(posedge wr_clk) begin
        if (!wr_rst) begin
            acc = acc + inc;
            if (acc >= 1.0) begin
                acc = acc - 1.0;
                ph = 2.0 * 3.14159265358979 * F0 * n_in / FS;
                in_l <= $rtoi(AMP * $sin(ph) * 2147483647.0);
                in_r <= $rtoi(AMP * $cos(ph) * 2147483647.0);
                n_in = n_in + 1;
                wr_stb <= 1'b1;
            end else if (acc >= 0.5)
                wr_stb <= 1'b0;
        end
    end

    // read strobe: exact Fs on rd_clk (fractional divider)
    real    racc = 0.0, rinc;
    reg     rd_stb = 0;
    initial rinc = FS / 49.152e6;
    always @(posedge rd_clk) begin
        if (!rd_rst) begin
            racc = racc + rinc;
            if (racc >= 1.0) begin racc = racc - 1.0; rd_stb <= 1'b1; end
            else if (racc >= 0.5) rd_stb <= 1'b0;
        end
    end

    wire [31:0] out_l, out_r;
    wire out_valid, flag_low, flag_high, xrun, rd_late, running, pause, wr_full, wr_in_pause;
    wire [AW:0] fill;
    jitter_fifo #(.AW(AW), .NT(NT), .MPH_SH(MPH_SH), .STEP_SH(STEP_SH), .COEF_FILE(COEF_FILE)) dut (
        .wr_clk(wr_clk), .wr_rst(wr_rst), .wr_stb(wr_stb), .in_l(in_l), .in_r(in_r),
        .pause_thr(32'd0), .pause_len(24'd0), .wr_full(wr_full), .wr_in_pause(wr_in_pause),
        .rd_clk(rd_clk), .rd_rst(rd_rst), .rd_stb(rd_stb),
        .out_l(out_l), .out_r(out_r), .out_valid(out_valid),
        .flag_low(flag_low), .flag_high(flag_high), .xrun(xrun), .rd_late(rd_late),
        .running(running), .pause(pause), .fill(fill));

    integer fo, n = 0, nx = 0, nl = 0;
    initial begin
        fo = $fopen("build/jf_audio.txt", "w");
        #200; wr_rst = 0; rd_rst = 0;
    end
    always @(posedge rd_clk) begin
        if (xrun) nx = nx + 1;
        if (rd_late) nl = nl + 1;
        if (out_valid && running) begin
            $fdisplay(fo, "%0d %0d %0d %0d %0d", $signed(out_l), $signed(out_r),
                      flag_low, flag_high, fill);
            n = n + 1;
            if (n == N_OUT) begin
                $display("DONE outputs %0d xrun %0d late %0d", n, nx, nl);
                $fclose(fo);
                $finish;
            end
        end
    end
endmodule
