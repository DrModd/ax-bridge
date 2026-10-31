`timescale 1ns/1ps
// bclk_rx automatic DSD detection (auto = 1, no flag).
//  A: periodic 48 long (16 BCLK) + 16 short (2 BCLK) LRCK intervals: every
//     window of 64 edges holds exactly 16 short ones -> must become DSD
//     whatever the window alignment (reported against 068e77d: the 64th edge
//     of a window was dropped, the detector stayed in PCM).
//  B: I2S PCM with 32- and 16-bit slots -> never DSD.
//  C: DSD-like data on LRCK -> DSD; then PCM 16-bit slots -> back to PCM.
module tb_dsd_det;
    reg bclk = 0;
    always #177 bclk = ~bclk;              // DSD64 bit clock
    reg rst_n = 0, lrck = 0;
    wire we; wire [64:0] wd;
    bclk_rx dut (.bclk(bclk), .rst_n(rst_n), .lrck(lrck), .sdata(1'b0), .dsd_on(1'b0),
                 .auto(1'b1), .rs(1'b1), .wr_en(we), .wr_data(wd));
    integer errors = 0, i, k, ph;
    reg seen_dsd;
    always @(posedge bclk) if (dut.ad_dsd) seen_dsd = 1;

    task gap(input integer n);             // toggle LRCK after n bit clocks
        begin repeat (n) @(negedge bclk); lrck = ~lrck; end
    endtask

    integer seed = 7, r;
    initial begin
        repeat (4) @(negedge bclk);
        rst_n = 1;
        // A: all 64 alignments of the period against the detector window
        for (ph = 0; ph < 64; ph = ph + 1) begin
            rst_n = 0; repeat (3) @(negedge bclk); rst_n = 1; repeat (4) @(negedge bclk);
            seen_dsd = 0;
            for (i = 0; i < ph; i = i + 1) gap(i % 64 < 48 ? 16 : 2);   // shift the phase
            for (k = 0; k < 4; k = k + 1) begin
                for (i = 0; i < 48; i = i + 1) gap(16);
                for (i = 0; i < 16; i = i + 1) gap(2);
            end
            if (!seen_dsd) begin
                if (errors < 5) $display("FAIL A: 48 long + 16 short not detected as DSD (phase %0d)", ph);
                errors = errors + 1;
            end
        end
        // B: PCM, 32-bit and 16-bit slots
        rst_n = 0; repeat (3) @(negedge bclk); rst_n = 1; repeat (4) @(negedge bclk);
        seen_dsd = 0;
        for (i = 0; i < 3000; i = i + 1) gap(32);
        for (i = 0; i < 3000; i = i + 1) gap(16);
        if (seen_dsd) begin $display("FAIL B: PCM taken for DSD"); errors = errors + 1; end
        // C: DSD-like runs (1..6 bits), then back to PCM
        seen_dsd = 0;
        for (i = 0; i < 400; i = i + 1) begin
            seed = (seed * 1103515245 + 12345) & 32'h7fffffff; r = 1 + (seed >> 16) % 6;
            gap(r);
        end
        if (!seen_dsd) begin $display("FAIL C: DSD stream not detected"); errors = errors + 1; end
        for (i = 0; i < 129; i = i + 1) gap(16);
        repeat (4) @(negedge bclk);
        if (dut.ad_dsd) begin $display("FAIL C: did not return to PCM after 129 long intervals"); errors = errors + 1; end
        if (errors == 0) $display("PASS tb_dsd_det (64 window phases, PCM 16/32-bit, DSD -> PCM)");
        else             $display("FAIL tb_dsd_det (%0d errors)", errors);
        $finish;
    end
endmodule
