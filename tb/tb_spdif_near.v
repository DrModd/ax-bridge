`timescale 1ns/1fs
// Small sample-rate steps on the fly (44.1 <-> 48, 88.2 <-> 96, 176.4 <-> 192),
// the case where a stale timing grid still half-decodes the new stream:
// subframes with an even number of wrong bits pass the parity check.
// Every output frame is checked against the counter (R = ~L).
`ifndef DUTY
`define DUTY 0.0
`endif
module tb_spdif_near;
    reg clk = 0;
`ifdef MCLK49
    always #10.1725 clk = ~clk;
`else
    always #10.0 clk = ~clk;
`endif
    reg rst_n = 0;
    wire line;
    tb_spdif_cnt #(.SEED(11), .T0(77.7)) s (.line(line));
    wire valid, locked, nonaudio, err;
    wire [31:0] l, r;
    wire [7:0] cs0, cs3;
    spdif_rx dut (.clk(clk), .rst_n(rst_n), .line({3'b000, line}), .sel(2'd0), .valid(valid), .l(l), .r(r),
                  .locked(locked), .nonaudio(nonaudio), .err(err), .cs0(cs0), .cs3(cs3));
    integer bad = 0, frames = 0, errs = 0, i, p;
    always @(posedge clk) begin
        #1;
        if (err) errs = errs + 1;
        if (valid) begin
            frames = frames + 1;
            if (r[31:8] != ~l[31:8]) begin
                bad = bad + 1;
                if (bad <= 10) $display("%10.3f us  CORRUPT fs %0.0f  L %06h R %06h", $realtime / 1000.0, s.fs, l[31:8], r[31:8]);
            end
        end
    end
    real pa [0:2], pb [0:2];
    initial begin
        pa[0] = 44100.0;  pb[0] = 48000.0;
        pa[1] = 88200.0;  pb[1] = 96000.0;
        pa[2] = 176400.0; pb[2] = 192000.0;
        s.duty = `DUTY; s.jit = 4.0;
        s.fs = pa[0];
        #(5000.0); rst_n = 1;
        for (p = 0; p < 3; p = p + 1)
            for (i = 0; i < 8; i = i + 1) begin
                s.fs = (i & 1) ? pa[p] : pb[p];
                #(9.0e6);
                if (!locked) $display("%10.3f us  not locked at %0.0f", $realtime / 1000.0, s.fs);
            end
        $display("DUTY %0.1f ns: frames %0d  corrupt %0d  errors %0d", `DUTY, frames, bad, errs);
        if (bad == 0) $display("PASS tb_spdif_near"); else $display("FAIL tb_spdif_near");
        $finish;
    end
endmodule
