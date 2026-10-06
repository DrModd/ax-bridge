`timescale 1ns/1ps
// Unit bench: fir_stage against the Python golden model (tb/ref_units.py).
// Feeds VEC pairs from build/fir_in.hex with GAP-cycle spacing (plus a
// burst of 4 at the start to exercise the queue), dumps every output pair.
module tb_fir;
    parameter N = 31;
    parameter INTERP = 0;
    parameter AW = 5;
    parameter CAW = 5;
    parameter FILE = "coef/hd_b.hex";
    parameter VEC = 400;
    parameter GAP = 200;

    reg clk = 0;
    always #10.172 clk = ~clk;
    reg rst_n = 0;
    reg in_valid = 0;
    reg signed [23:0] in_l = 0, in_r = 0;
    wire out_valid, ovf;
    wire signed [23:0] out_l, out_r;

    fir_stage #(.N(N), .INTERP(INTERP), .AW(AW), .CAW(CAW), .FILE(FILE)) dut (
        .clk(clk), .rst_n(rst_n), .clr(1'b0), .in_valid(in_valid),
        .in_l(in_l), .in_r(in_r), .out_valid(out_valid),
        .out_l(out_l), .out_r(out_r), .ovf(ovf));

    reg [47:0] vin [0:VEC-1];
    integer fo, i, g, nout;
    initial begin
        $readmemh("build/fir_in.hex", vin);
        fo = $fopen("build/fir_out.txt", "w");
        nout = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);
        for (i = 0; i < VEC; i = i + 1) begin
            @(posedge clk);
            in_valid <= 1'b1;
            in_l <= vin[i][47:24];
            in_r <= vin[i][23:0];
            @(posedge clk);
            in_valid <= 1'b0;
            if (i >= 4)
                for (g = 0; g < GAP; g = g + 1) @(posedge clk);
        end
        repeat (2000) @(posedge clk);
        $fclose(fo);
        if (ovf) $display("FAIL tb_fir: input queue overflow");
        else     $display("DONE tb_fir outputs=%0d", nout);
        $finish;
    end
    always @(posedge clk) if (out_valid) begin
        $fdisplay(fo, "%0d %0d", out_l, out_r);
        nout = nout + 1;
    end
endmodule
