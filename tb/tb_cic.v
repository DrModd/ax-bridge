`timescale 1ns/1ps
// Unit bench: cic5_dsd against the Python golden model (tb/run_units.py).
module tb_cic;
    parameter RLOG2 = 3;
    parameter VEC = 600;
    parameter GAP = 20;

    reg clk = 0;
    always #10.172 clk = ~clk;
    reg rst_n = 0;
    reg in_valid = 0;
    reg [15:0] in_l = 0, in_r = 0;
    wire out_valid, ovf;
    wire signed [23:0] out_l, out_r;
    reg [2:0] rl = RLOG2;

    cic5_dsd dut (
        .clk(clk), .rst_n(rst_n), .clr(1'b0), .r_log2(rl),
        .in_valid(in_valid), .in_l(in_l), .in_r(in_r),
        .out_valid(out_valid), .out_l(out_l), .out_r(out_r), .ovf(ovf));

    reg [31:0] vin [0:VEC-1];
    integer fo, i, g;
    initial begin
        $readmemh("build/cic_in.hex", vin);
        fo = $fopen("build/cic_out.txt", "w");
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);
        for (i = 0; i < VEC; i = i + 1) begin
            @(posedge clk);
            in_valid <= 1'b1;
            in_l <= vin[i][31:16];
            in_r <= vin[i][15:0];
            @(posedge clk);
            in_valid <= 1'b0;
            if (i >= 4)
                for (g = 0; g < GAP; g = g + 1) @(posedge clk);
        end
        repeat (200) @(posedge clk);
        $fclose(fo);
        if (ovf) $display("FAIL tb_cic: queue overflow");
        else     $display("DONE tb_cic");
        $finish;
    end
    always @(posedge clk) if (out_valid) $fdisplay(fo, "%0d %0d", out_l, out_r);
endmodule
