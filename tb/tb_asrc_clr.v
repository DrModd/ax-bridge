`timescale 1ns/1ps
// ASRC: clr in the middle of a convolution must flush the MAC pipeline.
// No y_valid / clip may appear after clr for a result begun before it.
// (Reported against 068e77d; fix as in DrModd/ax-bridgeGPT 20e4e4d.)
module tb_asrc_clr;
    reg clk = 0;
    always #10.1725 clk = ~clk;
    reg rst_n = 0, clr = 0;
    reg [7:0] fpos = 0;
    always @(posedge clk) fpos <= fpos + 8'd1;
    wire yv, clp;
    wire signed [31:0] yl, yr;
    asrc u (.clk(clk), .rst_n(rst_n), .clr(clr), .fam48(1'b1), .arm(1'b1),
            .in_valid(fpos == 8'd0), .in_l(24'sd4000000), .in_r(-24'sd4000000),
            .ev(fpos == 8'd0), .fpos(fpos), .y_valid(yv), .y_l(yl), .y_r(yr), .clip(clp));
    integer errors = 0, k, aborts = 0, stale = 0;
    initial begin
        repeat (5) @(negedge clk);
        rst_n = 1;
        // several aborts at different points of the convolution
        for (k = 0; k < 6; k = k + 1) begin
            wait (u.issuing === 1'b1);
            repeat (3 + 13 * k) @(negedge clk);
            clr = 1; aborts = aborts + 1;
            @(negedge clk);
            if (u.issuing !== 1'b0 || yv !== 1'b0) begin
                $display("FAIL: ASRC MAC still running one clock into clr (abort %0d)", k);
                errors = errors + 1;
            end
            repeat (2) @(negedge clk);
            clr = 0;
            repeat (120) begin
                @(negedge clk);
                if (yv || clp) begin stale = stale + 1; end
            end
        end
        if (stale) begin
            $display("FAIL: %0d stale y_valid/clip pulses after clr", stale);
            errors = errors + stale;
        end
        if (errors == 0) $display("PASS tb_asrc_clr (%0d aborts mid-convolution)", aborts);
        else             $display("FAIL tb_asrc_clr (%0d errors)", errors);
        $finish;
    end
    initial begin #(20e6); $display("FAIL tb_asrc_clr: timeout (ASRC never issued)"); $finish; end
endmodule
