`timescale 1ns/1ps
// Unit bench: eq_biquad against the bit-exact model (tb/run_eq.py).
// Frames every 256 MCLK. Bank A (NA sections) is written and applied
// before frame 0, bank B (NB sections) is written during frames and
// applied before frame SWAP, so the swap happens on live filter states.
module tb_eq;
    parameter VEC = 2000;
    parameter SWAP = 1000;
    parameter NA = 10;
    parameter NB = 4;

    reg clk = 0;
    always #10.172 clk = ~clk;
    reg rst_n = 0;
    reg start = 0;
    reg signed [31:0] in_l = 0, in_r = 0;
    wire signed [31:0] out_l, out_r;
    wire done, clip, apply_pend, bank;
    wire [3:0] nsec;
    reg cw_we = 0;
    reg [6:0] cw_idx = 0;
    reg [31:0] cw_data = 0;
    reg apply_req = 0;
    reg [3:0] nsec_new = 0;

    eq_biquad dut (
        .clk(clk), .rst_n(rst_n), .clr(1'b0), .start(start), .in_l(in_l), .in_r(in_r),
        .out_l(out_l), .out_r(out_r), .done(done), .clip(clip),
        .cw_we(cw_we), .cw_idx(cw_idx), .cw_data(cw_data),
        .apply_req(apply_req), .nsec_new(nsec_new),
        .apply_pend(apply_pend), .bank(bank), .nsec(nsec));

    reg [63:0] vin [0:VEC-1];
    reg [31:0] wa [0:99];
    reg [31:0] wb [0:99];
    integer fo, i, j, nclip, maxbusy, t0;

    task load(input which);
        integer k;
        begin
            for (k = 0; k < 100; k = k + 1) begin
                @(posedge clk);
                cw_we <= 1'b1; cw_idx <= k;
                cw_data <= which ? wb[k] : wa[k];
            end
            @(posedge clk); cw_we <= 1'b0;
        end
    endtask

    initial begin
        $readmemh("build/eq_in.hex", vin);
        $readmemh("build/eq_wa.hex", wa);
        $readmemh("build/eq_wb.hex", wb);
        fo = $fopen("build/eq_out.txt", "w");
        nclip = 0; maxbusy = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        load(0);
        @(posedge clk); apply_req <= 1'b1; nsec_new <= NA;
        @(posedge clk); apply_req <= 1'b0;
        for (i = 0; i < VEC; i = i + 1) begin
            @(posedge clk);
            start <= 1'b1; in_l <= vin[i][63:32]; in_r <= vin[i][31:0];
            t0 = 0;
            @(posedge clk); start <= 1'b0;
            if (i == SWAP - 3) begin            // write bank B while running
                for (j = 0; j < 100; j = j + 1) begin
                    @(posedge clk);
                    cw_we <= 1'b1; cw_idx <= j; cw_data <= wb[j];
                    t0 = t0 + 1;
                end
                @(posedge clk); cw_we <= 1'b0; t0 = t0 + 1;
            end
            if (i == SWAP - 1) begin
                @(posedge clk); apply_req <= 1'b1; nsec_new <= NB;
                @(posedge clk); apply_req <= 1'b0; t0 = t0 + 2;
            end
            while (t0 < 254) begin @(posedge clk); t0 = t0 + 1; end
        end
        repeat (300) @(posedge clk);
        $fclose(fo);
        $display("DONE tb_eq clips=%0d maxbusy=%0d bank=%0d nsec=%0d", nclip, maxbusy, bank, nsec);
        $finish;
    end
    integer cyc = 0;
    always @(posedge clk) begin
        if (start) cyc = 0; else cyc = cyc + 1;
        if (done) begin
            $fdisplay(fo, "%0d %0d", out_l, out_r);
            if (cyc > maxbusy) maxbusy = cyc;
        end
        if (clip) nclip = nclip + 1;
    end
endmodule
