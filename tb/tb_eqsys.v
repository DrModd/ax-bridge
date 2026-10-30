`timescale 1ns/1fs
// EQ through the whole bridge (tb/run_sys.py --eq). PCM 96 kHz on input 0.
// Bank 1 (build/eqsys_b1.hex: byte stream for EQ_DATA) is loaded over I2C
// while the bridge locks, applied, and captured (seg 0). Then a bypass
// (nsec = 0) is applied with audio running: no dropout, seg 1 must be
// the unprocessed level.
module tb_eqsys;
    parameter integer NB1 = 400;     // bytes in bank 1
    parameter integer NS1 = 2;       // sections in bank 1
    reg mclk = 0;
    always #10.1725 mclk = ~mclk;

    wire b0, l0, d0, dsd0;
    tb_src #(.MODE(0), .FS(96000.0), .PPM(-31.0), .FL(997.0), .FR(7003.0),
             .T0(555.5)) s0 (.en(1'b1), .bclk(b0), .lrck(l0), .sdata(d0), .dsd_on(dsd0));

    reg  scl_m = 1'b1, sda_m = 1'b1;
    tri1 i2c_sda;
    assign i2c_sda = sda_m ? 1'bz : 1'b0;

    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4)) dut (
        .mclk_in(mclk),
        .i2s_bclk_in(b0), .i2s_lrck_in(l0), .i2s_sdata_in(d0), .dsd_on(dsd0),
        .in1_bclk(1'b0), .in1_lrck(1'b0), .in1_sdata(1'b0),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0),
        .i2c_scl(scl_m), .i2c_sda(i2c_sda),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    // ---------------- I2C master, 400 kHz ----------------
    localparam real TQ = 625.0;   // quarter bit (ns)
    integer errors = 0;
    task i2c_start; begin
        sda_m = 1; scl_m = 1; #(TQ); sda_m = 0; #(TQ); scl_m = 0; #(TQ);
    end endtask
    task i2c_stop; begin
        sda_m = 0; #(TQ); scl_m = 1; #(TQ); sda_m = 1; #(2 * TQ);
    end endtask
    task i2c_bit_w(input b); begin
        sda_m = b; #(TQ); scl_m = 1; #(2 * TQ); scl_m = 0; #(TQ);
    end endtask
    task i2c_bit_r(output b); begin
        sda_m = 1; #(TQ); scl_m = 1; #(TQ); b = i2c_sda; #(TQ); scl_m = 0; #(TQ);
    end endtask
    task i2c_byte_w(input [7:0] v, output ack);
        integer i; reg a;
        begin
            for (i = 7; i >= 0; i = i - 1) i2c_bit_w(v[i]);
            i2c_bit_r(a);
            ack = ~a;
        end
    endtask
    task i2c_byte_r(output [7:0] v, input ack);
        integer i; reg b;
        begin
            for (i = 7; i >= 0; i = i - 1) begin i2c_bit_r(b); v[i] = b; end
            i2c_bit_w(~ack);
        end
    endtask
    task wr_reg(input [7:0] r, input [7:0] v);
        reg a0, a1, a2;
        begin
            i2c_start; i2c_byte_w({7'h5A, 1'b0}, a0); i2c_byte_w(r, a1); i2c_byte_w(v, a2); i2c_stop;
            if (!(a0 && a1 && a2)) begin
                $display("ERROR: no ACK writing reg %0h", r); errors = errors + 1;
            end
        end
    endtask
    task rd_regs(input [7:0] r, output [7:0] v0, output [7:0] v1, output [7:0] v2, output [7:0] v3);
        reg a0, a1, a2;
        begin
            i2c_start; i2c_byte_w({7'h5A, 1'b0}, a0); i2c_byte_w(r, a1);
            i2c_start; i2c_byte_w({7'h5A, 1'b1}, a2);
            i2c_byte_r(v0, 1); i2c_byte_r(v1, 1); i2c_byte_r(v2, 1); i2c_byte_r(v3, 0);
            i2c_stop;
            if (!(a0 && a1 && a2)) begin
                $display("ERROR: no ACK reading reg %0h", r); errors = errors + 1;
            end
        end
    endtask

    reg [7:0] blk [0:399];
    task wr_block(input integer n);      // EQ_IDX = 0, then n bytes into EQ_DATA
        integer k; reg a0, a1, a;
        begin
            wr_reg(8'h0B, 8'h00);
            i2c_start; i2c_byte_w({7'h5A, 1'b0}, a0); i2c_byte_w(8'h0C, a1);
            if (!(a0 && a1)) begin $display("ERROR: no ACK on EQ_DATA"); errors = errors + 1; end
            for (k = 0; k < n; k = k + 1) begin
                i2c_byte_w(blk[k], a);
                if (!a) begin $display("ERROR: NACK at byte %0d", k); errors = errors + 1; end
            end
            i2c_stop;
        end
    endtask

    // ---------------- output capture ----------------
    reg        olr_q = 0;
    reg [31:0] osr = 0, ow_l = 0;
    integer    fo = 0, ncap = 0;
    reg        cap = 0;
    real       peak = 0.0, v;
    always @(posedge obclk) begin
        olr_q <= olrck;
        if (olrck != olr_q) begin
            v = $itor($signed({osr[30:0], osdata})) / 2147483648.0;
            if (v < 0) v = -v;
            if (v > peak) peak = v;
            if (olr_q == 1'b0) ow_l <= {osr[30:0], osdata};
            else if (cap && audio_on) begin
                $fdisplay(fo, "%0d %0d", $signed(ow_l), $signed({osr[30:0], osdata}));
                ncap = ncap + 1;
            end
            osr <= 0;
        end else
            osr <= {osr[30:0], osdata};
    end

    task capture(input integer seg, input integer n);
        reg [8*24-1:0] name;
        begin
            $sformat(name, "build/eqsys_seg%0d.txt", seg);
            fo = $fopen(name, "w");
            ncap = 0; cap = 1;
            wait (ncap >= n);
            cap = 0;
            $fclose(fo);
        end
    endtask

    task wait_on(input real ms_max);
        real t0;
        begin
            t0 = $realtime;
            while (!audio_on && ($realtime - t0) < ms_max * 1e6) #(10000);
            if (!audio_on) begin
                $display("ERROR: no audio within %0.1f ms", ms_max); errors = errors + 1;
            end else
                $display("%8.2f ms  audio_on (input %0d)", $realtime / 1e6, dut.sel_cur);
        end
    endtask

    task check(input [7:0] got, input [7:0] exp, input [8*12-1:0] what);
        begin
            if (got !== exp) begin
                $display("ERROR: %0s = %02h, expected %02h", what, got, exp);
                errors = errors + 1;
            end else
                $display("          %0s = %02h ok", what, got);
        end
    endtask

    reg [7:0] r0, r1, r2, r3;
    integer drops = 0;
    always @(negedge audio_on) if ($realtime > 1.0) drops = drops + 1;
    initial begin
        $readmemh("build/eqsys_b1.hex", blk);
        #(2e6);
        rd_regs(8'h05, r0, r1, r2, r3);
        check(r0, 8'hA5, "ID"); check(r1, 8'h04, "VERSION");
        rd_regs(8'h09, r0, r1, r2, r3);
        check(r0, 8'h00, "EQ_CTRL");
        wr_block(NB1);
        rd_regs(8'h0B, r0, r1, r2, r3);
        check(r0, NB1 / 4, "EQ_IDX");
        wr_reg(8'h0A, NS1);
        wr_reg(8'h09, 8'h01);
        #(20000);
        rd_regs(8'h09, r0, r1, r2, r3);
        check(r0, {NS1[3:0], 4'b0010}, "EQ_CTRL");
        check(r1, NS1, "EQ_NSEC");
        wait_on(80.0);
        #(40e6);                      // fade-in done, 20 Hz HP start tail decayed
        drops = 0;
        capture(0, 8192);
        // bypass, applied live
        wr_reg(8'h0A, 8'h00);
        wr_reg(8'h09, 8'h01);
        #(20000);
        rd_regs(8'h09, r0, r1, r2, r3);
        check(r0, 8'h00, "EQ_CTRL");
        #(2e6);
        capture(1, 8192);
        if (drops) begin $display("ERROR: %0d audio dropout(s)", drops); errors = errors + 1; end
        rd_regs(8'h08, r0, r1, r2, r3);
        check(r0 & 8'h80, 8'h00, "CLIP");
        if (errors == 0) $display("PASS tb_eqsys");
        else             $display("FAIL tb_eqsys (%0d errors)", errors);
        $finish;
    end
    initial begin
        #(400e6);
        $display("FAIL tb_eqsys: timeout");
        $finish;
    end
endmodule
