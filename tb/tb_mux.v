`timescale 1ns/1fs
// Input switch + I2C bench. Four transports run at the same time on
// unrelated clocks:  in0 DSD64 (Fox, native), in1 PCM 44.1, in2 PCM 96,
// in3 silent. An I2C master (400 kHz) selects inputs and reads the
// status registers. Each selected input is captured to
// build/mux_seg<N>.txt for frequency / level / THD+N analysis
// (tb/run_sys.py --mux).
module tb_mux;
    parameter integer DSDN = 1;
    reg mclk = 0;
    always #10.1725 mclk = ~mclk;

    wire b0, l0, d0, dsd0, b1, l1, d1, x1, b2, l2, d2, x2, b3, l3, d3, x3;
    tb_src #(.MODE(1), .FS(2822400.0), .PPM(31.0), .FL(997.0), .FR(3001.0),
             .DSDN(DSDN), .T0(1234.5)) s0 (.en(1'b1), .bclk(b0), .lrck(l0), .sdata(d0), .dsd_on(dsd0));
    tb_src #(.MODE(0), .FS(44100.0), .PPM(60.0), .FL(1499.0), .FR(5003.0),
             .T0(777.7)) s1 (.en(1'b1), .bclk(b1), .lrck(l1), .sdata(d1), .dsd_on(x1));
    tb_src #(.MODE(0), .FS(96000.0), .PPM(-45.0), .FL(2003.0), .FR(7001.0),
             .T0(333.3)) s2 (.en(1'b1), .bclk(b2), .lrck(l2), .sdata(d2), .dsd_on(x2));
    tb_src #(.MODE(0), .FS(48000.0), .PPM(0.0)) s3 (.en(1'b0), .bclk(b3), .lrck(l3), .sdata(d3), .dsd_on(x3));

    reg  scl_m = 1'b1, sda_m = 1'b1;
    tri1 i2c_sda;
    assign i2c_sda = sda_m ? 1'bz : 1'b0;

    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4)) dut (
        .mclk_in(mclk),
        .i2s_bclk_in(b0), .i2s_lrck_in(l0), .i2s_sdata_in(d0), .dsd_on(dsd0),
        .in1_bclk(b1), .in1_lrck(l1), .in1_sdata(d1),
        .in2_bclk(b2), .in2_lrck(l2), .in2_sdata(d2),
        .in3_bclk(b3), .in3_lrck(l3), .in3_sdata(d3),
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
            $sformat(name, "build/mux_seg%0d.txt", seg);
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

    reg [7:0] r0, r1, r2, r3, q0, q1, q2, q3;
    initial begin
        #(5e6);
        rd_regs(8'h05, r0, r1, r2, r3);
        check(r0, 8'hA5, "ID"); check(r1, 8'h02, "VERSION");
        // input 0 (default): DSD64
        wait_on(60.0);
        rd_regs(8'h00, r0, r1, r2, r3);
        check(r0, 8'h00, "CTRL"); check(r1, 8'h73, "STATUS"); check(r3, 8'h11, "RATE");
        capture(0, 4096);
        // input 1: PCM 44.1
        wr_reg(8'h00, 8'h01);
        #(20000);
        if (audio_on) begin $display("ERROR: audio still on after switch"); errors = errors + 1; end
        wait_on(60.0);
        rd_regs(8'h01, r1, r2, r3, q0);
        check(r1, 8'h77, "STATUS"); check(r2, 8'h80, "FORMAT"); check(r3, 8'h01, "RATE");
        capture(1, 4096);
        // input 2: PCM 96
        wr_reg(8'h00, 8'h02);
        wait_on(60.0);
        rd_regs(8'h01, r1, r2, r3, q0);
        check(r1, 8'h7B, "STATUS"); check(r3, 8'h04, "RATE");
        capture(2, 4096);
        // input 3: nothing connected
        wr_reg(8'h00, 8'h03);
        #(30e6);
        rd_regs(8'h01, r1, r2, r3, q0);
        check(r1, 8'h7C, "STATUS"); check(r3, 8'h00, "RATE");
        if (audio_on) begin $display("ERROR: audio on with a dead input"); errors = errors + 1; end
        // mute + input 1: stays silent; unmute: plays
        wr_reg(8'h00, 8'h81);
        #(40e6);
        if (audio_on) begin $display("ERROR: audio on while muted"); errors = errors + 1; end
        rd_regs(8'h00, r0, r1, r2, r3);
        check(r0, 8'h81, "CTRL");
        wr_reg(8'h00, 8'h01);
        wait_on(60.0);
        capture(3, 2048);
        $display("peak output %0.4f FS", peak);
        if (peak > 0.90) errors = errors + 1;
        if (errors == 0) $display("PASS tb_mux");
        else             $display("FAIL tb_mux (%0d errors)", errors);
        $finish;
    end
    initial begin
        #(800e6);
        $display("FAIL tb_mux: timeout");
        $finish;
    end
endmodule
