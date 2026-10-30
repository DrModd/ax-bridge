`timescale 1ns/1fs
// Recovery / switch-ordering bench.
//  1. Overflow recovery: while PCM 96k plays, a stage overflow flag is
//     forced once. The bridge must drop audio and come back on its own
//     (a flag that survives the re-acquisition would block audio forever).
//  2. Switch ordering: every change of the input mux (sel_cur) must happen
//     while the front end is already in reset (rst_in_n low, bclk_rx
//     reset synchroniser cleared) and after the hold has been asserted for
//     at least 64 MCLK. Checked by a monitor on every sel_cur change.
//  3. Back-to-back requests: a second select request arriving inside the
//     hold window must end on the last requested input.
module tb_recover;
    reg mclk = 0;
    always #10.1725 mclk = ~mclk;

    wire b0, l0, d0, x0, b1, l1, d1, x1, b2, l2, d2, x2;
    tb_src #(.MODE(0), .FS(96000.0), .PPM(-45.0), .FL(2003.0), .FR(7001.0),
             .T0(333.3)) s0 (.en(1'b1), .bclk(b0), .lrck(l0), .sdata(d0), .dsd_on(x0));
    tb_src #(.MODE(0), .FS(44100.0), .PPM(60.0), .FL(1499.0), .FR(5003.0),
             .T0(777.7)) s1 (.en(1'b1), .bclk(b1), .lrck(l1), .sdata(d1), .dsd_on(x1));
    tb_src #(.MODE(0), .FS(192000.0), .PPM(13.0), .FL(997.0), .FR(3001.0),
             .T0(111.1)) s2 (.en(1'b1), .bclk(b2), .lrck(l2), .sdata(d2), .dsd_on(x2));

    tri1 i2c_sda;
    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4)) dut (
        .mclk_in(mclk),
        .i2s_bclk_in(b0), .i2s_lrck_in(l0), .i2s_sdata_in(d0), .dsd_on(1'b0),
        .in1_bclk(b1), .in1_lrck(l1), .in1_sdata(d1),
        .in2_bclk(b2), .in2_lrck(l2), .in2_sdata(d2),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0),
        .i2c_scl(1'b1), .i2c_sda(i2c_sda),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    integer errors = 0;

    // ---- monitor: the mux may only move under an established reset ----
    integer hold_len = 0;
    always @(posedge mclk) begin
        if (dut.hold) hold_len = hold_len + 1;
        else          hold_len = 0;
    end
    reg [1:0] sel_q = 2'd0;
    integer n_sw = 0;
    always @(posedge mclk) begin
        if (dut.rst_n === 1'b1 && dut.sel_cur !== sel_q) begin
            n_sw = n_sw + 1;
            if (dut.rst_in_n !== 1'b0 || dut.rst_b !== 2'b00 || hold_len < 64) begin
                $display("ERROR %0.3f ms: mux %0d -> %0d with rst_in_n=%b rst_b=%b, hold for %0d MCLK",
                         $realtime / 1e6, sel_q, dut.sel_cur, dut.rst_in_n, dut.rst_b, hold_len);
                errors = errors + 1;
            end else
                $display("%8.3f ms  mux %0d -> %0d under reset (hold %0d MCLK) ok",
                         $realtime / 1e6, sel_q, dut.sel_cur, hold_len);
            sel_q = dut.sel_cur;
        end
    end

    task wait_on(input real ms_max, input integer exp_in, input integer exp_path);
        real t0;
        begin
            t0 = $realtime;
            while (audio_on !== 1'b1 && ($realtime - t0) < ms_max * 1e6) #(10000);
            if (audio_on !== 1'b1) begin
                $display("ERROR: no audio within %0.1f ms (input %0d, state %0d)",
                         ms_max, dut.sel_cur, dut.cst);
                errors = errors + 1;
            end else if (dut.sel_cur != exp_in || dut.path != exp_path) begin
                $display("ERROR: audio on input %0d path %0d, expected input %0d path %0d",
                         dut.sel_cur, dut.path, exp_in, exp_path);
                errors = errors + 1;
            end else
                $display("%8.3f ms  audio_on, input %0d path %0d", $realtime / 1e6,
                         dut.sel_cur, dut.path);
        end
    endtask

    task wait_off(input real ms_max);
        real t0;
        begin
            t0 = $realtime;
            while (audio_on && ($realtime - t0) < ms_max * 1e6) #(1000);
            if (audio_on) begin
                $display("ERROR: audio did not stop within %0.2f ms", ms_max);
                errors = errors + 1;
            end else
                $display("%8.3f ms  audio_off", $realtime / 1e6);
        end
    endtask

    initial begin
        // ---- 1. overflow recovery (input 0, PCM 96k: path X2 via u_hbb) ----
        #(1000);
        wait_on(60.0, 0, 1);
        #(3e6);
        @(posedge mclk) dut.u_hbb.ovf = 1'b1;          // single fault
        $display("%8.3f ms  forced overflow flag in u_hbb", $realtime / 1e6);
        wait_off(1.0);
        wait_on(80.0, 0, 1);
        #(3e6);
        // second fault in another stage while the first has long gone
        @(posedge mclk) dut.u_hda.ovf = 1'b1;
        $display("%8.3f ms  forced overflow flag in u_hda (idle stage)", $realtime / 1e6);
        wait_off(1.0);
        wait_on(80.0, 0, 1);

        // ---- 2. switch to input 1 (PCM 44.1: path X4) ----
        #(2e6);
        @(posedge mclk) dut.ctl_sel = 2'd1;
        wait_off(0.1);
        wait_on(80.0, 1, 0);

        // ---- 3. second request inside the hold window: 2, then 0 ----
        #(2e6);
        @(posedge mclk) dut.ctl_sel = 2'd2;
        wait_off(1.0);
        wait (dut.hold === 1'b1);
        #(2000);                                        // 2 us into the hold
        @(posedge mclk) dut.ctl_sel = 2'd0;
        $display("%8.3f ms  re-request input 0 inside the hold", $realtime / 1e6);
        wait_on(80.0, 0, 1);

        // ---- 4. and once more to input 2 (PCM 192: path X1) ----
        #(2e6);
        @(posedge mclk) dut.ctl_sel = 2'd2;
        wait_off(1.0);
        wait_on(80.0, 2, 2);

        $display("mux changes %0d", n_sw);
        if (n_sw < 3) errors = errors + 1;
        if (errors == 0) $display("PASS tb_recover");
        else             $display("FAIL tb_recover (%0d errors)", errors);
        $finish;
    end
    initial begin
        #(700e6);
        $display("FAIL tb_recover: timeout");
        $finish;
    end
endmodule
