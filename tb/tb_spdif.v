`timescale 1ns/1fs
// S/PDIF receiver (input 1, tb/run_sys.py --spdif). Four lines at once on
// unrelated clocks:
//   line 0 coax     192 kHz, +35 ppm, 4 ns jitter, 10 ns duty distortion
//   line 1 optical   96 kHz, -80 ppm, 15 ns duty distortion
//   line 2 AES/EBU   48 kHz, +12 ppm
//   line 3 ARC       44.1 kHz, non-audio (channel status bit 1): must stay silent
module tb_spdif;
    reg mclk = 0;
    always #10.1725 mclk = ~mclk;

    wire [3:0] sp;
    tb_spdif_src #(.FS(192000.0), .PPM(35.0), .FL(1201.0), .FR(4001.0), .JIT(4.0), .DUTY(10.0),
                   .SEED(3), .T0(101.1)) s0 (.en(1'b1), .line(sp[0]));
    tb_spdif_src #(.FS(96000.0), .PPM(-80.0), .FL(997.0), .FR(3001.0), .DUTY(-15.0),
                   .SEED(5), .T0(202.2)) s1 (.en(1'b1), .line(sp[1]));
    tb_spdif_src #(.FS(48000.0), .PPM(12.0), .FL(1499.0), .FR(5003.0), .JIT(2.0),
                   .SEED(7), .T0(303.3)) s2 (.en(1'b1), .line(sp[2]));
    tb_spdif_src #(.FS(44100.0), .PPM(0.0), .NONAUDIO(1), .SEED(9), .T0(404.4)) s3 (.en(1'b1), .line(sp[3]));

    reg  scl_m = 1'b1, sda_m = 1'b1;
    tri1 i2c_sda;
    assign i2c_sda = sda_m ? 1'bz : 1'b0;

    wire obclk, olrck, osdata, audio_on;
    top #(.WBITS(16), .GSH(4)) dut (
        .mclk_in(mclk),
        .i2s_bclk_in(1'b0), .i2s_lrck_in(1'b0), .i2s_sdata_in(1'b0), .dsd_on(1'b0),
        .spdif_in(sp),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0), .in3_dsd(1'b0),
        .in4_bclk(1'b0), .in4_lrck(1'b0), .in4_sdata(1'b0),
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
            $sformat(name, "build/spdif_seg%0d.txt", seg);
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

    task stays_off(input real ms);
        real t0;
        begin
            t0 = $realtime;
            while (($realtime - t0) < ms * 1e6) begin
                if (audio_on) begin
                    $display("ERROR: audio on at %0.2f ms (must stay silent)", $realtime / 1e6);
                    errors = errors + 1;
                    t0 = -1e12;
                end
                #(10000);
            end
            $display("%8.2f ms  silent as expected", $realtime / 1e6);
        end
    endtask

    reg [7:0] r0, r1, r2, r3;
    initial begin
        #(3e6);
        rd_regs(8'h05, r0, r1, r2, r3);
        check(r0, 8'hA5, "ID"); check(r1, 8'h07, "VERSION");
        // line 0: coax 192k
        wr_reg(8'h00, 8'h01);
        wait_on(80.0);
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h8A, "FORMAT"); check(r1, 8'h06, "RATE");
        rd_regs(8'h10, r0, r1, r2, r3);
        check(r0, 8'h00, "SPDIF_SEL"); check(r1, 8'h01, "SPDIF_STAT");
        check(r2, 8'h04, "CS0"); check(r3, 8'h07, "CS3");
        capture(0, 4096);
        // line 1: optical 96k
        wr_reg(8'h10, 8'h01);
        #(20000);
        if (audio_on) begin $display("ERROR: audio still on after line switch"); errors = errors + 1; end
        wait_on(80.0);
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h89, "FORMAT"); check(r1, 8'h04, "RATE");
        capture(1, 4096);
        // line 2: AES 48k
        wr_reg(8'h10, 8'h02);
        wait_on(80.0);
        rd_regs(8'h02, r0, r1, r2, r3);
        check(r0, 8'h88, "FORMAT"); check(r1, 8'h02, "RATE");
        capture(2, 4096);
        // line 3: ARC, non-audio: must stay silent, status shows it
        wr_reg(8'h10, 8'h03);
        stays_off(40.0);
        rd_regs(8'h11, r0, r1, r2, r3);
        check(r0, 8'h03, "SPDIF_STAT"); check(r1, 8'h06, "CS0");
        rd_regs(8'h0F, r0, r1, r2, r3);
        check(r0, 8'h22, "INPUTS");     // input 1, S/PDIF locked
        // back to coax
        wr_reg(8'h10, 8'h00);
        wait_on(80.0);
        rd_regs(8'h11, r0, r1, r2, r3);
        check(r0 & 8'h7F, 8'h01, "SPDIF_STAT");
        capture(3, 4096);
        if (errors == 0) $display("PASS tb_spdif");
        else             $display("FAIL tb_spdif (%0d errors)", errors);
        $finish;
    end
    initial begin
        #(900e6);
        $display("FAIL tb_spdif: timeout");
        $finish;
    end
endmodule
