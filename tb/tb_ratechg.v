`define AXB_SIM_PORTS   // top: bench ports (in3_dsd)
`timescale 1ns/1fs
// Source changes its sample rate on the fly (next track on a CD transport /
// streamer): the S/PDIF line keeps toggling, rate changes at a frame
// boundary, the sine keeps its frequency in Hz. Whole output is logged
// (t, audio_on, L, R); tb/run_sys.py --ratechg looks for clicks and garbage
// (high-pass residual) and wrong pitch while audio_on.
// `define V2 for ax-bridge2 (two oscillators + 50 MHz service clock).
module tb_spdif_dyn #(parameter integer SEED = 1, parameter real T0 = 100.0) (output reg line = 1'b0);
    real    fs = 44100.0, half, j, t = 0.0, fl = 997.0, fr = 3001.0, jit = 2.0;
    integer seed, frame, gap = 0;
    reg [191:0] cs;
    task pulse(input integer k);
        begin
            j = jit * (($random(seed) & 1023) / 1023.0);
            #(j); line = ~line; #(k * half - j);
        end
    endtask
    task subframe(input integer pre, input [23:0] s, input c);
        integer i, ones; reg [27:0] d;
        begin
            pulse(3);
            if (pre == 1) begin pulse(1); pulse(1); pulse(3); end
            else if (pre == 2) begin pulse(3); pulse(1); pulse(1); end
            else begin pulse(2); pulse(1); pulse(2); end
            d = {1'b0, c, 2'b00, s};
            ones = 0;
            for (i = 0; i < 27; i = i + 1) ones = ones + d[i];
            d[27] = ones[0];
            for (i = 0; i < 28; i = i + 1)
                if (d[i]) begin pulse(1); pulse(1); end else pulse(2);
        end
    endtask
    initial begin
        seed = SEED; frame = 0;
        #(T0);
        forever begin
            if (gap > 0) begin #(gap); gap = 0; end
            half = 1.0e9 / (fs * 128.0);
            cs = 192'd0; cs[2] = 1'b1;
            cs[27:24] = (fs == 48000.0) ? 4'b0100 : (fs == 96000.0) ? 4'b1010 : (fs == 88200.0) ? 4'b1000 :
                        (fs == 192000.0) ? 4'b1110 : (fs == 176400.0) ? 4'b1100 : 4'b0000;
            subframe(frame == 0 ? 1 : 2, $rtoi($sin(6.283185307179586 * fl * t) * 0.891 * 8388607.0), cs[frame]);
            subframe(3, $rtoi($sin(6.283185307179586 * fr * t) * 0.891 * 8388607.0), cs[frame]);
            t = t + 1.0 / fs;
            frame = (frame == 191) ? 0 : frame + 1;
        end
    end
endmodule

`ifndef NRATES
`define NRATES 10
`endif
`ifndef SEG_MS
`define SEG_MS 40
`endif
module tb_ratechg;
`ifdef V2
    reg sclk = 0;  always #10.0 sclk = ~sclk;
    reg o45 = 0;   always #11.072164 o45 = ~o45;
    reg o49 = 0;   always #10.172526 o49 = ~o49;
    wire mclk_sel;
    wire mclk = mclk_sel ? o45 : o49;
`else
    reg mclk = 0;  always #10.1725 mclk = ~mclk;
`endif
    wire sp0;
    tb_spdif_dyn #(.SEED(3), .T0(101.1)) s (.line(sp0));

    reg  scl_m = 1'b1, sda_m = 1'b1;
    tri1 i2c_sda;
    assign i2c_sda = sda_m ? 1'bz : 1'b0;
    wire obclk, olrck, osdata, audio_on;
`ifdef V2
    top #(.WBITS(16), .GSH(4), .MWBITS(16), .CLK_SETTLE_MS(2), .ACK_WAIT_MS(20)) dut (
        .sys_clk(sclk), .mclk_in(mclk), .mclk_sel(mclk_sel), .fox_fsel(1'b0),
`else
    top #(.WBITS(16), .GSH(4)) dut (
        .mclk_in(mclk),
`endif
        .i2s_bclk_in(1'b0), .i2s_lrck_in(1'b0), .i2s_dsdl_in(1'b0), .i2s_sdata_in(1'b0), .dsd_on(1'b0),
        .spdif_in({3'b000, sp0}),
        .in2_bclk(1'b0), .in2_lrck(1'b0), .in2_sdata(1'b0),
        .in3_bclk(1'b0), .in3_lrck(1'b0), .in3_sdata(1'b0), .in3_dsd(1'b0),
        .in4_bclk(1'b0), .in4_lrck(1'b0), .in4_sdata(1'b0),
        .i2c_scl(scl_m), .i2c_sda(i2c_sda),
        .i2s_bclk_out(obclk), .i2s_lrck_out(olrck), .i2s_sdata_out(osdata),
        .audio_on(audio_on));

    localparam real TQ = 625.0;
    task i2c_start; begin sda_m = 1; scl_m = 1; #(TQ); sda_m = 0; #(TQ); scl_m = 0; #(TQ); end endtask
    task i2c_stop;  begin sda_m = 0; #(TQ); scl_m = 1; #(TQ); sda_m = 1; #(2 * TQ); end endtask
    task i2c_bit_w(input b); begin sda_m = b; #(TQ); scl_m = 1; #(2 * TQ); scl_m = 0; #(TQ); end endtask
    task i2c_bit_r(output b); begin sda_m = 1; #(TQ); scl_m = 1; #(TQ); b = i2c_sda; #(TQ); scl_m = 0; #(TQ); end endtask
    task i2c_byte_w(input [7:0] v);
        integer i; reg a;
        begin for (i = 7; i >= 0; i = i - 1) i2c_bit_w(v[i]); i2c_bit_r(a); end
    endtask
    task wr_reg(input [7:0] r, input [7:0] v);
        begin i2c_start; i2c_byte_w({7'h5A, 1'b0}); i2c_byte_w(r); i2c_byte_w(v); i2c_stop; end
    endtask

    // whole output: t (us), audio_on, L, R
    reg        olr_q = 0;
    reg [31:0] osr = 0, ow_l = 0;
    integer    fo;
    initial fo = $fopen("build/ratechg_out.txt", "w");
    always @(posedge obclk) begin
        olr_q <= olrck;
        if (olrck != olr_q) begin
            if (olr_q == 1'b0) ow_l <= {osr[30:0], osdata};
            else $fdisplay(fo, "%0.3f %0d %0d %0d", $realtime / 1000.0, audio_on, $signed(ow_l), $signed({osr[30:0], osdata}));
            osr <= 0;
        end else
            osr <= {osr[30:0], osdata};
    end

    integer fe, i;
    real rates [0:9];
    initial begin
        fe = $fopen("build/ratechg_ev.txt", "w");
        rates[0] = 44100.0;  rates[1] = 48000.0;  rates[2] = 96000.0;  rates[3] = 88200.0;
        rates[4] = 192000.0; rates[5] = 176400.0; rates[6] = 44100.0;  rates[7] = 88200.0;
        rates[8] = 48000.0;  rates[9] = 44100.0;
        // NRATES (default 10) limits the run
        s.fs = rates[0];
        #(3e6);
`ifdef V2
        wr_reg(8'h14, 8'h05);          // auto, sync, no STM32 acknowledge
`endif
        wr_reg(8'h00, 8'h01);          // input 1, line 0
        $fdisplay(fe, "%0.3f %0.0f", $realtime / 1000.0, rates[0]);
        for (i = 1; i < `NRATES; i = i + 1) begin
            #(`SEG_MS * 1e6);
            s.fs = rates[i];
            $fdisplay(fe, "%0.3f %0.0f", $realtime / 1000.0, rates[i]);
            $display("%8.2f ms  source -> %0.0f Hz", $realtime / 1e6, rates[i]);
        end
        #(`SEG_MS * 1e6);
        $fclose(fo); $fclose(fe);
        $display("DONE tb_ratechg");
        $finish;
    end
endmodule
