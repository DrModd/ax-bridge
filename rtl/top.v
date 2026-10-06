`timescale 1ns/1ps
// AX5689 input bridge (Tang Primer 25K, GW5A-LV25MG121NC1/I0).
//
//   4 inputs: 0 = Fox (PCM/DoP/DSD), 1..3 = I2S PCM (any own clocks)
//            |  BCLK, LRCK/DATA2, SDATA/DATA1, DSD_ON (input 0)
//            v                                         local 49.152 MHz
//   [in_mux] -> [bclk_rx] -- async FIFO -->  rate_mon / DoP   [i2s_tx32] -> AX5689
//                                   |                            ^  I2S 192k/32
//    PCM 44.1/48   -> x2 hb_a -> x2 hb_b --+                     |
//    PCM 88.2/96   ->           x2 hb_b ---+                     |
//    PCM 176.4/192 -------------------------+-> ASRC (DPLL) -> fade
//    PCM 352.8/384 ->           /2 hd_b ---+
//    PCM 705.6/768 -> /2 hd_a -> /2 hd_b --+
//    DSD64..512 / DoP -> CIC5 -> /2 dsd ----+
//
// Everything except bclk_rx and the FIFO write side runs on the local
// MCLK. The input clock never reaches the output: it only clocks data in.
// Control / status: I2C slave (see README for the register map).
module top #(
    parameter WBITS = 18,      // rate_mon window 2^WBITS MCLK
    parameter GSH   = 0,       // DPLL gear length shift (sim only)
    parameter DSD_SDATA_IS_RIGHT = 1,
    parameter USE_DCS = 0,
    parameter [6:0] I2C_ADDR = 7'h5A
) (
    input  wire mclk_in,        // 49.152 MHz local oscillator
    input  wire i2s_bclk_in,    // input 0 (Fox): BCLK / DSD bit clock
    input  wire i2s_lrck_in,    //   LRCK / DSD DATA2
    input  wire i2s_sdata_in,   //   SDATA / DSD DATA1
    input  wire dsd_on,         //   Amanero-style native-DSD flag
    input  wire in1_bclk, in1_lrck, in1_sdata,   // inputs 1..3: I2S PCM
    input  wire in2_bclk, in2_lrck, in2_sdata,
    input  wire in3_bclk, in3_lrck, in3_sdata,
    input  wire i2c_scl,
    inout  wire i2c_sda,
    output wire i2s_bclk_out,
    output wire i2s_lrck_out,
    output wire i2s_sdata_out,
    output wire audio_on        // high while audio is passed (lock + unmuted)
);
    wire clk = mclk_in;

    // ---------------- resets ----------------
    reg [7:0] por = 8'd0;
    always @(posedge clk) if (~&por) por <= por + 8'd1;
    wire rst_n = &por;

    // ---------------- input selector ----------------
    // sel_cur changes only inside a hold window: front end in reset.
    reg  [1:0]  sel_cur;
    reg  [10:0] hold_cnt;
    reg         hold;                      // registered: clean async reset
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hold <= 1'b1;
        else        hold <= (hold_cnt != 11'd0);
    end
    wire rst_in_n = rst_n & ~hold;

    wire m_bclk, m_lrck, m_sdata, m_dsd;
    wire [3:0] in_active;
    in_mux #(.USE_DCS(USE_DCS)) u_mux (
        .clk(clk), .rst_n(rst_n), .sel(sel_cur),
        .bclk_in({in3_bclk, in2_bclk, in1_bclk, i2s_bclk_in}),
        .lrck_in({in3_lrck, in2_lrck, in1_lrck, i2s_lrck_in}),
        .sdata_in({in3_sdata, in2_sdata, in1_sdata, i2s_sdata_in}),
        .dsd_on0(dsd_on),
        .bclk(m_bclk), .lrck(m_lrck), .sdata(m_sdata), .dsd_on(m_dsd),
        .active(in_active)
    );

    reg [1:0] rst_b;
    always @(posedge m_bclk or negedge rst_in_n) begin
        if (!rst_in_n) rst_b <= 2'b00;
        else           rst_b <= {rst_b[0], 1'b1};
    end

    // ---------------- input clock domain ----------------
    wire        rx_we;
    wire [64:0] rx_wd;
    bclk_rx #(.SDATA_IS_RIGHT(DSD_SDATA_IS_RIGHT)) u_rx (
        .bclk(m_bclk), .rst_n(rst_b[1]),
        .lrck(m_lrck), .sdata(m_sdata), .dsd_on(m_dsd),
        .wr_en(rx_we), .wr_data(rx_wd)
    );

    wire        f_empty, f_full;
    wire [64:0] f_rd;
    reg         f_re;
    afifo #(.W(65), .AW(4)) u_fifo (
        .wclk(m_bclk), .wrst_n(rst_b[1]), .we(rx_we), .wdata(rx_wd), .full(f_full),
        .rclk(clk), .rrst_n(rst_in_n), .re(f_re), .rdata(f_rd), .empty(f_empty)
    );

    // ---------------- item intake (MCLK) ----------------
    // At most one item per 8 MCLK (the densest input, DSD512, delivers
    // one item per ~35 MCLK) so a FIFO backlog never floods a stage queue.
    reg [2:0]  thr;
    reg        item;
    reg        item_dsd;
    reg [31:0] item_l, item_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            thr <= 3'd0; f_re <= 1'b0; item <= 1'b0; item_dsd <= 1'b0;
            item_l <= 32'd0; item_r <= 32'd0;
        end else begin
            f_re <= 1'b0;
            item <= 1'b0;
            if (thr != 3'd0) thr <= thr - 3'd1;
            if (!f_empty && !f_re && thr == 3'd0) begin
                f_re <= 1'b1;
                item <= 1'b1;
                item_dsd <= f_rd[64];
                item_l <= f_rd[63:32];
                item_r <= f_rd[31:0];
                thr <= 3'd7;
            end
        end
    end
    wire item_pcm = item & ~item_dsd;

    wire dop;
    dop_det u_dop (
        .clk(clk), .rst_n(rst_n), .frame(item_pcm), .l(item_l), .r(item_r), .dop(dop)
    );

    wire       rm_stable, rm_fam48, rm_dsd, rm_dop, rm_seq;
    wire [2:0] rm_k;
    rate_mon #(.WBITS(WBITS)) u_rm (
        .clk(clk), .rst_n(rst_n), .item(item), .item_dsd(item_dsd), .dop(dop),
        .stable(rm_stable), .k(rm_k), .fam48(rm_fam48), .dsd(rm_dsd),
        .is_dop(rm_dop), .seq(rm_seq)
    );

    // ---------------- configuration / control ----------------
    localparam P_X4 = 3'd0, P_X2 = 3'd1, P_X1 = 3'd2, P_D2 = 3'd3,
               P_D4 = 3'd4, P_DSD = 3'd5;
    localparam C_IDLE = 3'd0, C_FILL = 3'd1, C_LOCK = 3'd2, C_RUN = 3'd3,
               C_FADE = 3'd4;

    // class -> path (combinational, from rate_mon)
    reg [2:0] cls_path;
    reg       cls_ok;
    always @(*) begin
        cls_ok = rm_stable;
        cls_path = P_X1;
        if (rm_dsd) begin
            cls_path = P_DSD;
            if (rm_k < 3'd2) cls_ok = 1'b0;                   // DSD64..512
        end else if (rm_dop) begin
            cls_path = P_DSD;
            if (rm_k < 3'd2 || rm_k > 3'd4) cls_ok = 1'b0;    // DoP 64..256
        end else begin
            case (rm_k)
                3'd0: cls_path = P_X4;
                3'd1: cls_path = P_X2;
                3'd2: cls_path = P_X1;
                3'd3: cls_path = P_D2;
                3'd4: cls_path = P_D4;
                default: cls_ok = 1'b0;
            endcase
        end
    end

    reg [2:0] cst;
    reg [2:0] path;
    reg [2:0] cfg_k;
    reg       cfg_fam48, cfg_dsd, cfg_dop;
    reg       seq_seen;
    reg [1:0] fresh;          // window ends seen since entering IDLE
    reg [24:0] tmo;           // FILL/LOCK timeout (0.68 s)
    wire      running = (cst == C_FILL) || (cst == C_LOCK) || (cst == C_RUN);

    // ---- events: one per 2^k items (44.1 / 48 kHz) ----
    reg [4:0] ecnt;
    wire [4:0] emask = (5'd1 << cfg_k) - 5'd1;
    wire ev = running && item && ((ecnt & emask) == emask);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ecnt <= 5'd0;
        else if (!running) ecnt <= 5'd0;
        else if (item) ecnt <= ecnt + 5'd1;
    end

    // ---- event interval watchdog (1114.6 / 1024 MCLK nominal, +-3 %) ----
    reg [11:0] evint;
    reg        evseen;
    reg        ev_bad;
    wire [11:0] ev_lo = cfg_fam48 ? 12'd992  : 12'd1081;
    wire [11:0] ev_hi = cfg_fam48 ? 12'd1056 : 12'd1148;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            evint <= 12'd0; evseen <= 1'b0; ev_bad <= 1'b0;
        end else if (!running) begin
            evint <= 12'd0; evseen <= 1'b0; ev_bad <= 1'b0;
        end else begin
            if (ev) begin
                evint <= 12'd1;
                evseen <= 1'b1;
                if (evseen && (evint < ev_lo || evint > ev_hi))
                    ev_bad <= 1'b1;
            end else begin
                if (~&evint) evint <= evint + 12'd1;
                if (evseen && evint > ev_hi)
                    ev_bad <= 1'b1;
            end
        end
    end

    // ---- live mode mismatch (immediate) ----
    wire mode_bad = running && item &&
                    ((item_dsd != cfg_dsd) || (!item_dsd && (dop != cfg_dop)));

    // ---------------- DSP stages ----------------
    wire stg_clr = (cst == C_IDLE);
    wire feed = running || (cst == C_FADE);
    wire signed [23:0] pl = item_l[31:8];
    wire signed [23:0] pr = item_r[31:8];
    wire pcm_feed = feed & item_pcm & ~cfg_dop & ~cfg_dsd;

    wire hba_v, hbb_v, hda_v, hdb_v, cic_v, dsf_v;
    wire signed [23:0] hba_l, hba_r, hbb_l, hbb_r, hda_l, hda_r, hdb_l, hdb_r;
    wire signed [23:0] cic_l, cic_r, dsf_l, dsf_r;
    wire ovf_a, ovf_b, ovf_c, ovf_d, ovf_e, ovf_f;

    fir_stage #(.N(175), .INTERP(1), .AW(7), .CAW(8), .FILE("coef/hb_a.hex")) u_hba (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(pcm_feed && path == P_X4), .in_l(pl), .in_r(pr),
        .out_valid(hba_v), .out_l(hba_l), .out_r(hba_r), .ovf(ovf_a));

    wire hbb_in = (path == P_X4) ? hba_v : (pcm_feed && path == P_X2);
    fir_stage #(.N(91), .INTERP(1), .AW(6), .CAW(7), .FILE("coef/hb_b.hex")) u_hbb (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(hbb_in),
        .in_l(path == P_X4 ? hba_l : pl), .in_r(path == P_X4 ? hba_r : pr),
        .out_valid(hbb_v), .out_l(hbb_l), .out_r(hbb_r), .ovf(ovf_b));

    fir_stage #(.N(23), .INTERP(0), .AW(5), .CAW(5), .FILE("coef/hd_a.hex")) u_hda (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(pcm_feed && path == P_D4), .in_l(pl), .in_r(pr),
        .out_valid(hda_v), .out_l(hda_l), .out_r(hda_r), .ovf(ovf_c));

    wire hdb_in = (path == P_D4) ? hda_v : (pcm_feed && path == P_D2);
    fir_stage #(.N(31), .INTERP(0), .AW(5), .CAW(5), .FILE("coef/hd_b.hex")) u_hdb (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(hdb_in),
        .in_l(path == P_D4 ? hda_l : pl), .in_r(path == P_D4 ? hda_r : pr),
        .out_valid(hdb_v), .out_l(hdb_l), .out_r(hdb_r), .ovf(ovf_d));

    // DSD chunks: native items carry {L16,R16} in item_r; DoP frames carry
    // 16 bits per channel under the marker byte.
    wire cic_in = feed && (path == P_DSD) && item &&
                  (cfg_dsd ? item_dsd : (!item_dsd && cfg_dop));
    wire [15:0] ch_l = cfg_dsd ? item_r[31:16] : item_l[23:8];
    wire [15:0] ch_r = cfg_dsd ? item_r[15:0]  : item_r[23:8];
    cic5_dsd u_cic (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr), .r_log2(cfg_k + 3'd1),
        .in_valid(cic_in), .in_l(ch_l), .in_r(ch_r),
        .out_valid(cic_v), .out_l(cic_l), .out_r(cic_r), .ovf(ovf_e));

    fir_stage #(.N(95), .INTERP(0), .AW(7), .CAW(7), .FILE("coef/dsd.hex")) u_dsf (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr),
        .in_valid(cic_v), .in_l(cic_l), .in_r(cic_r),
        .out_valid(dsf_v), .out_l(dsf_l), .out_r(dsf_r), .ovf(ovf_f));

    wire stage_ovf = ovf_a | ovf_b | ovf_c | ovf_d | ovf_e | ovf_f;

    // intermediate stream -> ASRC
    reg               iv;
    reg signed [23:0] il, ir;
    always @(*) begin
        case (path)
            P_X4, P_X2: begin iv = hbb_v; il = hbb_l; ir = hbb_r; end
            P_D2, P_D4: begin iv = hdb_v; il = hdb_l; ir = hdb_r; end
            P_DSD:      begin iv = dsf_v; il = dsf_l; ir = dsf_r; end
            default:    begin iv = pcm_feed; il = pl; ir = pr; end
        endcase
    end

    wire [7:0] fpos;
    wire       a_yv, a_started, a_fault;
    wire [2:0] a_gear;
    wire signed [31:0] a_yl, a_yr;
    wire signed [47:0] a_e;
    asrc #(.GSH(GSH), .FILE("coef/asrc.hex")) u_asrc (
        .clk(clk), .rst_n(rst_n), .clr(stg_clr), .fam48(cfg_fam48),
        .arm(running), .in_valid(iv), .in_l(il), .in_r(ir),
        .ev(ev), .fpos(fpos),
        .y_valid(a_yv), .y_l(a_yl), .y_r(a_yr),
        .started(a_started), .gear(a_gear), .fault(a_fault), .dbg_e(a_e));

    // ---------------- control FSM ----------------
    reg [10:0] gain;          // 0..1024
    reg  [1:0] ctl_sel;       // I2C: requested input
    reg        ctl_mute;      // I2C: forced mute
    wire sw_req = (ctl_sel != sel_cur);
    wire fault = a_fault | ev_bad | mode_bad | stage_ovf | sw_req | ctl_mute |
                 (rm_seq != seq_seen && (!cls_ok || cls_path != path ||
                  rm_k != cfg_k || rm_fam48 != cfg_fam48));
    wire tick = (fpos == 8'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cst <= C_IDLE; path <= P_X1; cfg_k <= 3'd0; cfg_fam48 <= 1'b0;
            cfg_dsd <= 1'b0; cfg_dop <= 1'b0; seq_seen <= 1'b0; fresh <= 2'd0;
            tmo <= 25'd0; gain <= 11'd0;
            sel_cur <= 2'd0; hold_cnt <= 11'd0;
        end else begin
            if (hold_cnt != 11'd0)
                hold_cnt <= hold_cnt - 11'd1;
            case (cst)
                C_IDLE: begin
                    gain <= 11'd0;
                    if (rm_seq != seq_seen) begin
                        seq_seen <= rm_seq;
                        if (fresh != 2'd2) fresh <= fresh + 2'd1;
                    end
                    if (sw_req) begin
                        // switch input: front end held in reset ~42 us
                        sel_cur <= ctl_sel;
                        hold_cnt <= 11'd2047;
                        fresh <= 2'd0;
                    end else if (hold_cnt != 11'd0) begin
                        fresh <= 2'd0;
                    end else if (fresh == 2'd2 && cls_ok && !ctl_mute) begin
                        path <= cls_path;
                        cfg_k <= rm_k;
                        cfg_fam48 <= rm_fam48;
                        cfg_dsd <= rm_dsd;
                        cfg_dop <= rm_dop;
                        seq_seen <= rm_seq;
                        tmo <= 25'd0;
                        cst <= C_FILL;
                    end
                end
                C_FILL, C_LOCK, C_RUN: begin
                    if (rm_seq != seq_seen) seq_seen <= rm_seq;
                    if (cst != C_RUN) tmo <= tmo + 25'd1;
                    if (fault || (&tmo)) begin
                        cst <= C_FADE;
                    end else begin
                        if (cst == C_FILL && a_started) cst <= C_LOCK;
                        if (cst == C_LOCK && a_gear >= 3'd2) cst <= C_RUN;
                        if (cst == C_RUN && tick && gain != 11'd1024)
                            gain <= gain + 11'd1;
                    end
                end
                C_FADE: begin
                    if (tick) begin
                        if (gain <= 11'd8) begin
                            gain <= 11'd0;
                            fresh <= 2'd0;
                            seq_seen <= rm_seq;
                            cst <= C_IDLE;
                        end else begin
                            gain <= gain - 11'd8;
                        end
                    end
                end
                default: cst <= C_IDLE;
            endcase
        end
    end

    // ---------------- output: hold, fade, TX ----------------
    reg signed [31:0] hl, hr;     // last ASRC output (coasts in FADE)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hl <= 32'sd0; hr <= 32'sd0;
        end else if (cst == C_IDLE) begin
            hl <= 32'sd0; hr <= 32'sd0;
        end else if (a_yv && cst != C_FADE) begin
            hl <= a_yl; hr <= a_yr;
        end
    end
    wire signed [43:0] gl = hl * $signed({1'b0, gain});
    wire signed [43:0] gr = hr * $signed({1'b0, gain});
    reg  signed [31:0] ol, orr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ol <= 32'sd0; orr <= 32'sd0;
        end else if (fpos == 8'd255) begin
            ol <= gl[41:10];
            orr <= gr[41:10];
        end
    end

    i2s_tx32 u_tx (
        .clk(clk), .rst_n(rst_n), .in_l(ol), .in_r(orr), .fpos(fpos),
        .bclk_out(i2s_bclk_out), .lrck_out(i2s_lrck_out), .sdata_out(i2s_sdata_out)
    );

    assign audio_on = (cst == C_RUN);

    // ---------------- I2C control / status ----------------
    // 0x00 CTRL    rw  [1:0] input select, [7] mute
    // 0x01 STATUS  r   [0] audio on, [1] locking/locked, [3:2] current
    //                  input, [7:4] inputs with a live LRCK
    // 0x02 FORMAT  r   [2:0] k, [3] 48k family, [4] native DSD, [5] DoP,
    //                  [7] valid
    // 0x03 RATE    r   0 none; PCM 1..10 = 44.1,48,88.2,96,176.4,192,
    //                  352.8,384,705.6,768; 0x11..0x14 DSD64..512;
    //                  0x21..0x23 DoP64..256 (+0x08: 48k-family DSD)
    // 0x04 PLL     r   DPLL gear (0..4)
    // 0x05 ID      r   0xA5
    // 0x06 VERSION r   0x02
    wire       sda_oe;
    wire [7:0] i2c_ptr, i2c_waddr, i2c_wdata;
    wire       i2c_wr;
    reg  [7:0] i2c_rd;
    reg  [7:0] rate_code;
    assign i2c_sda = sda_oe ? 1'b0 : 1'bz;
    i2c_slave #(.ADDR(I2C_ADDR)) u_i2c (
        .clk(clk), .rst_n(rst_n), .scl_in(i2c_scl), .sda_in(i2c_sda),
        .sda_oe(sda_oe), .ptr(i2c_ptr), .rdata(i2c_rd),
        .wr(i2c_wr), .waddr(i2c_waddr), .wdata(i2c_wdata));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctl_sel <= 2'd0; ctl_mute <= 1'b0;
        end else if (i2c_wr && i2c_waddr == 8'h00) begin
            ctl_sel <= i2c_wdata[1:0];
            ctl_mute <= i2c_wdata[7];
        end
    end

    always @(*) begin
        if (!running)
            rate_code = 8'h00;
        else if (cfg_dsd)
            rate_code = {4'h1, cfg_fam48, cfg_k - 3'd1};
        else if (cfg_dop)
            rate_code = {4'h2, cfg_fam48, cfg_k - 3'd1};
        else
            rate_code = 8'd1 + {4'd0, cfg_k, 1'b0} + {7'd0, cfg_fam48};
        case (i2c_ptr)
            8'h00: i2c_rd = {ctl_mute, 5'd0, ctl_sel};
            8'h01: i2c_rd = {in_active, sel_cur, running, cst == C_RUN};
            8'h02: i2c_rd = {running, 1'b0, cfg_dop, cfg_dsd, cfg_fam48, cfg_k};
            8'h03: i2c_rd = rate_code;
            8'h04: i2c_rd = {5'd0, a_gear};
            8'h05: i2c_rd = 8'hA5;
            8'h06: i2c_rd = 8'h02;
            default: i2c_rd = 8'h00;
        endcase
    end
endmodule
