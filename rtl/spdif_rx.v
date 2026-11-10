`timescale 1ns/1ps
// S/PDIF / AES3 receiver (IEC 60958), runs entirely on the local MCLK.
// Replaces the AK4118A: four lines (coax, optical, AES/EBU, HDMI ARC),
// one decoded at a time. 44.1..192 kHz, 16..24-bit PCM.
//
// Sampling: every line is sampled on both MCLK edges (98.304 MS/s with
// MCLK = 49.152 MHz); the negedge sample is moved to the posedge domain,
// so the decoder gets two samples per clock, oldest first. At 192 kHz a
// biphase half-cell (1T) is 4 samples, a preamble pulse (3T) 12.
//
// Pulse classifier: the longest pulse of each polarity in a window of 256
// pulses is a preamble pulse (3T; B, M and W preambles contain 3T pulses
// of both polarities). Pulse-width distortion (optical receivers stretch
// one level and shorten the other by D) shifts the two maxima by +D / -D,
// so their sum S = Mh + Ml = 6T is distortion-free, and a pulse of a
// level whose maximum is M (= 3T +- D) is classified around that level's
// own grid:
//   4L <  4M - S      -> 1T   (L < M - 1.5T)
//   12L < 12M - S     -> 2T   (L < M - 0.5T)
//   12L < 12M + S     -> 3T   (L < M + 0.5T), longer -> error
// That coarse grid (the maxima carry +-1 sample of quantisation) only
// bootstraps the decoder. Once subframes decode, a fine grid takes over:
//   T  = (length of 8 good subframes = 512T) / 512, in 1/16 sample
//   Dh, Dl = mean residual (16L - n*T) of 64 decoded pulses of each level
//   x = 16L - D(level):  < 1.5T -> 1T,  < 2.5T -> 2T,  < 3.5T -> 3T
// The residuals also absorb the sampling bias (at 192 kHz T is ~4 whole
// samples, so every edge of one level is rounded the same way), so the
// thresholds sit in the middle between the measured pulse classes.
// No rate table; at 192 kHz this takes about +-15 ns of duty distortion
// plus jitter with 98 MS/s sampling.
//
// Subframe: preamble B/M (left; B = block start) or W (right) as pulse
// patterns 3-1-1-3 / 3-3-1-1 / 3-2-1-2, then 28 biphase cells (2T = 0,
// 1T 1T = 1): aux[3:0], audio[19:0] (LSB first: 24-bit sample = bits
// 4..27), V, U, C, P (even parity over bits 4..31).
//
// Output: one stereo frame per W subframe, 24-bit samples left-justified
// in 32 bits, only while locked (16 good frames in a row) and the stream is
// known to be audio:
//   - nothing leaves before one whole channel-status block (192 frames) has
//     been read: until then nobody knows whether it is PCM;
//   - channel status byte 0 bit 1 = 1 (AC3, DTS, ...) -> silent;
//   - IEC 61937 burst preamble Pa/Pb (0xF872 / 0x4E1F in the top 16 bits
//     of a frame) -> silent at once and for ~170 ms after the last one:
//     a player switching from PCM to Dolby flips the channel-status bit
//     only at the next block, and some sources never set it.
// A frame with a parity or biphase error (or a missing left subframe)
// repeats the previous one, so the frame rate seen downstream never
// changes.
// The fine grid is dropped (back to bootstrap) when no subframe has
// decoded cleanly for ~330 us: a source that changes its sample rate
// without a gap (x2, x4) would otherwise keep a stale grid for ever.
module spdif_rx (
    input  wire        clk,
    input  wire        rst_n,       // held low while the input is switched
    input  wire [3:0]  line,        // raw pins
    input  wire [1:0]  sel,         // line select (quasi-static, changed under reset)
    output reg         valid,       // one-clock strobe per stereo frame
    output reg  [31:0] l,
    output reg  [31:0] r,
    output reg         locked,
    output wire        nonaudio,    // not PCM: channel status (AC3, DTS, ...) or IEC 61937 bursts
    output reg         err,         // one-clock strobe: parity / biphase error
    output reg  [7:0]  cs0,         // channel status byte 0
    output reg  [7:0]  cs3          // channel status byte 3 (sample rate code)
);
    // ---------------- double-rate sampling ----------------
    reg [3:0] pa, nb;
    always @(posedge clk) pa <= line;
    always @(negedge clk) nb <= line;
    reg [3:0] a1, b1;                 // second synchroniser stage
    always @(posedge clk) begin a1 <= pa; b1 <= nb; end
    // pair at this clock: a1 (older), b1 (half a clock later)
    reg s0, s1;
    always @(posedge clk) begin s0 <= a1[sel]; s1 <= b1[sel]; end

    // ---------------- pulse lengths (in samples) ----------------
    reg       last;                   // level of the previous sample
    reg [7:0] run;                    // samples since the last edge
    reg       pv;                     // pulse strobe
    reg [7:0] plen;
    reg       plev;                   // level of the pulse that just ended
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last <= 1'b0; run <= 8'd0; pv <= 1'b0; plen <= 8'd0; plev <= 1'b0;
        end else begin
            pv <= 1'b0;
            last <= s1;
            if (s0 != last && s1 == s0) begin              // edge before s0
                pv <= 1'b1; plen <= run; run <= 8'd2; plev <= last;
            end else if (s0 == last && s1 != s0) begin     // edge between s0 and s1
                pv <= 1'b1; plen <= run + 8'd1; run <= 8'd1; plev <= s0;
            end else begin                                 // no edge, or a 1-sample spike
                if (run < 8'd254) run <= run + 8'd2;
            end
        end
    end

    // ---------------- classifier ----------------
    reg [7:0] wmh, wml, mh, ml;       // window maxima / references (3T), high / low pulses
    reg [7:0] wcnt;
    reg       ref_ok;
    wire [7:0] nmh = (plev && plen > wmh) ? plen : wmh;
    wire [7:0] nml = (!plev && plen > wml) ? plen : wml;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wmh <= 8'd0; wml <= 8'd0; mh <= 8'd0; ml <= 8'd0; wcnt <= 8'd0; ref_ok <= 1'b0;
        end else if (run >= 8'd250) begin                  // no signal: forget the reference
            wmh <= 8'd0; wml <= 8'd0; wcnt <= 8'd0; ref_ok <= 1'b0;
        end else if (pv) begin
            wcnt <= wcnt + 8'd1;
            if (wcnt == 8'd255) begin
                mh <= nmh; ml <= nml;
                ref_ok <= ({1'b0, nmh} + {1'b0, nml}) >= 9'd18;   // 6T >= 18 samples (192 kHz minus jitter)
                wmh <= 8'd0; wml <= 8'd0;
            end else begin
                wmh <= nmh; wml <= nml;
            end
        end
    end
    wire [12:0] S   = {5'd0, mh} + {5'd0, ml};
    wire [12:0] M   = {5'd0, plev ? mh : ml};
    wire [12:0] L   = {5'd0, plen};
    wire [12:0] L4  = L << 2,  M4 = M << 2;
    wire [12:0] L12 = (L << 3) + (L << 2), M12 = (M << 3) + (M << 2);
    wire k1 = (L4 + S) < M4;                          // coarse grid (bootstrap)
    wire k2 = !k1 && ((L12 + S) < M12);
    wire k3 = !k1 && !k2 && (L12 < M12 + S);

    // fine grid
    reg  [11:0] t16;                  // T in 1/16 sample (192 kHz: ~64, 44.1 kHz: ~279)
    reg  signed [12:0] dh16, dl16;    // per-level offset, 1/16 sample
    reg         fine;                 // fine grid valid
    wire signed [14:0] x16 = $signed({3'd0, plen, 4'd0}) - (plev ? dh16 : dl16);
    wire signed [14:0] h15 = $signed({3'd0, t16}) + $signed({4'd0, t16[11:1]});   // 1.5T
    wire signed [14:0] h25 = h15 + $signed({3'd0, t16});
    wire signed [14:0] h35 = h25 + $signed({3'd0, t16});
    wire f1 = x16 < h15;
    wire f2 = !f1 && (x16 < h25);
    wire f3 = !f1 && !f2 && (x16 < h35);
    wire c1 = fine ? f1 : k1;
    wire c2 = fine ? f2 : k2;
    wire c3 = fine ? f3 : k3;
    // class: 1, 2, 3; 0 = invalid
    wire [1:0] cls = c1 ? 2'd1 : c2 ? 2'd2 : c3 ? 2'd3 : 2'd0;

    // ---- fine-grid estimation (only from pulses of correctly decoded subframes) ----
    reg  [11:0] sub;                  // samples since the end of the last preamble 3T
    reg  [14:0] tsum;                 // 8 subframes
    reg  [2:0]  tn;
    reg  signed [18:0] rh, rl;        // residual sums
    reg  [5:0]  rnh, rnl;
    reg         t_ok, dh_ok, dl_ok;
    wire [13:0] nt16 = (cls == 2'd1) ? {2'd0, t16} : (cls == 2'd2) ? {1'd0, t16, 1'b0} :
                       ({2'd0, t16} + {1'd0, t16, 1'b0});
    wire signed [14:0] res16 = $signed({3'd0, plen, 4'd0}) - $signed({1'b0, nt16});
    wire        sub_end;              // a preamble 3T ends a subframe (64T since the last one)
    wire        res_take;             // pulse taken for the level offsets
    reg  [13:0] cl_wd;                // clocks since the last clean subframe
    wire        stale = &cl_wd;       // ~330 us: grid no longer fits the stream
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub <= 12'd0; tsum <= 15'd0; tn <= 3'd0; t16 <= 12'd0;
            rh <= 19'sd0; rl <= 19'sd0; rnh <= 6'd0; rnl <= 6'd0; dh16 <= 13'sd0; dl16 <= 13'sd0;
            t_ok <= 1'b0; dh_ok <= 1'b0; dl_ok <= 1'b0; fine <= 1'b0;
        end else begin
            if (!ref_ok || stale) begin                     // signal lost / grid stale: back to bootstrap
                t_ok <= 1'b0; dh_ok <= 1'b0; dl_ok <= 1'b0; fine <= 1'b0;
                tn <= 3'd0; tsum <= 15'd0; rnh <= 6'd0; rnl <= 6'd0; rh <= 19'sd0; rl <= 19'sd0;
            end else if (pv) begin
                if (sub_end) begin
                    sub <= 12'd0;
                    if (clean && sub + {4'd0, plen} < 12'd3000) begin  // clean subframe; 64T, 32 kHz: ~1570
                        tsum <= (tn == 3'd0 ? 15'd0 : tsum) + {3'd0, sub} + {7'd0, plen};
                        tn <= tn + 3'd1;
                        if (tn == 3'd7) begin
                            t16 <= (tsum + {3'd0, sub} + {7'd0, plen}) >> 5;   // 512T -> T/16
                            t_ok <= 1'b1;
                        end
                    end
                end else if (sub < 12'd4000)
                    sub <= sub + {4'd0, plen};
                if (res_take && t_ok && cls != 2'd0) begin
                    if (plev) begin
                        rh <= (rnh == 6'd0 ? 19'sd0 : rh) + res16;
                        rnh <= rnh + 6'd1;
                        if (rnh == 6'd63) begin dh16 <= (rh + res16) >>> 6; dh_ok <= 1'b1; end
                    end else begin
                        rl <= (rnl == 6'd0 ? 19'sd0 : rl) + res16;
                        rnl <= rnl + 6'd1;
                        if (rnl == 6'd63) begin dl16 <= (rl + res16) >>> 6; dl_ok <= 1'b1; end
                    end
                end
                fine <= t_ok && dh_ok && dl_ok;
            end
        end
    end

    // ---------------- biphase / preamble decoder ----------------
    localparam S_HUNT = 2'd0, S_PRE = 2'd1, S_DATA = 2'd2;
    reg [1:0]  st;
    reg [1:0]  pn;                    // preamble pulses seen after the first 3T
    reg [5:0]  pat;                   // classes of preamble pulses 2..4
    reg        half;                  // first 1T of a '1' cell seen
    reg [4:0]  nb_;                   // data bits received
    reg [27:0] sr;
    reg [1:0]  ptype;                 // 1 = B, 2 = M, 3 = W
    reg        sf_v;                  // subframe complete strobe
    reg        clean;                 // last subframe decoded without error
    reg        sf_bad;
    reg [1:0]  sf_type;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cl_wd <= 14'd0;
        else if (sf_v && !sf_bad && clean) cl_wd <= 14'd0;   // clean: one clock after sf_v
        else if (!stale) cl_wd <= cl_wd + 14'd1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_HUNT; pn <= 2'd0; pat <= 6'd0; half <= 1'b0; nb_ <= 5'd0;
            sr <= 28'd0; ptype <= 2'd0; sf_v <= 1'b0; sf_bad <= 1'b0; sf_type <= 2'd0; clean <= 1'b0;
        end else begin
            sf_v <= 1'b0;
            if (pv && ref_ok) begin
                case (st)
                S_HUNT: begin
                    if (cls == 2'd3) begin st <= S_PRE; pn <= 2'd0; end
                    else clean <= 1'b0;
                end
                S_PRE: begin
                    pat <= {pat[3:0], cls};
                    pn <= pn + 2'd1;
                    if (pn == 2'd2) begin
                        // pattern after the first 3T: B 1-1-3, M 3-1-1, W 2-1-2
                        case ({pat[3:0], cls})
                        6'b01_01_11: begin ptype <= 2'd1; st <= S_DATA; end
                        6'b11_01_01: begin ptype <= 2'd2; st <= S_DATA; end
                        6'b10_01_10: begin ptype <= 2'd3; st <= S_DATA; end
                        default:     begin st <= S_HUNT; clean <= 1'b0; end
                        endcase
                        half <= 1'b0; nb_ <= 5'd0;
                    end
                end
                default: begin // S_DATA
                    if (cls == 2'd2 && !half) begin
                        sr <= {1'b0, sr[27:1]}; nb_ <= nb_ + 5'd1;
                    end else if (cls == 2'd1) begin
                        if (half) begin sr <= {1'b1, sr[27:1]}; nb_ <= nb_ + 5'd1; end
                        half <= ~half;
                    end else begin
                        // 3T or invalid inside the data: lost sync
                        sf_v <= 1'b1; sf_bad <= 1'b1; sf_type <= ptype; clean <= 1'b0;
                        st <= (cls == 2'd3) ? S_PRE : S_HUNT; pn <= 2'd0;
                    end
                    if (nb_ == 5'd27 && ((cls == 2'd2 && !half) || (cls == 2'd1 && half))) begin
                        sf_v <= 1'b1; sf_bad <= 1'b0; sf_type <= ptype; clean <= ~^sr[27:1] ^ (cls == 2'd1);
                        st <= S_HUNT;
                    end
                end
                endcase
            end else if (pv && st != S_HUNT) begin
                st <= S_HUNT;                     // no reference yet
                clean <= 1'b0;
            end
        end
    end

    assign sub_end  = pv && ref_ok && st == S_HUNT && cls == 2'd3;
    assign res_take = locked && (st != S_HUNT || cls == 2'd3);

    // ---------------- frames, channel status, lock ----------------
    // sr after the last bit: sr[0] = bit 4 ... sr[27] = bit 31 (one clock
    // after sf_v, the shift has landed)
    reg        sf_d;
    reg        sf_bad_d;
    reg [1:0]  sf_type_d;
    always @(posedge clk) begin sf_d <= sf_v; sf_bad_d <= sf_bad; sf_type_d <= sf_type; end
    wire       par_ok = ~^sr;
    wire [23:0] smp = sr[23:0];

    reg [31:0] hl;                     // left sample waiting for its right
    reg        have_l, l_bad;
    reg [31:0] cs;
    reg [7:0]  csi;                    // channel-status bit index (left subframes)
    reg        cs_run;
    reg        cs_ok;                  // one whole channel-status block read
    reg [4:0]  good;
    reg [15:0] wd;                     // watchdog: clocks since the last good frame
    reg [22:0] bst;                    // IEC 61937 burst hold (~170 ms)
    reg        na_cs;                  // channel status byte 0 bit 1
    assign     nonaudio = na_cs || (bst != 23'd0);
    // IEC 61937 burst preamble: Pa = 0xF872 (left), Pb = 0x4E1F (right), 16-bit words at the top
    wire       pab = (hl[31:16] == 16'hF872) && (smp[23:8] == 16'h4E1F);
    wire       frame_ok = !sf_bad_d && par_ok && have_l && !l_bad;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid <= 1'b0; l <= 32'd0; r <= 32'd0; locked <= 1'b0; na_cs <= 1'b0;
            err <= 1'b0; cs0 <= 8'd0; cs3 <= 8'd0;
            hl <= 32'd0; have_l <= 1'b0; l_bad <= 1'b0; cs <= 32'd0; csi <= 8'd0; cs_run <= 1'b0;
            cs_ok <= 1'b0; good <= 5'd0; wd <= 16'd0; bst <= 23'd0;
        end else begin
            valid <= 1'b0;
            err <= 1'b0;
            if (wd != 16'hFFFF) wd <= wd + 16'd1;
            // ~1.3 ms without frames, or the grid went stale (rate changed): unlock,
            // and read the channel status again before the next sample
            if (wd == 16'hFFFF || stale) begin locked <= 1'b0; good <= 5'd0; cs_ok <= 1'b0; end
            if (bst != 23'd0) bst <= bst - 23'd1;
            if (sf_d) begin
                if (sf_bad_d || !par_ok) begin
                    err <= 1'b1;
                    if (good > 5'd4) good <= good - 5'd4; else begin good <= 5'd0; locked <= 1'b0; cs_ok <= 1'b0; end
                end
                if (sf_type_d != 2'd3) begin                   // left (B or M)
                    have_l <= 1'b1;
                    l_bad  <= sf_bad_d || !par_ok;
                    if (!sf_bad_d) begin
                        if (par_ok) hl <= {smp, 8'd0};
                        // channel status: bit n of the block from the n-th left subframe
                        if (sf_type_d == 2'd1) begin csi <= 8'd1; cs_run <= 1'b1; cs <= {31'd0, sr[26]}; end
                        else if (cs_run) begin
                            if (csi < 8'd32) cs[csi[4:0]] <= sr[26];
                            if (csi == 8'd31) begin
                                cs0 <= cs[7:0]; cs3 <= {sr[26], cs[30:24]};
                                na_cs <= cs[1];
                                cs_ok <= 1'b1;
                            end
                            csi <= csi + 8'd1;
                        end
                    end else
                        cs_run <= 1'b0;                        // a lost bit: wait for the next block start
                end else begin                                 // right (W): one frame
                    have_l <= 1'b0;
                    if (!sf_bad_d) wd <= 16'd0;
                    if (frame_ok) begin
                        if (good != 5'd31) good <= good + 5'd1;
                        if (good >= 5'd16) locked <= 1'b1;
                        l <= hl;
                        r <= {smp, 8'd0};
                        if (pab) bst <= {23{1'b1}};
                    end
                    // any error: l / r keep the previous frame (same frame count)
                    valid <= locked && cs_ok && !na_cs && bst == 23'd0 && !(frame_ok && pab);
                end
            end
        end
    end
endmodule
