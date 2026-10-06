`timescale 1ns/1ps
// Asynchronous sample-rate converter: intermediate stream (176.4/192 kHz
// from the input clock) -> 192 kHz on the local MCLK grid (MCLK/256).
//
// Ring: 128 stereo S24 pairs, write counter wcnt (16-bit, wraps).
// Read position phi (Q16.32, same wrap) is driven by a digital PLL:
//   * NCO: phi += s at every output frame (s ~ f_int / 192k, Q2.32)
//   * events: one per 4 intermediate samples (44.1/48 kHz). At an event
//     (dt MCLK after the last frame start) the expected position is T;
//       e = T - (phi + s*dt/256)
//       v += (e - v) >> a          (error low-pass, 3rd-order loop)
//       phi += v >> kp;  s += v >> ki;  T += 4
//   * gear shifting from ~200 Hz (acquisition) down to ~0.3 Hz.
// Output sample at frame m uses p = phi - D (D = 40 samples of latency):
//   y = sum_{j=0..39} x[i-19+j] * h(mu + 19 - j),  p = i + mu
// h: Kaiser-windowed sinc, 256 phases/sample, symmetric half table
// (coef/asrc.hex: h[a] | h[a+1] per word), coefficient linearly
// interpolated with 12 fraction bits. Result S32 (S24 data << 8 plus the
// extra fraction bits of the filter).
module asrc #(
    parameter D    = 40,
    parameter FILL = 72,
    parameter GSH  = 0,              // gear length >> GSH (sim speed-up)
    parameter FILE = "coef/asrc.hex"
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              clr,
    input  wire              fam48,
    input  wire              arm,       // allowed to start the DPLL
    input  wire              in_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    input  wire              ev,        // input timing event
    input  wire [7:0]        fpos,      // MCLK position in the frame
    output reg               y_valid,
    output reg  signed [31:0] y_l,
    output reg  signed [31:0] y_r,
    output reg               started,
    output reg  [2:0]        gear,
    output reg               fault,
    output reg  signed [47:0] dbg_e
);
    localparam [33:0] S44 = 34'd3946001203;   // 0.91875 * 2^32
    localparam [33:0] S48 = 34'h1_0000_0000;  // 1.0

    wire tick = (fpos == 8'd0);

    // ---------------- ring ----------------
    reg  [15:0] wcnt;
    wire [6:0]  ring_ra;
    wire [47:0] ring_rd;
    sdp_ram #(.W(48), .AW(7)) u_ring (
        .clk(clk), .we(in_valid), .waddr(wcnt[6:0]), .wdata({in_l, in_r}),
        .raddr(ring_ra), .rdata(ring_rd)
    );

    // ---------------- DPLL ----------------
    reg  [47:0] phi;
    reg  [33:0] s;
    reg  signed [47:0] v;
    reg  [15:0] T;
    reg         evp;          // event pending
    reg  signed [9:0] evdt;   // dt relative to the current frame start
    reg  [1:0]  ast;          // event pipeline: 0 idle, 1 error, 2 filter, 3 update
    reg  signed [43:0] sdt_r;
    reg  signed [47:0] e_r;
    reg  [16:0] gcnt;

    reg [4:0] kp, ki, la;
    reg [16:0] glen;
    always @(*) begin
        case (gear)
            3'd0: begin kp = 5'd5;  ki = 5'd12; la = 5'd0;  glen = 17'd4000;  end
            3'd1: begin kp = 5'd8;  ki = 5'd18; la = 5'd3;  glen = 17'd8000;  end
            3'd2: begin kp = 5'd11; ki = 5'd25; la = 5'd6;  glen = 17'd40000; end
            3'd3: begin kp = 5'd13; ki = 5'd29; la = 5'd9;  glen = 17'd40000; end
            default: begin kp = 5'd14; ki = 5'd31; la = 5'd10; glen = 17'd0; end
        endcase
    end

    // The event pipeline (4 cycles) only starts early in a frame, so no
    // frame tick can fall inside it: phi, s and dt stay consistent and
    // every stage has a single adder / shifter between registers.
    wire a_go = evp && !ev && !tick && (fpos < 8'd240) && (ast == 2'd0);

    // stage 1: phase error at the event
    wire [47:0]        pred = phi + {{12{sdt_r[43]}}, sdt_r[43:8]};
    wire [47:0]        tgt  = {T, 32'd0};
    wire signed [47:0] e_raw = $signed(tgt - pred);
    localparam signed [47:0] EMAX = 48'sh0100_0000_0000;   // 256 samples
    wire signed [47:0] e_sat = (e_raw > EMAX) ? EMAX : (e_raw < -EMAX) ? -EMAX : e_raw;
    wire [47:0] e_abs = e_raw[47] ? -e_raw : e_raw;
    wire init_now = !started && arm && (wcnt >= FILL);

    // stage 3: loop-filter outputs (from the registered v)
    wire signed [47:0] dphi_v = v >>> kp;
    wire signed [47:0] ds_v   = v >>> ki;

    // buffer guard (checked at frame start)
    wire signed [15:0] lead = $signed(phi[47:32] - wcnt);   // phi - wcnt

    reg  [47:0] p;            // latched read position
    reg         eng_go;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wcnt <= 16'd0; phi <= 48'd0; s <= S44; v <= 48'sd0; T <= 16'd0;
            evp <= 1'b0; evdt <= 10'sd0; ast <= 2'd0; sdt_r <= 44'sd0; e_r <= 48'sd0;
            gcnt <= 17'd0; gear <= 3'd0; started <= 1'b0; fault <= 1'b0;
            p <= 48'd0; eng_go <= 1'b0; dbg_e <= 48'sd0;
        end else if (clr) begin
            wcnt <= 16'd0; started <= 1'b0; fault <= 1'b0; gear <= 3'd0;
            evp <= 1'b0; ast <= 2'd0; eng_go <= 1'b0; v <= 48'sd0;
            s <= fam48 ? S48 : S44;
        end else begin
            eng_go <= 1'b0;
            if (in_valid)
                wcnt <= wcnt + 16'd1;

            // ---- event capture / frame bookkeeping of the pending dt ----
            if (ev) begin
                evp <= 1'b1;
                evdt <= {2'b00, fpos};
            end else if (evp && tick) begin
                evdt <= evdt - 10'sd256;
            end

            case (ast)
                2'd0: if (a_go) begin            // s * dt
                    evp <= 1'b0;
                    sdt_r <= $signed({1'b0, s}) * evdt;
                    ast <= 2'd1;
                end
                2'd1: begin                      // phase error / init
                    ast <= 2'd0;
                    if (started) begin
                        e_r <= e_sat;
                        dbg_e <= e_sat;
                        ast <= 2'd2;
                        if (gear != 3'd0 && e_abs > 48'h2_0000_0000)
                            fault <= 1'b1;
                    end else if (init_now) begin
                        started <= 1'b1;
                        s <= fam48 ? S48 : S44;
                        phi <= {wcnt, 32'd0} - {{12{sdt_r[43]}}, sdt_r[43:8]};
                        T <= wcnt + 16'd4;
                        v <= 48'sd0;
                        gear <= 3'd0;
                        gcnt <= 17'd0;
                    end
                end
                2'd2: begin                      // error low-pass
                    v <= v + ((e_r - v) >>> la);
                    ast <= 2'd3;
                end
                default: begin                   // phase / frequency update
                    s <= s + ds_v[33:0];
                    T <= T + 16'd4;
                    if (gear != 3'd4) begin
                        if (gcnt >= (glen >> GSH)) begin
                            gcnt <= 17'd0;
                            gear <= gear + 3'd1;
                        end else begin
                            gcnt <= gcnt + 17'd1;
                        end
                    end
                    ast <= 2'd0;
                end
            endcase

            // ---- NCO (frame start) + phase correction ----
            if (ast == 2'd3)
                phi <= phi + dphi_v + ((started && tick) ? {14'd0, s} : 48'd0);
            else if (started && tick)
                phi <= phi + {14'd0, s};
            if (started && tick) begin
                p <= phi - {D[15:0], 32'd0};
                eng_go <= 1'b1;
                if ((lead > 16'sd18) || (lead < -16'sd60))
                    fault <= 1'b1;
            end
        end
    end

    // ---------------- interpolation engine ----------------
    wire [15:0] pi_  = p[47:32];
    wire [19:0] mu20 = p[31:12];
    reg  [5:0]  j;
    reg         ech;
    reg         issuing;
    wire [5:0]  jm   = 6'd19 - j;        // valid when j <= 19
    wire [5:0]  jp   = j - 6'd20;        // valid when j >= 20
    wire [20:0] m21  = 21'h10_0000 - {1'b0, mu20};
    wire [25:0] ufx  = (j <= 6'd19) ? ({jm[4:0], 20'd0} + {6'd0, mu20})
                                    : ({jp[4:0], 20'd0} + {5'd0, m21});
    wire [12:0] ka   = ufx[24:12];
    wire [11:0] kfr  = ufx[11:0];
    assign ring_ra   = pi_[6:0] - 7'd19 + {1'b0, j};

    wire [49:0] kd;
    rom #(.W(50), .AW(13), .DEPTH(5121), .FILE(FILE)) u_kern (
        .clk(clk), .addr(ka), .data(kd)
    );

    reg        a1v, a1f, a1l, a1c;
    reg [11:0] a1fr;
    reg        a2v, a2f, a2l, a2c;
    reg signed [25:0] coef2;
    reg signed [23:0] x2;
    reg        a3v, a3f, a3l, a3c;
    reg signed [49:0] prod3;
    reg signed [55:0] acc;

    wire signed [24:0] h0 = kd[49:25];
    wire signed [24:0] h1 = kd[24:0];
    wire signed [25:0] hd = $signed({h1[24], h1}) - $signed({h0[24], h0});
    wire signed [38:0] hm = hd * $signed({1'b0, a1fr});
    wire signed [25:0] cnew = $signed({h0[24], h0}) + $signed(hm[37:12]);
    wire signed [23:0] xsel = a1c ? $signed(ring_rd[23:0]) : $signed(ring_rd[47:24]);

    reg        fin, finc;
    wire signed [55:0] acc_n = a3f ? {{6{prod3[49]}}, prod3} : acc + {{6{prod3[49]}}, prod3};
    wire signed [55:0] acc_r = acc + 56'sd16384;             // +0.5 LSB of >>15
    wire signed [40:0] ysh   = acc_r[55:15];
    wire signed [31:0] ysat  = (ysh > 41'sd2147483647)  ? 32'sh7FFF_FFFF :
                               (ysh < -41'sd2147483648) ? 32'sh8000_0000 :
                               ysh[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            j <= 6'd0; ech <= 1'b0; issuing <= 1'b0;
            a1v <= 1'b0; a1f <= 1'b0; a1l <= 1'b0; a1c <= 1'b0; a1fr <= 12'd0;
            a2v <= 1'b0; a2f <= 1'b0; a2l <= 1'b0; a2c <= 1'b0;
            coef2 <= 26'sd0; x2 <= 24'sd0;
            a3v <= 1'b0; a3f <= 1'b0; a3l <= 1'b0; a3c <= 1'b0;
            prod3 <= 50'sd0; acc <= 56'sd0; fin <= 1'b0; finc <= 1'b0;
            y_valid <= 1'b0; y_l <= 32'sd0; y_r <= 32'sd0;
        end else begin
            y_valid <= 1'b0;
            if (eng_go) begin
                issuing <= 1'b1; j <= 6'd0; ech <= 1'b0;
            end else if (issuing) begin
                if (j == 6'd39) begin
                    j <= 6'd0;
                    if (ech) issuing <= 1'b0;
                    ech <= ~ech;
                end else begin
                    j <= j + 6'd1;
                end
            end
            // stage 1: RAM/ROM outputs valid
            a1v <= issuing; a1f <= issuing && (j == 6'd0);
            a1l <= issuing && (j == 6'd39); a1c <= ech; a1fr <= kfr;
            // stage 2: coefficient, sample
            a2v <= a1v; a2f <= a1f; a2l <= a1l; a2c <= a1c;
            coef2 <= cnew;
            x2 <= xsel;
            // stage 3: product
            a3v <= a2v; a3f <= a2f; a3l <= a2l; a3c <= a2c;
            prod3 <= x2 * coef2;
            // stage 4: accumulate; stage 5: round / saturate
            if (a3v)
                acc <= acc_n;
            fin <= a3v && a3l;
            finc <= a3c;
            if (fin) begin
                if (!finc) y_l <= ysat;
                else begin
                    y_r <= ysat;
                    y_valid <= 1'b1;
                end
            end
        end
    end
endmodule
