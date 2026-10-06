`timescale 1ns/1ps
// Generic stereo FIR stage, one shared MAC (channels processed in turn).
//   INTERP=1: x2 polyphase interpolator. Per input pair two output pairs:
//             phase 0 uses taps 0,2,4..; phase 1 uses taps 1,3,5..
//             (coefficients carry the gain of 2).
//   INTERP=0: /2 decimator. One output pair per two input pairs.
// N odd. Coefficients: signed 25-bit, 2^23 = 1.0 (coef/*.hex).
// Samples S24; result rounded half-up and saturated to S24.
// History in a BSRAM ring of 2^AW pairs (2^AW >= taps per output).
// Input queue: 4 pairs (throughput is checked by the benches: every
// stage is idle long before its next input at the highest rate it sees).
module fir_stage #(
    parameter N      = 31,
    parameter INTERP = 0,
    parameter AW     = 6,
    parameter CAW    = 5,
    parameter FILE   = "coef/hd_b.hex"
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              clr,
    input  wire              in_valid,
    input  wire signed [23:0] in_l,
    input  wire signed [23:0] in_r,
    output reg               out_valid,
    output reg  signed [23:0] out_l,
    output reg  signed [23:0] out_r,
    output reg               ovf
);
    localparam integer T0 = INTERP ? (N + 1) / 2 : N;
    localparam integer T1 = (N - 1) / 2;

    // ---- input queue ----
    reg [47:0] q0, q1, q2, q3;
    reg [2:0]  qn;
    reg        pop;

    // ---- history ring ----
    reg  [AW-1:0] wp;
    reg  [AW-1:0] newest;
    reg           we;
    reg  [47:0]   wdat;
    reg  [AW-1:0] waddr;
    wire [AW-1:0] raddr;
    wire [47:0]   rdata;
    sdp_ram #(.W(48), .AW(AW)) u_hist (
        .clk(clk), .we(we), .waddr(waddr), .wdata(wdat),
        .raddr(raddr), .rdata(rdata)
    );

    // ---- coefficients ----
    wire [CAW-1:0] caddr;
    wire [24:0]    cdata;
    rom #(.W(25), .AW(CAW), .DEPTH(N), .FILE(FILE)) u_coef (
        .clk(clk), .addr(caddr), .data(cdata)
    );

    // ---- control ----
    localparam S_IDLE = 2'd0, S_WAIT = 2'd1, S_ISSUE = 2'd2, S_DRAIN = 2'd3;
    reg [1:0]  st;
    reg        ph;          // interp phase / decim input parity
    reg        ch;          // channel being issued (0 = L)
    reg [7:0]  t;           // tap index within the phase
    wire [7:0] tlast = (INTERP && ph) ? (T1 - 1) : (T0 - 1);

    assign raddr = newest - t[AW-1:0];
    assign caddr = INTERP ? ({t[CAW-2:0], 1'b0} | {{(CAW-1){1'b0}}, ph}) : t[CAW-1:0];

    // ---- MAC pipeline ----
    reg        v1, f1, l1, c1;
    reg        v2, f2, l2, c2;
    reg signed [48:0] prod;
    reg signed [55:0] acc;
    wire signed [23:0] xs = c1 ? $signed(rdata[23:0]) : $signed(rdata[47:24]);
    reg        fin, finc;   // result in acc (registered rounding stage)
    wire signed [55:0] acc_n = f2 ? {{7{prod[48]}}, prod} : acc + {{7{prod[48]}}, prod};
    wire signed [55:0] acc_r = acc + 56'sd4194304;            // +0.5 LSB
    wire signed [32:0] y_sh  = acc_r[55:23];
    wire signed [23:0] y_sat = (y_sh > 33'sd8388607)  ? 24'sd8388607 :
                               (y_sh < -33'sd8388608) ? -24'sd8388608 :
                               y_sh[23:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q0 <= 48'd0; q1 <= 48'd0; q2 <= 48'd0; q3 <= 48'd0; qn <= 3'd0;
            pop <= 1'b0;
            wp <= {AW{1'b0}}; newest <= {AW{1'b0}};
            we <= 1'b0; wdat <= 48'd0; waddr <= {AW{1'b0}};
            st <= S_IDLE; ph <= 1'b0; ch <= 1'b0; t <= 8'd0;
            v1 <= 1'b0; f1 <= 1'b0; l1 <= 1'b0; c1 <= 1'b0;
            v2 <= 1'b0; f2 <= 1'b0; l2 <= 1'b0; c2 <= 1'b0;
            prod <= 49'sd0; acc <= 56'sd0; fin <= 1'b0; finc <= 1'b0;
            out_valid <= 1'b0; out_l <= 24'sd0; out_r <= 24'sd0; ovf <= 1'b0;
        end else if (clr) begin
            qn <= 3'd0; st <= S_IDLE; ph <= 1'b0; we <= 1'b0;
            v1 <= 1'b0; v2 <= 1'b0; fin <= 1'b0; out_valid <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            we <= 1'b0;
            // ---- queue ----
            pop <= 1'b0;
            case ({in_valid, (st == S_IDLE) && (qn != 3'd0)})
                2'b10: begin
                    case (qn)
                        3'd0: q0 <= {in_l, in_r};
                        3'd1: q1 <= {in_l, in_r};
                        3'd2: q2 <= {in_l, in_r};
                        3'd3: q3 <= {in_l, in_r};
                        default: ovf <= 1'b1;
                    endcase
                    if (qn != 3'd4) qn <= qn + 3'd1;
                end
                2'b01: begin
                    q0 <= q1; q1 <= q2; q2 <= q3; qn <= qn - 3'd1;
                end
                2'b11: begin
                    q0 <= q1; q1 <= q2; q2 <= q3;
                    case (qn)
                        3'd1: q0 <= {in_l, in_r};
                        3'd2: q1 <= {in_l, in_r};
                        3'd3: q2 <= {in_l, in_r};
                        default: q3 <= {in_l, in_r};
                    endcase
                end
                default: ;
            endcase

            // ---- control ----
            case (st)
                S_IDLE: if (qn != 3'd0) begin
                    we <= 1'b1;
                    waddr <= wp;
                    wdat <= q0;
                    newest <= wp;
                    wp <= wp + 1'b1;
                    if (INTERP) begin
                        ph <= 1'b0;
                        st <= S_WAIT;
                    end else begin
                        ph <= ~ph;
                        if (ph) st <= S_WAIT;     // every 2nd input
                    end
                end
                S_WAIT: begin                    // RAM write lands
                    ch <= 1'b0; t <= 8'd0;
                    st <= S_ISSUE;
                end
                S_ISSUE: begin
                    if (t == tlast) begin
                        t <= 8'd0;
                        if (ch) begin
                            ch <= 1'b0;
                            st <= S_DRAIN;
                        end else begin
                            ch <= 1'b1;
                        end
                    end else begin
                        t <= t + 8'd1;
                    end
                end
                S_DRAIN: if (!v1 && !v2 && !fin) begin
                    if (INTERP && !ph) begin
                        ph <= 1'b1;
                        st <= S_ISSUE;
                    end else begin
                        st <= S_IDLE;
                    end
                end
                default: st <= S_IDLE;
            endcase

            // ---- pipeline ----
            v1 <= (st == S_ISSUE);
            f1 <= (st == S_ISSUE) && (t == 8'd0);
            l1 <= (st == S_ISSUE) && (t == tlast);
            c1 <= ch;
            v2 <= v1; f2 <= f1; l2 <= l1; c2 <= c1;
            prod <= xs * $signed(cdata);
            if (v2)
                acc <= acc_n;
            fin <= v2 && l2;
            finc <= c2;
            if (fin) begin
                if (!finc) out_l <= y_sat;
                else begin
                    out_r <= y_sat;
                    out_valid <= 1'b1;
                end
            end
        end
    end

endmodule
