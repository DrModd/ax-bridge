`timescale 1ns/1ps
// Room-correction / tone EQ: up to NS biquads per channel at the fixed
// output rate (192 kHz), between the ASRC and the I2S transmitter.
//
//   y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2       (Direct Form I)
//
// Data S32, coefficients signed 32 bit Q3.29 (2^29 = 1.0, |c| < 4).
// The 67-bit sum is floored to S32; the dropped 29 bits are fed back
// with (2 z^-1 - z^-2) (2nd-order error feedback), so the rounding noise
// reaches the output through (1 - z^-1)^2 / A(z): no noise build-up in
// low-frequency sections with poles close to z = 1 (20 Hz at 192 kHz).
// Each section output saturates to S32 (clip pulse).
//
// One shared 32x32 multiplier. A frame (start pulse) runs 2*NS jobs:
// section 0 L, section 0 R, section 1 L, ... 9 MCLK each, 180 MCLK for
// NS = 10, inside the 256-MCLK frame. Section states live in a rotating
// chain of 2*NS entries: the head is always the job being computed.
//
// Coefficients: two banks of 128 words in one BSRAM. The host writes the
// inactive bank (word index = sec*10 + ch*5 + k, k = b0 b1 b2 a1 a2),
// then requests "apply": at the next frame start the banks swap and the
// new section count takes effect. Sections >= nsec are exact
// pass-throughs (identity, no error feedback), so nsec = 0 is a
// bit-exact bypass whatever the RAM holds.
module eq_biquad #(
    parameter NS = 10
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clr,         // clear the filter states
    input  wire               start,       // frame start: take in_l / in_r
    input  wire signed [31:0] in_l,
    input  wire signed [31:0] in_r,
    output reg  signed [31:0] out_l,
    output reg  signed [31:0] out_r,
    output reg                done,
    output reg                clip,
    // host side
    input  wire               cw_we,       // write a coefficient word
    input  wire [6:0]         cw_idx,
    input  wire [31:0]        cw_data,
    input  wire               apply_req,
    input  wire [3:0]         nsec_new,
    output reg                apply_pend,
    output reg                bank,        // active bank
    output reg  [3:0]         nsec         // active section count
);
    localparam NJ = 2 * NS;
    localparam signed [31:0] ONE = 32'sd536870912;   // 1.0 in Q3.29

    // ---------------- coefficient RAM ----------------
    reg  [6:0]  ca;              // word index of the next issue
    wire [31:0] cdata;
    sdp_ram #(.W(32), .AW(8)) u_coef (
        .clk(clk), .we(cw_we), .waddr({~bank, cw_idx}), .wdata(cw_data),
        .raddr({bank, ca}), .rdata(cdata));

    // ---------------- section state chain ----------------
    reg signed [31:0] sx1 [0:NJ-1];
    reg signed [31:0] sx2 [0:NJ-1];
    reg signed [31:0] sy1 [0:NJ-1];
    reg signed [31:0] sy2 [0:NJ-1];
    reg        [28:0] se1 [0:NJ-1];
    reg        [28:0] se2 [0:NJ-1];

    reg signed [31:0] cur_l, cur_r;   // running sample through the cascade

    // ---------------- sequencer ----------------
    reg        busy;
    reg [4:0]  job;              // 0 .. NJ-1
    reg [3:0]  ph;               // 0..8 within a job
    wire       jch   = job[0];
    wire [3:0] jsec  = job[4:1];
    wire       ident = (jsec >= nsec);
    wire signed [31:0] x0 = jch ? cur_r : cur_l;

    // issue (ph 0..4: k = ph)
    wire       iss = busy && (ph < 4'd5);
    reg signed [31:0] dsel;
    always @(*) begin
        case (ph)
            4'd0:    dsel = x0;
            4'd1:    dsel = sx1[0];
            4'd2:    dsel = sx2[0];
            4'd3:    dsel = sy1[0];
            default: dsel = sy2[0];
        endcase
    end

    // pipeline
    reg               o_v, o_first, o_sub, o_id, o_k0;   // operand stage
    reg signed [31:0] o_d;
    reg               p_v, p_first, p_sub;               // product stage
    reg signed [63:0] prod;
    reg signed [66:0] acc;
    wire signed [31:0] o_c = o_id ? (o_k0 ? ONE : 32'sd0) : $signed(cdata);
    wire signed [66:0] ef  = ident ? 67'sd0 :
                             ($signed({38'd0, se1[0]}) <<< 1) - $signed({38'd0, se2[0]});
    wire signed [66:0] pe  = p_sub ? -{{3{prod[63]}}, prod} : {{3{prod[63]}}, prod};

    // quantiser (ph 8): acc holds the complete sum
    wire signed [37:0] yq  = acc[66:29];
    wire               yhi = (yq > 38'sd2147483647);
    wire               ylo = (yq < -38'sd2147483648);
    wire signed [31:0] ys  = yhi ? 32'sd2147483647 : ylo ? -32'sd2147483648 : yq[31:0];
    wire        [28:0] ye  = (yhi | ylo | ident) ? 29'd0 : acc[28:0];

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; job <= 5'd0; ph <= 4'd0; ca <= 7'd0;
            cur_l <= 32'sd0; cur_r <= 32'sd0;
            out_l <= 32'sd0; out_r <= 32'sd0; done <= 1'b0; clip <= 1'b0;
            o_v <= 1'b0; o_first <= 1'b0; o_sub <= 1'b0; o_id <= 1'b0; o_k0 <= 1'b0;
            o_d <= 32'sd0; p_v <= 1'b0; p_first <= 1'b0; p_sub <= 1'b0;
            prod <= 64'sd0; acc <= 67'sd0;
            apply_pend <= 1'b0; bank <= 1'b0; nsec <= 4'd0;
        end else begin
            done <= 1'b0;
            clip <= 1'b0;
            if (apply_req) apply_pend <= 1'b1;

            if (start && !busy) begin
                if (apply_pend || apply_req) begin
                    bank <= ~bank;
                    nsec <= (nsec_new > NS) ? NS[3:0] : nsec_new;
                    apply_pend <= 1'b0;
                end
                cur_l <= in_l; cur_r <= in_r;
                busy <= 1'b1; job <= 5'd0; ph <= 4'd0; ca <= 7'd0;
            end else if (busy) begin
                if (iss) ca <= ca + 7'd1;
                if (ph == 4'd8) begin
                    // job result: next sample in the cascade, rotate chain
                    if (jch) cur_r <= ys; else cur_l <= ys;
                    if (yhi | ylo) clip <= 1'b1;
                    ph <= 4'd0;
                    if (job == NJ - 1) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        out_l <= cur_l;          // L finished one job earlier
                        out_r <= ys;
                    end else
                        job <= job + 5'd1;
                end else
                    ph <= ph + 4'd1;
            end

            // operand stage (RAM data arrives with it)
            o_v <= iss;
            o_first <= iss && (ph == 4'd0);
            o_sub <= iss && (ph >= 4'd3);
            o_id <= ident;
            o_k0 <= (ph == 4'd0);
            o_d <= dsel;
            // product
            p_v <= o_v; p_first <= o_first; p_sub <= o_sub;
            prod <= o_d * o_c;
            // accumulate
            if (p_v)
                acc <= p_first ? ef + pe : acc + pe;

            if (clr) begin
                cur_l <= 32'sd0; cur_r <= 32'sd0;
                out_l <= 32'sd0; out_r <= 32'sd0;
                busy <= 1'b0; ph <= 4'd0; job <= 5'd0;
            end
        end
    end

    // Section state chain: plain registers with a synchronous clear and a
    // clock enable (no asynchronous reset), so clr and the rotate map onto
    // the flip-flops' own reset / enable pins instead of ~3.7k LUTs.
    // Power-up content is cleared by the top level (clr is held while idle);
    // the initial block only keeps simulation free of X.
    wire rot = busy && (ph == 4'd8);
    integer c;
    initial
        for (c = 0; c < NJ; c = c + 1) begin
            sx1[c] = 32'sd0; sx2[c] = 32'sd0; sy1[c] = 32'sd0; sy2[c] = 32'sd0;
            se1[c] = 29'd0; se2[c] = 29'd0;
        end
    always @(posedge clk) begin
        if (clr) begin
            for (c = 0; c < NJ; c = c + 1) begin
                sx1[c] <= 32'sd0; sx2[c] <= 32'sd0; sy1[c] <= 32'sd0; sy2[c] <= 32'sd0;
                se1[c] <= 29'd0; se2[c] <= 29'd0;
            end
        end else if (rot) begin
            for (c = 0; c < NJ - 1; c = c + 1) begin
                sx1[c] <= sx1[c + 1]; sx2[c] <= sx2[c + 1];
                sy1[c] <= sy1[c + 1]; sy2[c] <= sy2[c + 1];
                se1[c] <= se1[c + 1]; se2[c] <= se2[c + 1];
            end
            sx1[NJ - 1] <= x0;      sx2[NJ - 1] <= sx1[0];
            sy1[NJ - 1] <= ys;      sy2[NJ - 1] <= sy1[0];
            se1[NJ - 1] <= ye;      se2[NJ - 1] <= se1[0];
        end
    end
endmodule
