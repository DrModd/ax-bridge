`timescale 1ns/1ps
// DSD -> PCM first stage: 5th-order CIC decimator, both channels.
// Input: 16-bit DSD chunks per channel (MSB = earliest bit), from native
// DSD or DoP. Bits are consumed one per MCLK (17 cycles per chunk; the
// densest stream, DSD512, delivers a chunk every ~35 MCLK).
// Decimation R = 2^r_log2 (8..64) brings every DSD rate to 352.8 kHz
// (48-family DSD: 384 kHz). Bit 1 = +1, bit 0 = -1. CIC gain R^5 is an
// exact power of two, normalised to S24 full scale (+1.0 saturates to
// +FS - 1 LSB). Response: droop -0.33 dB @24 kHz (compensated by the
// following FIR), image rejection > 120 dB around multiples of 352.8 k.
//
// Timing: the integrators are pipelined (each adds the PREVIOUS value of
// the stage before it - same transfer function, 4 samples of delay) and
// the comb section runs one stage per MCLK after each decimation point,
// so no path has more than one 32-bit adder.
module cic5_dsd (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              clr,        // sync clear
    input  wire [2:0]        r_log2,     // 3..6
    input  wire              in_valid,
    input  wire [15:0]       in_l,
    input  wire [15:0]       in_r,
    output reg               out_valid,
    output reg signed [23:0] out_l,
    output reg signed [23:0] out_r,
    output reg               ovf        // input queue overflow (sticky)
);
    // 4-deep input queue
    reg [31:0] q0, q1, q2, q3;
    reg [2:0]  qn;
    reg [15:0] sl, sr;      // chunk being serialised
    reg [4:0]  bits;        // bits left in sl/sr
    wire take = (bits == 5'd0) && (qn != 3'd0);

    // integrators, modular 32-bit
    reg [31:0] il1, il2, il3, il4, il5, ir1, ir2, ir3, ir4, ir5;
    // comb delays
    reg [31:0] dl1, dl2, dl3, dl4, dl5, dr1, dr2, dr3, dr4, dr5;
    reg [31:0] cvl, cvr;    // value travelling through the comb stages
    reg [2:0]  cst;         // 0 idle, 1..5 comb stage, 6 output
    reg        snap;        // decimation point reached last cycle
    reg [6:0]  dec;         // decimation phase
    wire [6:0] rm1 = (7'd1 << r_log2) - 7'd1;

    wire [31:0] xl = sl[15] ? 32'd1 : 32'hFFFF_FFFF;
    wire [31:0] xr = sr[15] ? 32'd1 : 32'hFFFF_FFFF;

    // gain 2^(5*r_log2) -> 2^23
    function signed [23:0] norm;
        input [31:0] v;
        input [2:0]  rl;
        reg signed [39:0] w;
        begin
            case (rl)
                3'd3:    w = $signed({{8{v[31]}}, v}) <<< 8;   // 2^15
                3'd4:    w = $signed({{8{v[31]}}, v}) <<< 3;   // 2^20
                3'd5:    w = $signed({{8{v[31]}}, v}) >>> 2;   // 2^25
                default: w = $signed({{8{v[31]}}, v}) >>> 7;   // 2^30
            endcase
            if (w > 40'sd8388607)        norm = 24'sd8388607;
            else if (w < -40'sd8388608)  norm = -24'sd8388608;
            else                         norm = w[23:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q0 <= 32'd0; q1 <= 32'd0; q2 <= 32'd0; q3 <= 32'd0; qn <= 3'd0;
            sl <= 16'd0; sr <= 16'd0; bits <= 5'd0;
            il1 <= 0; il2 <= 0; il3 <= 0; il4 <= 0; il5 <= 0;
            ir1 <= 0; ir2 <= 0; ir3 <= 0; ir4 <= 0; ir5 <= 0;
            dl1 <= 0; dl2 <= 0; dl3 <= 0; dl4 <= 0; dl5 <= 0;
            dr1 <= 0; dr2 <= 0; dr3 <= 0; dr4 <= 0; dr5 <= 0;
            cvl <= 0; cvr <= 0; cst <= 3'd0; snap <= 1'b0;
            dec <= 7'd0;
            out_valid <= 1'b0; out_l <= 24'sd0; out_r <= 24'sd0; ovf <= 1'b0;
        end else if (clr) begin
            qn <= 3'd0; bits <= 5'd0; dec <= 7'd0; out_valid <= 1'b0;
            cst <= 3'd0; snap <= 1'b0;
            il1 <= 0; il2 <= 0; il3 <= 0; il4 <= 0; il5 <= 0;
            ir1 <= 0; ir2 <= 0; ir3 <= 0; ir4 <= 0; ir5 <= 0;
            dl1 <= 0; dl2 <= 0; dl3 <= 0; dl4 <= 0; dl5 <= 0;
            dr1 <= 0; dr2 <= 0; dr3 <= 0; dr4 <= 0; dr5 <= 0;
        end else begin
            out_valid <= 1'b0;
            snap <= 1'b0;
            // queue: push at tail, pop from head (q0)
            case ({in_valid, take})
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
            if (take) begin
                sl <= q0[31:16];
                sr <= q0[15:0];
                bits <= 5'd16;
            end else if (bits != 5'd0) begin
                // one DSD bit per channel, pipelined integrators
                il1 <= il1 + xl; il2 <= il2 + il1; il3 <= il3 + il2;
                il4 <= il4 + il3; il5 <= il5 + il4;
                ir1 <= ir1 + xr; ir2 <= ir2 + ir1; ir3 <= ir3 + ir2;
                ir4 <= ir4 + ir3; ir5 <= ir5 + ir4;
                sl <= {sl[14:0], 1'b0};
                sr <= {sr[14:0], 1'b0};
                bits <= bits - 5'd1;
                if (dec == rm1) begin
                    dec <= 7'd0;
                    snap <= 1'b1;
                end else begin
                    dec <= dec + 7'd1;
                end
            end
            // comb section, one stage per cycle (>= 8 cycles between snaps)
            case (cst)
                3'd0: if (snap) begin
                    cvl <= il5; cvr <= ir5; cst <= 3'd1;
                end
                3'd1: begin cvl <= cvl - dl1; dl1 <= cvl; cvr <= cvr - dr1; dr1 <= cvr; cst <= 3'd2; end
                3'd2: begin cvl <= cvl - dl2; dl2 <= cvl; cvr <= cvr - dr2; dr2 <= cvr; cst <= 3'd3; end
                3'd3: begin cvl <= cvl - dl3; dl3 <= cvl; cvr <= cvr - dr3; dr3 <= cvr; cst <= 3'd4; end
                3'd4: begin cvl <= cvl - dl4; dl4 <= cvl; cvr <= cvr - dr4; dr4 <= cvr; cst <= 3'd5; end
                3'd5: begin cvl <= cvl - dl5; dl5 <= cvl; cvr <= cvr - dr5; dr5 <= cvr; cst <= 3'd6; end
                default: begin
                    out_valid <= 1'b1;
                    out_l <= norm(cvl, r_log2);
                    out_r <= norm(cvr, r_log2);
                    cst <= 3'd0;
                end
            endcase
        end
    end
endmodule
