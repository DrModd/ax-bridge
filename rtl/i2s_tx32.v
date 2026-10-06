`timescale 1ns/1ps
// I2S master transmitter for the AX5689: Philips format, 32-bit slots,
// 192 kHz from the local 49.152 MHz MCLK.
//   BCLK = MCLK/4 = 12.288 MHz (64 fs), LRCK = MCLK/256 = 192 kHz.
//   LRCK low = left. Data changes on BCLK falling edges, MSB one BCLK
//   after the LRCK edge (the right word's LSB lands in slot 0 of the
//   next frame). All three outputs come from flip-flops in the same
//   cycle, so they are mutually aligned.
// fpos is the MCLK index inside the frame (0 = frame start); the samples
// are taken from in_l/in_r at fpos == 0.
module i2s_tx32 (
    input  wire              clk,
    input  wire              rst_n,
    input  wire signed [31:0] in_l,
    input  wire signed [31:0] in_r,
    output reg  [7:0]        fpos,
    output reg               bclk_out,
    output reg               lrck_out,
    output reg               sdata_out
);
    reg [31:0] hl, hr;
    wire [5:0] b = fpos[7:2];   // bit slot 0..63

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fpos <= 8'd0; hl <= 32'd0; hr <= 32'd0;
            bclk_out <= 1'b0; lrck_out <= 1'b0; sdata_out <= 1'b0;
        end else begin
            fpos <= fpos + 8'd1;
            if (fpos == 8'd0) begin
                hl <= in_l;
                hr <= in_r;
            end
            bclk_out <= fpos[1];
            if (fpos[1:0] == 2'd0) begin
                // BCLK falling edge
                lrck_out <= b[5];
                if (b == 6'd0)
                    sdata_out <= hr[0];          // previous right LSB
                else if (b <= 6'd32)
                    sdata_out <= hl[6'd32 - b];
                else
                    sdata_out <= hr[7'd64 - {1'b0, b}];
            end
        end
    end
endmodule
