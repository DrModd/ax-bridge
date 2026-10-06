`timescale 1ns/1ps
// DoP (DSD over PCM, v1.1) detector on received PCM frames.
// A DoP frame carries the marker 0x05 / 0xFA (alternating per frame) in
// the top byte of both 24-bit words and 16 DSD bits below it.
// 'dop' sets after 32 consecutive valid frames and clears on the first
// invalid one (the controller mutes on any change of this flag).
module dop_det (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame,       // PCM frame strobe
    input  wire [31:0] l,
    input  wire [31:0] r,
    output reg         dop
);
    wire [7:0] ml = l[31:24];
    wire [7:0] mr = r[31:24];
    reg  [7:0] prev;
    reg  [5:0] good;
    wire valid = (ml == mr) && ((ml == 8'h05) || (ml == 8'hFA)) && (ml != prev);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev <= 8'd0; good <= 6'd0; dop <= 1'b0;
        end else if (frame) begin
            prev <= ml;
            if (valid) begin
                if (good != 6'd32)
                    good <= good + 6'd1;
                else
                    dop <= 1'b1;
            end else begin
                good <= 6'd0;
                dop <= 1'b0;
            end
        end
    end
endmodule
