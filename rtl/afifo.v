`timescale 1ns/1ps
// Asynchronous FIFO (Gray-coded pointers, 2-FF synchronizers).
// Write side: input BCLK domain. Read side: local MCLK domain.
// Show-ahead read: rdata is the head word whenever !empty.
// Depth 2^AW. The storage is a small register file: a word is written
// at least two read-clock cycles before its pointer becomes visible, so
// the combinational cross-domain read is stable when sampled.
module afifo #(
    parameter W  = 65,
    parameter AW = 4
) (
    input  wire         wclk,
    input  wire         wrst_n,
    input  wire         we,
    input  wire [W-1:0] wdata,
    output wire         full,

    input  wire         rclk,
    input  wire         rrst_n,
    input  wire         re,
    output wire [W-1:0] rdata,
    output wire         empty
);
    reg [W-1:0] mem [0:(1<<AW)-1];

    reg [AW:0] wbin, wgray;
    reg [AW:0] rbin, rgray;
    reg [AW:0] rgray_w1, rgray_w2;   // read pointer seen by the writer
    reg [AW:0] wgray_r1, wgray_r2;   // write pointer seen by the reader

    wire [AW:0] wbin_n  = wbin + {{AW{1'b0}}, (we & ~full)};
    wire [AW:0] wgray_n = (wbin_n >> 1) ^ wbin_n;
    wire [AW:0] rbin_n  = rbin + {{AW{1'b0}}, (re & ~empty)};
    wire [AW:0] rgray_n = (rbin_n >> 1) ^ rbin_n;

    assign full  = (wgray == {~rgray_w2[AW:AW-1], rgray_w2[AW-2:0]});
    assign empty = (rgray == wgray_r2);
    assign rdata = mem[rbin[AW-1:0]];

    always @(posedge wclk) begin
        if (we & ~full)
            mem[wbin[AW-1:0]] <= wdata;
    end

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wbin <= {(AW+1){1'b0}};
            wgray <= {(AW+1){1'b0}};
            rgray_w1 <= {(AW+1){1'b0}};
            rgray_w2 <= {(AW+1){1'b0}};
        end else begin
            wbin <= wbin_n;
            wgray <= wgray_n;
            rgray_w1 <= rgray;
            rgray_w2 <= rgray_w1;
        end
    end

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rbin <= {(AW+1){1'b0}};
            rgray <= {(AW+1){1'b0}};
            wgray_r1 <= {(AW+1){1'b0}};
            wgray_r2 <= {(AW+1){1'b0}};
        end else begin
            rbin <= rbin_n;
            rgray <= rgray_n;
            wgray_r1 <= wgray;
            wgray_r2 <= wgray_r1;
        end
    end
endmodule
