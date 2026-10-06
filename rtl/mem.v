`timescale 1ns/1ps
// Inferred block memories (Gowin BSRAM / SDPB, registered read port).
// Read-during-write to the same address returns the OLD word; the
// datapaths never rely on that case.

// Simple dual-port RAM, one clock.
module sdp_ram #(
    parameter W  = 48,
    parameter AW = 7
) (
    input  wire          clk,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [W-1:0]  wdata,
    input  wire [AW-1:0] raddr,
    output reg  [W-1:0]  rdata
);
    reg [W-1:0] mem [0:(1<<AW)-1];
    integer i;
    // Simulation-only clear (silicon power-up content is don't-care:
    // every history is refilled before the output is unmuted).
    initial begin
        for (i = 0; i < (1 << AW); i = i + 1)
            mem[i] = {W{1'b0}};
        rdata = {W{1'b0}};
    end
    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule

// ROM initialised from a hex file, registered read.
// Gowin IDE: the FILE path is resolved relative to the project; keep the
// coef/ directory next to the .gprj (see README).
module rom #(
    parameter W     = 25,
    parameter AW    = 8,
    parameter DEPTH = 256,
    parameter FILE  = "coef/hd_b.hex"
) (
    input  wire          clk,
    input  wire [AW-1:0] addr,
    output reg  [W-1:0]  data
);
    reg [W-1:0] mem [0:DEPTH-1];
    initial begin
        $readmemh(FILE, mem);
        data = {W{1'b0}};
    end
    always @(posedge clk)
        data <= mem[addr];
endmodule
