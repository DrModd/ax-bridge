`timescale 1ns/1ps
// Input front end, clocked by the transport's own bit clock (BCLK pin).
// Nothing here depends on the local MCLK, so any input rate works:
// PCM up to 768 kHz x 64 BCLK (49.152 MHz) and native DSD64..DSD512.
//
// PCM (dsd_on low): I2S Philips slave. LRCK low = left. Slots of up to
// 32 bits; the word is left-justified into 32 bits. One FIFO word per
// stereo frame: {tag=0, L32, R32}.
//
// DSD (dsd_on high, Amanero style): BCLK = DSD bit clock, the two data
// streams on the SDATA and LRCK pins, sampled on BCLK rising edges,
// MSB-first (earliest bit at the MSB). One FIFO word per 16 bit-clocks:
// {tag=1, 32'h0, L16, R16}.
// SDATA_IS_RIGHT=1 matches the transport measured by ppy2 (PureFox):
// RIGHT on SDATA, LEFT on LRCK.
module bclk_rx #(
    parameter SDATA_IS_RIGHT = 1
) (
    input  wire        bclk,
    input  wire        rst_n,      // async assert, sync release (bclk)
    input  wire        lrck,
    input  wire        sdata,
    input  wire        dsd_on,     // asynchronous level
    output reg         wr_en,
    output reg  [64:0] wr_data
);
    reg [1:0] dsd_s;
    always @(posedge bclk or negedge rst_n) begin
        if (!rst_n) dsd_s <= 2'b00;
        else        dsd_s <= {dsd_s[0], dsd_on};
    end
    wire dsd_mode = dsd_s[1];
    reg  dsd_mode_q;

    // ---------------- PCM ----------------
    reg        lr_q;
    reg [30:0] sr;
    reg [4:0]  cnt;          // bits collected in the current slot (sat 31)
    reg [31:0] word_l;
    wire [31:0] cat = {sr, sdata};
    wire [31:0] word_new = cat << (5'd31 - cnt);

    // ---------------- DSD ----------------
    reg [15:0] sh_a, sh_b;   // a = SDATA pin, b = LRCK pin
    reg [3:0]  dcnt;
    wire [15:0] na = {sh_a[14:0], sdata};
    wire [15:0] nb = {sh_b[14:0], lrck};

    always @(posedge bclk or negedge rst_n) begin
        if (!rst_n) begin
            dsd_mode_q <= 1'b0;
            lr_q <= 1'b0; sr <= 31'd0; cnt <= 5'd0; word_l <= 32'd0;
            sh_a <= 16'd0; sh_b <= 16'd0; dcnt <= 4'd0;
            wr_en <= 1'b0; wr_data <= 65'd0;
        end else begin
            wr_en <= 1'b0;
            dsd_mode_q <= dsd_mode;
            if (dsd_mode != dsd_mode_q) begin
                // mode flip: restart both assemblers
                cnt <= 5'd0; sr <= 31'd0; dcnt <= 4'd0;
                lr_q <= lrck;
            end else if (!dsd_mode) begin
                lr_q <= lrck;
                if (lrck != lr_q) begin
                    // the bit sampled now is the LSB of the slot that ended
                    if (lr_q == 1'b0) begin
                        word_l <= word_new;
                    end else begin
                        wr_en <= 1'b1;
                        wr_data <= {1'b0, word_l, word_new};
                    end
                    sr <= 31'd0;
                    cnt <= 5'd0;
                end else begin
                    sr <= {sr[29:0], sdata};
                    if (cnt != 5'd31)
                        cnt <= cnt + 5'd1;
                end
            end else begin
                sh_a <= na;
                sh_b <= nb;
                dcnt <= dcnt + 4'd1;
                if (dcnt == 4'd15) begin
                    wr_en <= 1'b1;
                    if (SDATA_IS_RIGHT)
                        wr_data <= {1'b1, 32'd0, nb, na};
                    else
                        wr_data <= {1'b1, 32'd0, na, nb};
                end
            end
        end
    end
endmodule
