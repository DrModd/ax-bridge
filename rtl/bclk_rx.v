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
// rs = 1 matches the transport measured by ppy2 (PureFox): RIGHT on
// SDATA, LEFT on LRCK.
//
// Automatic DSD detection (auto = 1, for transports without a DSD flag,
// e.g. HDMI receivers): in I2S PCM the LRCK pin toggles once per slot,
// i.e. never less than 16 bit clocks apart. In DSD it carries a data
// stream whose runs are a few bits long. 16 LRCK edges closer than 8
// bit clocks within 64 edges -> DSD; 128 edges in a row that are at
// least 16 bit clocks apart -> PCM again.
module bclk_rx (
    input  wire        bclk,
    input  wire        rst_n,      // async assert, sync release (bclk)
    input  wire        lrck,
    input  wire        sdata,
    input  wire        dsd_on,     // asynchronous level (flag pin)
    input  wire        auto,       // asynchronous level: auto-detect DSD
    input  wire        rs,         // asynchronous level: RIGHT on SDATA
    output reg         wr_en,
    output reg  [64:0] wr_data
);
    reg [1:0] dsd_s, auto_s, rs_s;
    always @(posedge bclk or negedge rst_n) begin
        if (!rst_n) begin
            dsd_s <= 2'b00; auto_s <= 2'b00; rs_s <= 2'b00;
        end else begin
            dsd_s <= {dsd_s[0], dsd_on};
            auto_s <= {auto_s[0], auto};
            rs_s <= {rs_s[0], rs};
        end
    end

    // ---------------- DSD auto-detect ----------------
    reg       ad_lq;          // LRCK pin, previous bit clock
    reg [4:0] ad_gap;         // bit clocks since the last LRCK edge (sat 31)
    reg [5:0] ad_edges;       // edges in the current window of 64
    reg [4:0] ad_short;       // short gaps in the window (sat 16)
    reg [7:0] ad_long;        // consecutive long gaps (sat 128)
    reg       ad_dsd;
    always @(posedge bclk or negedge rst_n) begin
        if (!rst_n) begin
            ad_lq <= 1'b0; ad_gap <= 5'd0; ad_edges <= 6'd0; ad_short <= 5'd0;
            ad_long <= 8'd0; ad_dsd <= 1'b0;
        end else if (!auto_s[1]) begin
            ad_lq <= lrck; ad_gap <= 5'd0; ad_edges <= 6'd0; ad_short <= 5'd0;
            ad_long <= 8'd0; ad_dsd <= 1'b0;
        end else begin
            ad_lq <= lrck;
            if (lrck != ad_lq) begin
                ad_gap <= 5'd1;
                ad_edges <= ad_edges + 6'd1;
                if (ad_edges == 6'd63) ad_short <= 5'd0;
                if (ad_gap < 5'd8) begin
                    ad_long <= 8'd0;
                    if (ad_edges != 6'd63 && ad_short != 5'd16) ad_short <= ad_short + 5'd1;
                end else if (ad_gap >= 5'd16) begin
                    if (ad_long != 8'd128) ad_long <= ad_long + 8'd1;
                end else
                    ad_long <= 8'd0;
                if (ad_short == 5'd16) ad_dsd <= 1'b1;
                if (ad_long == 8'd128) ad_dsd <= 1'b0;
            end else if (ad_gap != 5'd31)
                ad_gap <= ad_gap + 5'd1;
        end
    end

    wire dsd_mode = dsd_s[1] | ad_dsd;
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
                    if (rs_s[1])
                        wr_data <= {1'b1, 32'd0, nb, na};
                    else
                        wr_data <= {1'b1, 32'd0, na, nb};
                end
            end
        end
    end
endmodule
