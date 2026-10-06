`timescale 1ns/1ps
// I2C slave (7-bit address), standard / fast mode, oversampled on the
// local 49.152 MHz clock. Register access with an auto-incrementing
// pointer:
//   write: S addr+W  ptr  [data data ...]  P
//   read : S addr+W  ptr  Sr addr+R  [data ... NACK]  P
// SCL / SDA pass a 3-FF synchronizer and a 6-sample (~120 ns) glitch
// filter. SDA is driven only low (open drain); the slave never stretches
// SCL.
module i2c_slave #(
    parameter [6:0] ADDR = 7'h5A
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       scl_in,
    input  wire       sda_in,
    output reg        sda_oe,      // 1 = pull SDA low
    output reg  [7:0] ptr,         // register pointer (read address)
    input  wire [7:0] rdata,       // register at ptr (combinational)
    output reg        wr,          // write strobe
    output reg  [7:0] waddr,
    output reg  [7:0] wdata
);
    // ---- input conditioning ----
    reg [2:0] scl_s, sda_s;
    reg [2:0] scl_c, sda_c;
    reg       scl_f, sda_f, scl_q, sda_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_s <= 3'b111; sda_s <= 3'b111;
            scl_c <= 3'd0; sda_c <= 3'd0;
            scl_f <= 1'b1; sda_f <= 1'b1; scl_q <= 1'b1; sda_q <= 1'b1;
        end else begin
            scl_s <= {scl_s[1:0], scl_in};
            sda_s <= {sda_s[1:0], sda_in};
            if (scl_s[2] == scl_f) scl_c <= 3'd0;
            else if (scl_c == 3'd5) begin scl_f <= scl_s[2]; scl_c <= 3'd0; end
            else scl_c <= scl_c + 3'd1;
            if (sda_s[2] == sda_f) sda_c <= 3'd0;
            else if (sda_c == 3'd5) begin sda_f <= sda_s[2]; sda_c <= 3'd0; end
            else sda_c <= sda_c + 3'd1;
            scl_q <= scl_f;
            sda_q <= sda_f;
        end
    end
    wire scl_rise = scl_f & ~scl_q;
    wire scl_fall = ~scl_f & scl_q;
    wire start    = scl_f & scl_q & sda_q & ~sda_f;
    wire stop     = scl_f & scl_q & ~sda_q & sda_f;

    // ---- protocol ----
    localparam S_IDLE = 3'd0, S_ADDR = 3'd1, S_AACK = 3'd2, S_WR = 3'd3,
               S_WACK = 3'd4, S_RD = 3'd5, S_RACK = 3'd6, S_RLOAD = 3'd7;
    reg [2:0] st;
    reg [3:0] bitn;
    reg [7:0] sh;
    reg       rw;
    reg       first;      // next written byte is the pointer
    reg       mack;       // master ACKed the byte just read
    wire [7:0] sh_n = {sh[6:0], sda_f};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; bitn <= 4'd0; sh <= 8'd0; rw <= 1'b0; first <= 1'b0;
            mack <= 1'b0; sda_oe <= 1'b0; ptr <= 8'd0;
            wr <= 1'b0; waddr <= 8'd0; wdata <= 8'd0;
        end else begin
            wr <= 1'b0;
            if (start) begin
                st <= S_ADDR; bitn <= 4'd0; sda_oe <= 1'b0;
            end else if (stop) begin
                st <= S_IDLE; sda_oe <= 1'b0;
            end else begin
                case (st)
                    S_ADDR: begin
                        if (scl_rise) begin
                            sh <= sh_n;
                            bitn <= bitn + 4'd1;
                        end
                        if (scl_fall && bitn == 4'd8) begin
                            if (sh[7:1] == ADDR) begin
                                rw <= sh[0];
                                sda_oe <= 1'b1;          // ACK
                                st <= S_AACK;
                            end else begin
                                st <= S_IDLE;
                            end
                        end
                    end
                    S_AACK: if (scl_fall) begin
                        bitn <= 4'd0;
                        if (rw) begin
                            sh <= rdata;
                            sda_oe <= ~rdata[7];
                            st <= S_RD;
                        end else begin
                            sda_oe <= 1'b0;
                            first <= 1'b1;
                            st <= S_WR;
                        end
                    end
                    S_WR: begin
                        if (scl_rise) begin
                            sh <= sh_n;
                            bitn <= bitn + 4'd1;
                        end
                        if (scl_fall && bitn == 4'd8) begin
                            sda_oe <= 1'b1;              // ACK
                            if (first) begin
                                ptr <= sh;
                                first <= 1'b0;
                            end else begin
                                wr <= 1'b1;
                                waddr <= ptr;
                                wdata <= sh;
                                ptr <= ptr + 8'd1;
                            end
                            st <= S_WACK;
                        end
                    end
                    S_WACK: if (scl_fall) begin
                        sda_oe <= 1'b0;
                        bitn <= 4'd0;
                        st <= S_WR;
                    end
                    S_RD: begin
                        // bit 7 is on the line; shift on each falling edge
                        if (scl_rise)
                            bitn <= bitn + 4'd1;
                        if (scl_fall) begin
                            if (bitn == 4'd8) begin
                                sda_oe <= 1'b0;          // master ACK slot
                                st <= S_RACK;
                            end else begin
                                sh <= {sh[6:0], 1'b1};
                                sda_oe <= ~sh[6];
                            end
                        end
                    end
                    S_RACK: begin
                        if (scl_rise)
                            mack <= ~sda_f;
                        if (scl_fall) begin
                            if (mack) begin
                                ptr <= ptr + 8'd1;
                                bitn <= 4'd0;
                                st <= S_RLOAD;
                            end else begin
                                st <= S_IDLE;
                            end
                        end
                    end
                    S_RLOAD: begin                       // rdata at ptr+1
                        sh <= rdata;
                        sda_oe <= ~rdata[7];
                        st <= S_RD;
                    end
                    default: st <= S_IDLE;
                endcase
            end
        end
    end
endmodule
