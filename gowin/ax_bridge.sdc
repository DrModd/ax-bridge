# Local audio clock: 49.152 MHz.
create_clock -name mclk_in -period 20.345 [get_ports {mclk_in}]

# Front-end clock: the selected input BCLK after the 4:1 mux (in_mux), net
# m_bclk in top.v. bclk_rx and the FIFO write side run on it. Worst case
# PCM 768 kHz x 64 = 49.152 MHz. The four BCLK pins only feed the mux, so
# this one clock covers every input.
create_clock -name m_bclk -period 20.345 [get_nets {m_bclk}]

# The front end only meets the local clock in the Gray-coded async FIFO
# (and the activity detector / reset synchronisers, all 2-FF synchronised).
set_clock_groups -asynchronous -group [get_clocks {mclk_in}] -group [get_clocks {m_bclk}]
