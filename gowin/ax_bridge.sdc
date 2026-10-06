# Local audio clock: 49.152 MHz.
create_clock -name mclk_in -period 20.345 [get_ports {mclk_in}]
# Transport bit clock, worst case PCM 768 kHz x 64 = 49.152 MHz.
create_clock -name bclk_in -period 20.345 [get_ports {i2s_bclk_in}]
# The two domains only meet in the Gray-coded async FIFO.
set_clock_groups -asynchronous -group [get_clocks {mclk_in}] -group [get_clocks {bclk_in}]
