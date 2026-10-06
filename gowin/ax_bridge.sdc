# Local audio clock: 49.152 MHz.
create_clock -name mclk_in -period 20.345 [get_ports {mclk_in}]
# Input bit clocks, worst case PCM 768 kHz x 64 = 49.152 MHz. Only the
# selected one (muxed in fabric) clocks the front end.
create_clock -name bclk0 -period 20.345 [get_ports {i2s_bclk_in}]
create_clock -name bclk1 -period 20.345 [get_ports {in1_bclk}]
create_clock -name bclk2 -period 20.345 [get_ports {in2_bclk}]
create_clock -name bclk3 -period 20.345 [get_ports {in3_bclk}]
# Every domain only meets the local clock in the Gray-coded async FIFO.
set_clock_groups -asynchronous -group [get_clocks {mclk_in}] -group [get_clocks {bclk0}] -group [get_clocks {bclk1}] -group [get_clocks {bclk2}] -group [get_clocks {bclk3}]
