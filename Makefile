SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c

RTL := rtl/mem.v rtl/afifo.v rtl/in_mux.v rtl/bclk_rx.v rtl/rate_mon.v rtl/dop_det.v \
       rtl/cic5_dsd.v rtl/fir_stage.v rtl/asrc.v rtl/i2s_tx32.v rtl/i2c_slave.v rtl/top.v

.PHONY: all verify coef units sys switch mux lint synth clean

all: verify

# Everything that must be green before flashing.
verify: coef lint units synth sys switch mux

coef:
	python3 tools/gen_coefs.py --check

lint:
	mkdir -p build
	iverilog -g2005 -Wall -s top -o build/lint.vvp $(RTL)

# Bit-exact unit benches: every FIR configuration and the CIC at all R.
units:
	python3 tb/run_units.py

# System benches: 10 PCM rates, DSD64..512, DoP64..256 with clock offsets,
# output decoded from the I2S pins, THD+N / level / frequency checked.
# ~25-40 min with Icarus; single case: python3 tb/run_sys.py pcm44
sys:
	python3 tb/run_sys.py

# Live format switching (PCM 44.1 -> 96 -> DSD64 -> 768k -> DoP64 -> 192k,
# with and without clock stops): relock on the right path, no clicks.
switch:
	python3 tb/run_sys.py --switch

# Input selector + I2C: four transports on unrelated clocks (DSD64, PCM
# 44.1, PCM 96, dead input), selection / mute / status over I2C.
mux:
	python3 tb/run_sys.py --mux

# Elaboration gate (the real synthesis is Gowin IDE):
# 10 multipliers, 13 inferred memories (BSRAM), no errors.
synth:
	mkdir -p build
	yosys -p "read_verilog $(RTL); hierarchy -check -top top; proc; opt -fast; memory -nomap; opt -fast; flatten; stat" > build/synth.log
	! grep -iE "error" build/synth.log
	grep -E -q '[$$]mul +10$$' build/synth.log
	grep -E -q '[$$]mem_v2 +13$$' build/synth.log

clean:
	rm -rf build
