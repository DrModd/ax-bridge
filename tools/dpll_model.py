#!/usr/bin/env python3
"""Bit-true-ish model of the ASRC DPLL (event-rate PI loop, frame-rate NCO).

Clock domain: mclk = 49.152 MHz (local). Output frame = 256 mclk (192 kHz).
Input events: one per 4 intermediate samples (44.1 / 48 kHz nominal),
timestamped in whole mclk cycles (+-1 cycle sync jitter).
phi: read position in intermediate samples, Q16.32 (wraps mod 2^48).
s:   phi advance per output frame, Q1.32.
At event k (time = frame start + dt mclk):
    e = T_k - (phi + s*dt/256);  phi += e >> kp;  s += e >> ki;  T += 4
Returns the timing error of the read position vs ideal, per output frame.
"""
import numpy as np

FR = 32
ONE = 1 << FR

def simulate(fam44, ppm, gears, seconds, seed=1, jitter=1):
    rng = np.random.default_rng(seed)
    mclk = 49.152e6
    f_int = (176400.0 if fam44 else 192000.0) * (1 + ppm * 1e-6)
    f_ev = f_int / 4
    n_frames = int(seconds * 192000)
    # event times in mclk cycles (continuous), first at t0
    t0 = 1000.3
    n_ev = int(seconds * f_ev) + 2
    ev_t = t0 + np.arange(n_ev) * (mclk / f_ev)
    ev_q = np.floor(ev_t) + rng.integers(0, jitter + 1, n_ev)   # sync delay
    s = int(round((176400 / 192000 if fam44 else 1.0) * ONE))
    phi = 0
    T = 0
    started = False
    ei = 0
    err = np.zeros(n_frames)
    g = 0
    gear_kp, gear_ki, gear_len, gear_a = gears[0]
    v = 0
    ev_count = 0
    for m in range(n_frames):
        fs_t = m * 256        # frame start time (mclk)
        if started:
            ideal = (fs_t - ev_t[0]) / mclk * f_int
            err[m] = (phi / ONE - ideal) / f_int     # seconds
        # process events inside this frame
        while ei < n_ev and ev_q[ei] < fs_t + 256:
            dt = int(ev_q[ei] - fs_t)
            if dt < 0:
                dt = 0
            if not started:
                phi = 0 - ((s * dt) >> 8)   # so that pred == T at this event
                T = 0
                started = True
            pred = phi + ((s * dt) >> 8)
            e = (T << FR) - pred
            v += (e - v) >> gear_a
            phi += v >> gear_kp
            s += v >> gear_ki
            T += 4
            ei += 1
            ev_count += 1
            if gear_len is not None and ev_count >= gear_len and g + 1 < len(gears):
                g += 1
                gear_kp, gear_ki, gear_len, gear_a = gears[g]
                ev_count = 0
        # advance NCO
        if started:
            phi += s
    return err

def tone_spur_db(err, f0, fs=192000.0, skip=0):
    e = err[skip:]
    n = len(e)
    t = np.arange(n) / fs
    y = np.sin(2 * np.pi * f0 * (t + e))
    w = np.hanning(n)
    Y = np.abs(np.fft.rfft(y * w)) ** 2
    f = np.fft.rfftfreq(n, 1 / fs)
    k0 = int(round(f0 / fs * n))
    sig = Y[k0 - 3:k0 + 4].sum()
    band = (f > 20) & (f < 20000)
    return 10 * np.log10((Y[band].sum() - sig) / sig)

if __name__ == "__main__":
    import sys
    gears = [(5, 12, 4000, 0), (8, 18, 8000, 3), (11, 25, None, 6)]
    for fam in (True, False):
        for ppm in (0.03, 0.3, 3.0, 47.0, -180.0):
            err = simulate(fam, ppm, gears, 3.0)
            n = len(err)
            st = err[int(0.4 * n):]
            off = st.mean()
            print(f"fam44={fam} ppm={ppm:7.2f}  steady err mean {off*1e9:8.2f} ns "
                  f"p-p {np.ptp(st)*1e9:6.3f} ns  rms {st.std()*1e12:7.1f} ps  "
                  f"THD+N@15k {tone_spur_db(err - off, 15000.0, skip=int(0.4*n)):6.1f} dB")
