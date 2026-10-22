#!/usr/bin/env python3
"""Run tb_pfb.sv under iverilog and require bit-exact agreement with pfb_model.

Covers the boot image (64 channels from rtl/pfb_coeffs.mem), every runtime
size from 2 to N_MAX with taps loaded through the coefficient port, a size
change across a reset, and the sustained throughput per size: with input
offered every cycle, cycles per accepted beat is what limits the sample rate
(fs_max = 200 MHz / cycles per beat).
"""
import argparse, subprocess, sys, tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "filter"))
import pfb_model as m
import pfb_filter as F

STAGE_GAP_BASE = None  # override the RTL parameter (--stage-gap-base)


def stimulus(n, rng):
    t = np.arange(n)
    tone = 2000 * np.exp(2j * np.pi * 0.137 * t)
    x = np.stack([tone.real, tone.imag, -tone.imag, tone.real], 1)
    x = np.rint(x + rng.normal(0, 30, x.shape))
    x[: n // 4] = rng.integers(-2048, 2048, (n // 4, 4))   # full-scale noise
    x[n // 2 : n // 2 + 64] = [2047, -2048, -2048, 2047]   # clamp corners
    return np.clip(x, -2048, 2047).astype(np.int64)


def taps_for(logn):
    n = 1 << logn
    return F.phase_taps(F.design(n, 400e3, 300e3), n)


def words(x):
    return "".join(f"{(int(r[0]) & 0xFFFF) | (int(r[1]) & 0xFFFF) << 16 | (int(r[2]) & 0xFFFF) << 32 | (int(r[3]) & 0xFFFF) << 48:016x}\n"
                   for r in x)


def coef_hex(q):
    mask = (1 << F.COEF_W) - 1
    return "".join(f"{sum((int(v) & mask) << (t * F.COEF_W) for t, v in enumerate(row)):036x}\n"
                   for row in q)


def run(segs, gap, stall, work, mult=4):
    """segs: [(logn, coeffs or None for the boot image, x)]; returns per-segment beats, cycles."""
    (work / "in.hex").write_text("".join(words(x) for _, _, x in segs))
    params = []
    for k, (logn, q, x) in enumerate(segs):
        params += [f"-Ptb_pfb.NSAMP{k}={len(x)}", f"-Ptb_pfb.LOGN{k}={logn}",
                   f"-Ptb_pfb.LOAD{k}={int(q is not None)}"]
        if q is not None:
            (work / f"coef{k}.hex").write_text(coef_hex(q))
    if STAGE_GAP_BASE is not None:
        params.append(f"-Ptb_pfb.STAGE_GAP_BASE={STAGE_GAP_BASE}")
    for f in ("pfb_coeffs.mem", "twiddle.mem"):
        (work / f).write_bytes((m.RTL / f).read_bytes())
    subprocess.run(["iverilog", "-g2012", f"-Ptb_pfb.GAP={gap}", f"-Ptb_pfb.STALL={stall}",
                    f"-Ptb_pfb.MULT_TAPS={mult}", *params, "-o", str(work / "a.out"),
                    str(HERE / "tb_pfb.sv"), str(m.RTL / "pfb_channelizer.sv")], check=True)
    subprocess.run(["vvp", "-n", "a.out"], cwd=work, check=True, stdout=subprocess.DEVNULL)
    out = [int(l, 16) for l in (work / "out.hex").read_text().split()]
    got = np.array([[(w >> (16 * k)) & 0xFFFF for k in range(4)] for w in out],
                   dtype=np.uint16).view(np.int16).reshape(-1, 4)
    cycles = [int(l.split()[2]) for l in (work / "cycles.txt").read_text().splitlines()]
    return got, cycles


def check(label, segs, gap, stall, work, mult=4):
    got, cycles = run(segs, gap, stall, work, mult)
    want = np.concatenate([m.beats_fixed(x, coeffs=q) for _, q, x in segs])
    bad = np.count_nonzero(np.any(got[: len(want)] != want[: len(got)], axis=1))
    ok = len(got) == len(want) and bad == 0
    print(f"{label}: {len(got)}/{len(want)} beats, {bad} mismatched" + ("" if ok else "  FAIL"))
    return ok, cycles


def main():
    global STAGE_GAP_BASE
    p = argparse.ArgumentParser()
    p.add_argument("--stage-gap-base", type=int, default=None, help="override the RTL parameter")
    p.add_argument("--quick", action="store_true", help="one rate pattern per size")
    a = p.parse_args()
    STAGE_GAP_BASE = a.stage_gap_base
    rng = np.random.default_rng(0)
    x = stimulus(4096, rng)
    ok = True
    rates = []
    with tempfile.TemporaryDirectory() as d:
        w = Path(d)
        # The boot image is the previous fixed 64-channel design, bit for bit.
        for gap, stall, mult in ((7, 0, 4), (0, 1, 4), (0, 0, 2), (0, 1, 8)):
            ok &= check(f"boot n=64 gap={gap} stall={stall} mult_taps={mult}",
                        [(6, None, x)], gap, stall, w, mult)[0]
        for logn in range(1, m.LOGN_MAX + 1):
            q = taps_for(logn)
            patterns = ((0, 0),) if a.quick else ((0, 0), (0, 1), (3, 0), (7, 1))
            for gap, stall in patterns:
                good, cycles = check(f"n={1 << logn:3d} gap={gap} stall={stall}",
                                     [(logn, q, x)], gap, stall, w)
                ok &= good
                if (gap, stall) == (0, 0):
                    rates.append((1 << logn, cycles[0] / (len(x) - 1)))
        # Size changes across a reset: nothing of the old configuration survives.
        for l0, l1 in ((8, 1), (1, 8), (6, 3), (2, 7)):
            ok &= check(f"switch n={1 << l0} -> n={1 << l1}",
                        [(l0, taps_for(l0), x[:2048]), (l1, taps_for(l1), x[2048:])], 0, 1, w)[0]

    print("\nsustained throughput, input offered every cycle at 200 MHz:")
    for n, cpb in rates:
        print(f"  n={n:3d}: {cpb:5.2f} cycles/beat -> fs_max {200 / cpb:5.1f} MS/s")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
