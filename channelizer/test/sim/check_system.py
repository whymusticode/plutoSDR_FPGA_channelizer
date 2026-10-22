#!/usr/bin/env python3
"""Run tb_system.sv (control registers + channelizer + selector) under iverilog.

Each case configures a size and taps over AXI-Lite, streams at the real-rate
input gap, and commits a second mask mid-stream. The selected beats must be
the bit-exact model's frames filtered by the first mask up to some frame
boundary and by the second after it, with the channel of each beat in
fftshift numbering.
"""
import subprocess, sys, tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(ROOT / "filter"))
import pfb_model as m
import pfb_filter as F
from check_rtl import stimulus, words, coef_hex, taps_for  # noqa: E402

SOURCES = [HERE / "tb_system.sv", HERE / "xpm_stubs.v", ROOT / "hdl" / "pfb_ctrl.v",
           ROOT / "hdl" / "pfb_channelizer_wrapper.v", ROOT / "hdl" / "pfb_dual_bin_selector.v",
           m.RTL / "pfb_channelizer.sv"]


def mask_words(channels):
    bits = 0
    for c in channels:
        bits |= 1 << int(c)
    return "".join(f"{(bits >> (32 * k)) & 0xFFFFFFFF:08x}\n" for k in range(8))


def run(logn, mask0, mask1, x, switch_at, work, gap=7):
    q = taps_for(logn)
    (work / "in.hex").write_text(words(x))
    (work / "coef.hex").write_text(coef_hex(q))
    (work / "mask0.hex").write_text(mask_words(mask0))
    (work / "mask1.hex").write_text(mask_words(mask1))
    for f in ("pfb_coeffs.mem", "twiddle.mem"):
        (work / f).write_bytes((m.RTL / f).read_bytes())
    subprocess.run(["iverilog", "-g2012", f"-Ptb_system.NSAMP={len(x)}", f"-Ptb_system.LOGN={logn}",
                    f"-Ptb_system.GAP={gap}", f"-Ptb_system.SWITCH_AT={switch_at}",
                    "-o", str(work / "a.out"), *map(str, SOURCES)], check=True)
    subprocess.run(["vvp", "-n", "a.out"], cwd=work, check=True, stdout=subprocess.DEVNULL)
    out = [int(l, 16) for l in (work / "sel.hex").read_text().split()]
    got = np.array([[(w >> (16 * k)) & 0xFFFF for k in range(4)] for w in out],
                   dtype=np.uint16).view(np.int16).reshape(-1, 4)
    regs = dict(l.split() for l in (work / "regs.txt").read_text().splitlines())
    return got, regs, q


def expected(x, q, logn, mask0, mask1):
    n = 1 << logn
    beats = m.beats_fixed(x, coeffs=q).reshape(-1, n, 4)
    chan = (np.array([m.bitrev(p, logn) for p in range(n)]) + n // 2) % n
    keep0, keep1 = np.isin(chan, list(mask0)), np.isin(chan, list(mask1))
    return [np.concatenate([b[keep0] for b in beats[:k]] + [b[keep1] for b in beats[k:]])
            .reshape(-1, 4) for k in range(len(beats) + 1)]


rng = np.random.default_rng(1)
x = stimulus(4096, rng)
ok = True
with tempfile.TemporaryDirectory() as d:
    for logn in (1, 2, 3, 6, 8):
        n = 1 << logn
        mask0 = sorted(rng.choice(n, max(1, n // 3), replace=False))
        mask1 = mask0
        while mask1 == mask0:
            mask1 = sorted(rng.choice(n, max(1, n // 2), replace=False))
        got, regs, q = run(logn, mask0, mask1, x, 2048, Path(d))
        cands = expected(x, q, logn, mask0, mask1)
        hit = [k for k, e in enumerate(cands) if len(e) == len(got) and (e == got).all()]
        regs_ok = (regs["id"] == "50464232" and regs["build"] == "00120808"
                   and regs["control_boot"] == "00000600"
                   and regs["coef_idx"] == f"{n * 8 % 2048:08x}"   # 11 bits, wraps at 256
                   and regs["status_held"] == "00000100"
                   and regs["control"] == f"{logn << 8:08x}"
                   and int(regs["status"], 16) & 0x103 == 0)
        good = bool(hit) and regs_ok and len(mask0) and hit[0] not in (0, len(cands) - 1)
        ok &= good
        where = f"switch after frame {hit[0]} of {len(cands) - 1}" if hit else "no frame boundary matches"
        print(f"n={n:3d}: {len(got)} beats, {where}, regs {'ok' if regs_ok else regs}"
              + ("" if good else "  FAIL"))
sys.exit(0 if ok else 1)
