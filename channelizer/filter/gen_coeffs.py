#!/usr/bin/env python3
"""Write rtl/pfb_coeffs.mem from the equiripple prototype (see pfb_filter.py)."""
import argparse
from pathlib import Path

import pfb_filter as F

p = argparse.ArgumentParser()
p.add_argument("--taps", type=int, default=8, help="taps per phase")
p.add_argument("--pass-hz", type=float, default=100e3, help="protected half-bandwidth B")
p.add_argument("--stop-hz", type=float, default=None, help="default: spacing - B")
p.add_argument("--weight-stop", type=float, default=10.0)
p.add_argument("--out", default=str(Path(__file__).resolve().parents[1] / "rtl" / "pfb_coeffs.mem"))
a = p.parse_args()
h = F.equiripple(a.taps, a.pass_hz, a.stop_hz, a.weight_stop)
F.write_rom(a.out, h, a.taps)
r = F.metrics(F.quantize(h) / 2.0**F.COEF_FRAC, a.pass_hz)
print(a.out, {k: round(v, 3) for k, v in r.items()})
