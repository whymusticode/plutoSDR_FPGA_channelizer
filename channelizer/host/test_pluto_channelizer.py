#!/usr/bin/env python3
"""Hardware check for PlutoChannelizer: channelizer firmware, TX1 cabled to RX2,
RX1 on an antenna. For each bank configuration, a host tone on TX1 a quarter
spacing above one channel's centre must appear in that channel only, at a
phase step of pi/2 per output sample, contiguous across read() calls; the
neighbouring channels show the leakage. The 64-channel case also lists FM
stations on RX1 as before."""
import sys
import time

import iio
import numpy as np

from pluto_channelizer import PlutoChannelizer

CONFIGS = [  # (label, kwargs, tone channel, other selected channels)
    ("64 x 400 kHz", dict(nch=64, spacing=400e3, bandwidth=200e3, lo=90.7e6), 22, [34, 52, 37]),
    ("6 x 400 kHz (n=8)", dict(nch=6, spacing=400e3, bandwidth=300e3, lo=90.7e6), 2, [1, 3, 5]),
    ("4 x 600 kHz", dict(nch=4, spacing=600e3, bandwidth=400e3, lo=90.7e6), 1, [0, 2, 3]),
    ("200 x 100 kHz (n=256)", dict(nch=200, spacing=100e3, bandwidth=50e3, lo=90.7e6), 80, [79, 81, 120]),
]
only = sys.argv[1:]


def db(z):
    return 10 * np.log10(np.mean(np.abs(z) ** 2) / 2048**2 + 1e-30)


for label, kw, tone_ch, others in CONFIGS:
    if only and not any(o in label for o in only):
        continue
    t0 = time.time()
    with PlutoChannelizer(gains=(40, 30), channels=[tone_ch] + others, **kw) as pfb:
        setup = time.time() - t0
        f = pfb.channel_freq(tone_ch) + pfb.spacing / 4 - pfb.lo
        tx = pfb.ctx.find_device("cf-ad9361-dds-core-lpc")
        pfb.phy.find_channel("altvoltage1", True).attrs["frequency"].value = str(int(pfb.lo))
        pfb.phy.find_channel("voltage0", True).attrs["hardwaregain"].value = "-40"
        s = 0.5 * np.exp(2j * np.pi * f * np.arange(4096) / pfb.fs)
        d = np.zeros((4096, 4), np.int16)
        d[:, 0], d[:, 1] = np.round(s.real * 32767), np.round(s.imag * 32767)
        for i in range(4):
            tx.find_channel(f"voltage{i}", True).enabled = True
        tb = iio.Buffer(tx, 4096, True)
        tb.write(bytearray(d.tobytes()))
        tb.push()
        time.sleep(0.3)
        pfb.select(pfb.channels)                 # fresh buffer after the tone is up
        pfb.status()                             # clear the drop flags
        rot = pfb.align(tone_ch, rx=1)
        n = int(min(300_000, 0.5 * pfb.spacing))
        a, b = pfb.read(n), pfb.read(n)
        z = np.concatenate([a[tone_ch][1], b[tone_ch][1]])   # RX2, across the read() boundary
        dphi = np.angle(z[1:] * np.conj(z[:-1]))
        dev = np.abs(np.angle(np.exp(1j * (dphi - np.median(dphi)))))
        st = pfb.status()
        m = pfb.filter_metrics
        print(f"== {label}: fs {pfb.fs/1e6:.3f} MS/s, setup {setup:.1f} s, rotation {rot}, "
              f"filter ripple {m['ripple_db']:.2f} dB / alias rejection {m['alias_rejection_db']:.0f} dB, {st}")
        print(f"   tone ch {tone_ch} ({pfb.channel_freq(tone_ch)/1e6:.3f} MHz +{pfb.spacing/4e3:g} kHz): "
              f"{db(z):.1f} dBFS, phase step {np.median(dphi):.3f} rad (expect {np.pi/2:.3f}), "
              f"outliers {np.count_nonzero(dev > 0.5)} of {len(dev)}")
        for ch in pfb.channels:
            print(f"   ch {ch:3d} {pfb.channel_freq(ch)/1e6:8.3f} MHz  RX1 {db(a[ch][0]):6.1f} dBFS"
                  f"  RX2 {db(a[ch][1]):6.1f} dBFS")
        del tb, tx
