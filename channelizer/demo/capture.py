"""Capture raw DMA blocks from the channelizer for the demo video.

Each block is contiguous; between blocks the DMA may drop beats when the
selection outruns the link (~25 MB/s), so the slot rotation is fixed per
block in analysis. Saves {blocks: (nb, beats, 4) int16, t: block times, cfg}.
"""
import sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "host"))
from pluto_channelizer import PlutoChannelizer  # noqa: E402

OUT = Path(__file__).resolve().parent / "captures"


def capture(name, nblocks, channels, block_frames=None, gains=(50, 50), **cfg):
    with PlutoChannelizer(gains=gains, channels=channels, **cfg) as pfb:
        if block_frames:
            import iio
            pfb._buf = None
            pfb._block = len(pfb.order) * block_frames
            pfb._buf = iio.Buffer(pfb.rx, pfb._block)
            pfb._buf.refill()
        blocks, t = [], []
        t0 = time.time()
        for _ in range(nblocks):
            pfb._buf.refill()
            t.append(time.time() - t0)
            blocks.append(np.frombuffer(pfb._buf.read(), np.int16).reshape(-1, 4).copy())
        meta = dict(name=name, nch=pfb.nch, n=pfb.n, spacing=pfb.spacing, fs=pfb.fs, lo=pfb.lo,
                    bandwidth=pfb.bandwidth, order=pfb.order,
                    freqs=[pfb.channel_freq(c) for c in pfb.order], gains=pfb.gains)
    OUT.mkdir(exist_ok=True)
    path = OUT / f"{name}.npz"
    np.savez(path, blocks=np.array(blocks), t=np.array(t), meta=np.array(meta, dtype=object))
    print(f"{path}: {len(blocks)} blocks x {len(blocks[0])} beats, order {meta['order']}, "
          f"{len(blocks)*len(blocks[0])*8/1e6/(t[-1]+1e-9):.1f} MB/s")
    return path


def load(name):
    d = np.load(OUT / f"{name}.npz", allow_pickle=True)
    return d["blocks"], d["t"], d["meta"].item()
