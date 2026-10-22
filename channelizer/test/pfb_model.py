"""Bit-exact and floating-point models of rtl/pfb_channelizer.sv.

Both take the channelizer's input exactly as the ingress delivers it: an
(n, 4) int array of {rx0_i, rx0_q, rx1_i, rx1_q} sign-extended 12-bit ADC
codes, which is also what stock firmware's IIO buffer returns. Both return
(frames, 2, channels) complex arrays in natural FFT-bin order, scaled in input
LSBs, so fixed - float is the implementation error and nothing else.

The channel count is the runtime FFT size, any power of two up to N_MAX, and
is taken from the coefficients: an int array (channels, taps) or a ROM image
path (default rtl/pfb_coeffs.mem, the 64-channel bank the bitstream boots
with). Coefficients are Q(coef_frac(logn)), which keeps the largest tap of a
unity-DC prototype (about 1/channels) near the top of COEF_W at every size;
the FIR's output shift follows, so the datapath sees the same scale for any
size. OUT_FRAC is the output stage's fractional bits; the Pluto wrapper uses
FRAC + 3.
"""
from functools import lru_cache
from pathlib import Path

import numpy as np

RTL = Path(__file__).resolve().parents[1] / "rtl"
N_MAX, LOGN_MAX = 256, 8
N, LOGN = 64, 6          # the boot configuration, and the default here
COEF_W = 18
TWID_W, TWID_FRAC = 18, 17
DATA_W, DATA_FRAC = 25, 22
WIDTH, FRAC = 16, 15
OUT_FRAC = 18  # pfb_channelizer_wrapper.v


def coef_frac(logn):
    return 16 + logn


def fir_shift(logn):
    return FRAC + coef_frac(logn) - DATA_FRAC


COEF_FRAC = coef_frac(LOGN)
FIR_SHIFT = fir_shift(LOGN)


def _signed(v, bits):
    mask = (1 << bits) - 1
    v = (v & mask) if isinstance(v, int) else (np.asarray(v, dtype=np.int64) & mask)
    return np.where(v >> (bits - 1), v - (1 << bits), v)


def _clamp(v, bits):
    return np.clip(v, -(1 << (bits - 1)), (1 << (bits - 1)) - 1)


def load_coeffs(path=None):
    """(channels, taps) int; tap 0 multiplies the oldest frame."""
    if path is not None and not isinstance(path, (str, Path)):
        return np.asarray(path, dtype=np.int64)
    text = Path(path or RTL / "pfb_coeffs.mem").read_text().split()
    taps = len(text[0]) * 4 // COEF_W
    words = [int(l, 16) for l in text]
    return np.array([[_signed(w >> (t * COEF_W), COEF_W) for t in range(taps)]
                     for w in words], dtype=np.int64)


TAPS = load_coeffs().shape[1]  # of the current ROM; frames to drop after reset - 1


def log2_channels(c):
    n = len(c)
    logn = n.bit_length() - 1
    if n != 1 << logn or not 1 <= logn <= LOGN_MAX:
        raise ValueError(f"{n} channels: need a power of two from 2 to {N_MAX}")
    return logn


def load_twiddles():
    """W_N_MAX^k for k < N_MAX/2; a smaller FFT strides through the table."""
    words = [int(l, 16) for l in (RTL / "twiddle.mem").read_text().split()]
    wr = np.array([_signed(w, TWID_W) for w in words], dtype=np.int64)
    wi = np.array([_signed(w >> TWID_W, TWID_W) for w in words], dtype=np.int64)
    return wr, wi


def bitrev(i, bits=LOGN):
    return int(f"{i:0{bits}b}"[::-1], 2)


@lru_cache(None)
def stages(logn):
    """Butterfly addressing, exactly as the RTL's address generator computes it."""
    out = []
    for s in range(logn):
        span = 1 << (logn - 1 - s)
        c = np.arange(1 << (logn - 1))
        j, g = c & (span - 1), c >> (logn - 1 - s)
        ia = (g << (logn - s)) | j
        out.append((ia, ia | span, j << (s + LOGN_MAX - logn)))
    return out


def position_from_natural(logn):
    """Memory position i holds natural bin bitrev(i) after a DIF transform."""
    return np.argsort([bitrev(i, logn) for i in range(1 << logn)])


def _frames(x, n):
    x = np.asarray(x, dtype=np.int64)
    f = len(x) // n
    return x[: f * n].reshape(f, n, 4)


def _polyphase(blocks, h):
    """sum_t h[p, t] * x[frame - (T-1) + t, p], zero history as after reset."""
    taps = h.shape[1]
    pad = np.concatenate([np.zeros((taps - 1,) + blocks.shape[1:], blocks.dtype), blocks])
    f = len(blocks)
    return sum(pad[t : t + f] * h[None, :, t, None] for t in range(taps))


def _fixed_beats(x, out_frac, coeffs):
    """Integer output codes, (frames, n positions, 2 rx) complex, beat order."""
    out_shift = DATA_FRAC - out_frac
    c = load_coeffs(coeffs)
    logn = log2_channels(c)
    shift = fir_shift(logn)
    wr, wi = load_twiddles()
    acc = _polyphase(_frames(x, len(c)), c) + (1 << (shift - 1))
    v = _clamp(acc >> shift, DATA_W)              # (frames, n positions, 4 lanes), Q22
    re, im = v[..., 0::2].copy(), v[..., 1::2].copy()  # (frames, n, 2 rx)
    tr = 1 << (TWID_FRAC - 1)
    for ia, ib, tw in stages(logn):
        ar, ai, br, bi = re[:, ia], im[:, ia], re[:, ib], im[:, ib]
        dr, di = _clamp(ar - br, DATA_W), _clamp(ai - bi, DATA_W)
        re[:, ia], im[:, ia] = _clamp(ar + br, DATA_W), _clamp(ai + bi, DATA_W)
        w_r, w_i = wr[tw][None, :, None], wi[tw][None, :, None]
        re[:, ib] = _clamp(((dr * w_r + tr) - di * w_i) >> TWID_FRAC, DATA_W)
        im[:, ib] = _clamp(((dr * w_i + tr) + di * w_r) >> TWID_FRAC, DATA_W)

    def to_sample(u):  # round-half-up added in a 25-bit register, then clamp
        u = _signed(u + (1 << (out_shift - 1)), DATA_W)
        return _clamp(u >> out_shift, WIDTH)
    return to_sample(re) + 1j * to_sample(im)


def channelize_fixed(x, out_frac=OUT_FRAC, coeffs=None):
    y = _fixed_beats(x, out_frac, coeffs) / 2.0 ** (out_frac - FRAC)
    return y[:, position_from_natural(y.shape[1].bit_length() - 1)].transpose(0, 2, 1)


def channelize_float(x, coeffs=None):
    c = load_coeffs(coeffs)
    h = c / 2.0 ** coef_frac(log2_channels(c))
    v = _polyphase(_frames(x, len(c)).astype(np.float64), h)
    z = v[..., 0::2] + 1j * v[..., 1::2]          # (frames, n, 2)
    return np.fft.fft(z, axis=1).transpose(0, 2, 1)


def beats_fixed(x, out_frac=OUT_FRAC, coeffs=None):
    """Raw m_axis_tdata words, (frames*n, 4) int16, for comparing with RTL sim."""
    y = _fixed_beats(x, out_frac, coeffs)
    return np.stack([y[..., 0].real, y[..., 0].imag, y[..., 1].real, y[..., 1].imag],
                    axis=-1).reshape(-1, 4).astype(np.int16)


def channel_of_bin(natural_bin, n=N):
    """fftshift channel index (0 = -Fs/2) used by the firmware's mask."""
    return (np.asarray(natural_bin) + n // 2) % n
