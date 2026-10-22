"""Prototype low-pass design for the critically sampled PFB.

In a critically sampled bank, a channel's output at offset f from its centre
is aliased by whatever sits at f +- one channel spacing, attenuated by the
prototype there. So a signal occupying +-B of a channel centre stays clean
when the passband reaches B and the stopband starts at spacing - B.
Equiripple (Parks-McClellan) spends the taps on the steepest transition for
a given ripple/attenuation trade, set by the band weights.

The ROM layout matches rtl/pfb_channelizer.sv: line p packs taps t = 0..T-1 of
phase p (tap 0 in the LSBs, multiplying the oldest frame), each COEF_W-bit
two's complement at coef_frac(channels) fractional bits, with h normalised to
unity DC gain. For a symmetric prototype this is proto.reshape(T, N)[t, p].
The same values, tap by tap, are what the host loads at runtime.
"""
import numpy as np
from scipy.signal import freqz, remez

N, N_MAX, TAPS_MAX, COEF_W = 64, 256, 8, 18
DATA_W, DATA_FRAC = 25, 22


def coef_frac(channels):
    """Q format of the taps for a bank of `channels`; the RTL derives its FIR
    shift from the same FFT size. A unity-DC prototype's largest tap is about
    1/channels, so this keeps it near the top of COEF_W at every size."""
    return 16 + int(channels).bit_length() - 1


COEF_FRAC = coef_frac(N)


def equiripple(taps_per_phase=8, pass_hz=100e3, stop_hz=None, weight_stop=10.0,
               fs=25.6e6, channels=N):
    """Unity-DC equiripple prototype, length channels * taps_per_phase."""
    spacing = fs / channels
    stop_hz = spacing - pass_hz if stop_hz is None else stop_hz
    h = remez(channels * taps_per_phase, [0, pass_hz, stop_hz, fs / 2], [1, 0],
              weight=[1, weight_stop], fs=fs, maxiter=200)
    return h / h.sum()


def kaiser(taps_per_phase=4, beta=7.8562, channels=N):
    """The verilog_rapid_prototyping windowed-sinc prototype (cutoff = spacing/2)."""
    length = channels * taps_per_phase
    n = np.arange(length) - (length - 1) / 2
    c = 1.0 / (2 * channels)
    h = 2 * c * np.sinc(2 * c * n) * np.kaiser(length, beta)
    return h / h.sum()


def design(channels, spacing, bandwidth, taps_per_phase=TAPS_MAX, weight_stop=10.0):
    """Prototype for `channels` channels `spacing` Hz apart, keeping +-bandwidth/2
    of every centre clean: passband to bandwidth/2, stopband from
    spacing - bandwidth/2 (what aliases onto that band when critically sampled)."""
    if not 0 < bandwidth < spacing:
        raise ValueError(f"bandwidth {bandwidth:g} Hz must be below the spacing {spacing:g} Hz")
    return equiripple(taps_per_phase, bandwidth / 2, spacing - bandwidth / 2, weight_stop,
                      fs=channels * spacing, channels=channels)


def quantize(h, bits=COEF_W, frac=COEF_FRAC):
    q = np.rint(np.asarray(h) * 2.0**frac).astype(np.int64)
    lim = 1 << (bits - 1)
    if q.max() >= lim or q.min() < -lim:
        raise ValueError(f"coefficient exceeds {bits}-bit range at Q{frac}")
    return q


def response_db(h, offsets_hz, fs=25.6e6):
    _, H = freqz(h, worN=2 * np.pi * np.asarray(offsets_hz, float) / fs)
    return 20 * np.log10(np.abs(H) + 1e-300)


def metrics(h, pass_hz, fs=25.6e6, channels=N):
    """Ripple over +-pass_hz and worst alias attenuation onto that band."""
    spacing = fs / channels
    f = np.linspace(0, pass_hz, 400)
    pb = response_db(h, f, fs)
    alias = response_db(h, np.linspace(spacing - pass_hz, fs / 2, 4000), fs)
    return dict(ripple_db=float(pb.max() - pb.min()), edge_db=float(response_db(h, [spacing / 2], fs)[0]),
                alias_rejection_db=float(-alias.max()),
                max_phase_abs_sum=float(np.abs(np.asarray(h).reshape(-1, channels)).sum(0).max()))


def phase_taps(h, channels, taps_max=TAPS_MAX):
    """Quantized (channels, taps_max) int array in the RTL's layout: [p, t] is tap
    t of phase p, tap 0 multiplying the oldest frame. A shorter prototype is
    zero padded on the oldest side, which only adds delay.

    Also checks the FFT's headroom: the worst-case branch sum, times the FFT's
    gain of `channels`, must fit DATA_W at DATA_FRAC for a full-scale input."""
    taps = len(h) // channels
    if len(h) != taps * channels or not 1 <= taps <= taps_max:
        raise ValueError(f"prototype length {len(h)} is not channels * (1..{taps_max})")
    q = quantize(h, COEF_W, coef_frac(channels)).reshape(taps, channels).T
    q = np.concatenate([np.zeros((channels, taps_max - taps), np.int64), q], axis=1)
    worst = channels * np.abs(q).sum(1).max() / 2.0 ** coef_frac(channels)
    if worst * 2.0 ** DATA_FRAC >= 2 ** (DATA_W - 1):
        raise ValueError(f"prototype gain {worst:.2f} would overflow the FFT datapath")
    return q


def write_rom(path, h, taps_per_phase, bits=COEF_W, frac=COEF_FRAC, channels=N):
    q = quantize(h, bits, frac).reshape(taps_per_phase, channels)
    mask = (1 << bits) - 1
    lines = []
    for p in range(channels):
        word = 0
        for t in range(taps_per_phase):
            word |= (int(q[t, p]) & mask) << (t * bits)
        lines.append(f"{word:0{(taps_per_phase * bits + 3) // 4}x}")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def write_twiddles(path, n_max=N_MAX, bits=18):
    """W_{n_max}^k for k < n_max/2 at Q(bits-1) scaled by 2^(bits-1) - 1, imag
    in the upper half of each word; a 2^m-smaller FFT uses every 2^m-th entry."""
    k = np.arange(n_max // 2)
    scale, mask = (1 << (bits - 1)) - 1, (1 << bits) - 1
    wr = np.round(np.cos(2 * np.pi * k / n_max) * scale).astype(np.int64)
    wi = np.round(-np.sin(2 * np.pi * k / n_max) * scale).astype(np.int64)
    with open(path, "w") as f:
        f.write("".join(f"{((int(i) & mask) << bits) | (int(r) & mask):0{(2 * bits + 3) // 4}x}\n"
                        for r, i in zip(wr, wi)))
