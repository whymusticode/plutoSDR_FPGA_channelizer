"""LTE downlink sync: PSS (N_ID2) and SSS (N_ID1) detection -> physical cell ID.

Works on complex baseband at any rate fs = N * 15 kHz with N >= 128 (N need
not be a power of two); the 62 sync subcarriers occupy +-465 kHz. Formulas
from 3GPP TS 36.211 6.11. FDD: SSS sits one OFDM symbol before PSS.
"""
import numpy as np
from scipy.signal import fftconvolve

SC = 15e3
PSS_ROOT = {0: 25, 1: 29, 2: 34}


def pss_freq(nid2):
    u = PSS_ROOT[nid2]
    n = np.arange(62)
    m = np.where(n < 31, n, n + 1)
    return np.exp(-1j * np.pi * u * m * (m + 1) / 63)


def to_time(d62, N):
    """62 sync subcarriers onto an N-point grid (DC unused), one symbol of N samples."""
    X = np.zeros(N, complex)
    k = np.r_[np.arange(-31, 0), np.arange(1, 32)]
    X[k % N] = d62
    return np.fft.ifft(X) * np.sqrt(N)


def _m_seq(taps, n=31):
    x = [0, 0, 0, 0, 1]
    for i in range(n - 5):
        x.append(sum(x[i + t] for t in taps) % 2)
    return 1 - 2 * np.array(x[:n])


S_TILDE = _m_seq([2, 0])          # x(i+5) = x(i+2) + x(i)
C_TILDE = _m_seq([3, 0])          # x(i+5) = x(i+3) + x(i)
Z_TILDE = _m_seq([4, 2, 1, 0])    # x(i+5) = x(i+4) + x(i+2) + x(i+1) + x(i)


def sss_freq(nid1, nid2, subframe=0):
    q1 = nid1 // 30
    q = (nid1 + q1 * (q1 + 1) // 2) // 30
    mp = nid1 + q * (q + 1) // 2
    m0 = mp % 31
    m1 = (m0 + mp // 31 + 1) % 31
    n = np.arange(31)
    s0, s1 = S_TILDE[(n + m0) % 31], S_TILDE[(n + m1) % 31]
    c0, c1 = C_TILDE[(n + nid2) % 31], C_TILDE[(n + nid2 + 3) % 31]
    z0, z1 = Z_TILDE[(n + m0 % 8) % 31], Z_TILDE[(n + m1 % 8) % 31]
    d = np.zeros(62)
    if subframe == 0:
        d[0::2], d[1::2] = s0 * c0, s1 * c1 * z0
    else:
        d[0::2], d[1::2] = s1 * c0, s0 * c1 * z1
    return d


def pss_search(x, fs):
    """Normalised PSS correlation for each N_ID2: dict nid2 -> |corr| array."""
    N = int(round(fs / SC))
    out = {}
    e = np.convolve(np.abs(x) ** 2, np.ones(N), "valid") + 1e-12
    for nid2 in range(3):
        t = to_time(pss_freq(nid2), N)
        c = np.abs(fftconvolve(x, np.conj(t[::-1]), "valid")) / np.sqrt(e * N)
        out[nid2] = c
    return out


def detect(x, fs, offsets=(0.0,)):
    """Best (score, offset_hz, nid2, start) over carrier offsets and N_ID2."""
    best = (0, 0, 0, 0)
    n = np.arange(len(x))
    for off in offsets:
        y = x * np.exp(-2j * np.pi * off * n / fs)
        for nid2, c in pss_search(y, fs).items():
            i = int(np.argmax(c))
            if c[i] > best[0]:
                best = (float(c[i]), off, nid2, i)
    return best


def cell_id(x, fs, nid2, pss_starts, cp="normal"):
    """N_ID1 from the SSS one symbol before each PSS; returns (pci, score)."""
    N = int(round(fs / SC))
    cp_len = int(round(N * 144 / 2048))
    k = np.r_[np.arange(-31, 0), np.arange(1, 32)] % N
    H = np.fft.fft(to_time(pss_freq(nid2), N))[k]
    scores = np.zeros((168, 2))
    for s in pss_starts:
        a = s - N - cp_len
        if a < 0:
            continue
        P = np.fft.fft(x[s:s + N])[k]
        S = np.fft.fft(x[a:a + N])[k]
        h = P / H                                  # channel estimate from the PSS
        eq = S * np.conj(h) / (np.abs(h) ** 2 + 1e-9)
        for nid1 in range(168):
            for sf in (0, 1):
                scores[nid1, sf] += np.real(np.vdot(sss_freq(nid1, nid2, sf * 5), eq))
    i, sf = np.unravel_index(np.argmax(np.abs(scores)), scores.shape)
    s = np.sort(np.abs(scores).ravel())
    return 3 * int(i) + nid2, float(s[-1] / (s[-2] + 1e-9))
