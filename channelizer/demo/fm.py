"""Wideband-FM mono demodulation of channelizer slots (200 kS/s -> 48 kHz audio)."""
import numpy as np
from scipy.signal import lfilter, resample_poly, butter, sosfilt


class FMDemod:
    def __init__(self, fs=200e3, audio_fs=48000, tau=75e-6):
        self.fs, self.prev = fs, 1 + 0j
        a = np.exp(-1 / (fs * tau))
        self.de = ([1 - a], [1, -a]); self.zi = np.zeros(1)
        self.lp = butter(8, 15e3, fs=fs, output="sos"); self.lpz = np.zeros((self.lp.shape[0], 2))
        self.up, self.down = 6, 25            # 200k * 6/25 = 48k

    def __call__(self, z):
        z = np.concatenate([[self.prev], z]); self.prev = z[-1]
        d = np.angle(z[1:] * np.conj(z[:-1])) * self.fs / (2 * np.pi * 75e3)
        d, self.zi = lfilter(*self.de, d, zi=self.zi)
        d, self.lpz = sosfilt(self.lp, d, zi=self.lpz)
        return resample_poly(d, self.up, self.down).astype(np.float32)
