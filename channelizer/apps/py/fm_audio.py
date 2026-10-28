"""Wideband-FM mono audio from every selected channel, in numpy.

    --rx 1|2   receiver (default 1)

Phase step per sample, averaged down to ~48 kHz, then 75 us de-emphasis as a
short FIR at the audio rate (state carried across blocks).
"""
import numpy as np
from pfbapp import App, AUDIO

app = App()
rx = app.arg("--rx", 1, int) - 1
D = max(1, round(app.rate / 48000))
arate = app.rate / D
app.meta(app="fm_audio", audio_rate=arate)
h = np.exp(-np.arange(int(4 * 75e-6 * arate) + 1) / (75e-6 * arate))
h /= h.sum()
gain = app.rate / (2 * np.pi * 75e3) * 32767
prev = np.ones(app.slots, np.complex64)
tail = np.zeros((app.slots, len(h) - 1))
for first, x in app.blocks():
    for s in range(app.slots):
        z = app.iq(x, s, rx)
        d = np.angle(z * np.conj(np.concatenate([[prev[s]], z[:-1]])))
        prev[s] = z[-1]
        a = np.concatenate([tail[s], d[: len(d) // D * D].reshape(-1, D).mean(1)])
        tail[s] = a[-(len(h) - 1):]
        y = np.convolve(a, h, "valid") * gain
        app.send(AUDIO, s, first // D, np.clip(y, -32767, 32767).astype(np.int16), rx)
app.close()
