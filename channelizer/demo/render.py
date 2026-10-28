#!/usr/bin/env python3
"""Render the demo video from the real captures in demo/captures/.

    python3 render.py            # -> demo/out/plutosdr_fpga_channelizer.mp4

Every number, spectrum, packet and audio sample on screen comes from a
capture made with the channelizer firmware; nothing is synthesised except
text and layout. Scenes are rendered with matplotlib straight into ffmpeg.
"""
import os
import pickle
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, Rectangle

import ble
import lte
from capture import load

HERE = Path(__file__).resolve().parent
CAP, OUT = HERE / "captures", HERE / "out"
W, H, FPS = 1920, 1080, 30
# The session's LD_LIBRARY_PATH carries an alsa-lib built against a newer glibc
# than the one ffmpeg would then pick up; ffmpeg needs neither.
FFENV = {k: v for k, v in os.environ.items() if k != "LD_LIBRARY_PATH"}
BG, FG, DIM = "#0b0f17", "#e8edf5", "#7d8899"
ACC = ["#4cc9f0", "#f72585", "#b8f25a", "#ffb703", "#9d7bff", "#2ec4b6"]
plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": FG, "axes.labelcolor": DIM,
                     "xtick.color": DIM, "ytick.color": DIM, "axes.edgecolor": "#2a3242",
                     "axes.facecolor": BG, "figure.facecolor": BG, "savefig.facecolor": BG})


def ease(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


class Writer:
    def __init__(self, path):
        self.p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgba",
                                   "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-c:v", "libx264",
                                   "-pix_fmt", "yuv420p", "-crf", "18", "-preset", "medium", str(path)],
                                  stdin=subprocess.PIPE, env=FFENV)
        self.fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)

    def frame(self, draw, *a):
        self.fig.clf()
        draw(self.fig, *a)
        self.fig.canvas.draw()
        self.p.stdin.write(self.fig.canvas.buffer_rgba())

    def close(self):
        self.p.stdin.close()
        self.p.wait()
        plt.close(self.fig)


class Preview(Writer):
    """Draws only frame `at` of a scene to a PNG, for layout checks."""
    def __init__(self, path, at):
        self.path, self.at, self.i = path, at, 0
        self.fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)

    def frame(self, draw, *a):
        if self.i == self.at:
            self.fig.clf(); draw(self.fig, *a); self.fig.savefig(self.path, dpi=50)
        self.i += 1

    def close(self):
        plt.close(self.fig)


def title(fig, text, sub=None, a=1.0, y=0.9):
    fig.text(0.05, y, text, fontsize=40, weight="bold", alpha=a)
    if sub:
        fig.text(0.05, y - 0.05, sub, fontsize=22, color=DIM, alpha=a)


def footer(fig, text):
    fig.text(0.05, 0.035, text, fontsize=15, color=DIM)


# ---------------------------------------------------------------- scene 1: title
def scene_title(w, secs=5):
    n = int(secs * FPS)
    for i in range(n):
        t = i / FPS
        def draw(fig):
            a = ease(t / 1.0) * ease((secs - t) / 0.6)
            fig.text(0.5, 0.58, "plutoSDR_FPGA_channelizer", ha="center", fontsize=72, weight="bold", alpha=a)
            fig.text(0.5, 0.48, "A polyphase filter bank inside a $200 ADALM-Pluto.", ha="center",
                     fontsize=30, color=DIM, alpha=a)
            fig.text(0.5, 0.42, "2 to 256 channels, chosen at runtime. Stream only the ones you want.",
                     ha="center", fontsize=30, color=DIM, alpha=a * ease((t - 0.8) / 1.0))
            fig.text(0.5, 0.18, "Every signal in this video is a real capture from the firmware.", ha="center",
                     fontsize=20, color=ACC[0], alpha=a * ease((t - 1.8) / 1.0))
        w.frame(draw)


# ---------------------------------------------------------- scene 2: spectrum tour
def scene_tour(w, per=1.6):
    bands = [("b71", "600 MHz (Band 71)"), ("b12_13", "700 MHz (Bands 12/13)"),
             ("b14_850", "750 MHz (Bands 13/14)"), ("cell850", "850 MHz cellular"),
             ("pcs", "1.9 GHz PCS"), ("aws", "2.1 GHz AWS")]
    data = [(np.load(CAP / f"survey_{b}.npz"), lab) for b, lab in bands]
    n = int(per * FPS)
    for k, (d, lab) in enumerate(data):
        f, pw = d["f"] / 1e6, 10 * np.log10(d["pw"] / 2048**2)
        for i in range(n):
            g = ease(i / (0.6 * FPS))
            def draw(fig):
                title(fig, "Spectrum tour", "64 channels x 400 kHz per band, both receivers, measured channel by channel")
                ax = fig.add_axes([0.07, 0.14, 0.88, 0.62])
                m = int(np.ceil(g * len(f)))
                for r, c in ((0, ACC[0]), (1, ACC[1])):
                    ax.bar(f[:m], pw[r, :m] + 80, bottom=-80, width=0.38, color=c, alpha=0.55 if r else 0.85,
                           label=f"RX{r + 1}")
                ax.set_xlim(f[0] - 0.4, f[-1] + 0.4); ax.set_ylim(-80, -10)
                ax.set_xlabel("MHz", fontsize=16); ax.set_ylabel("dBFS per channel", fontsize=16)
                ax.tick_params(labelsize=14); ax.legend(loc="upper right", fontsize=14, frameon=False)
                ax.text(0.01, 0.92, lab, transform=ax.transAxes, fontsize=26, weight="bold", color=ACC[2])
                footer(fig, "Each band: one runtime configuration, 8 channels streamed at a time")
            w.frame(draw)


# ------------------------------------------------------------- scene 3: 12 x FM
def scene_fm(w, secs=15.0):
    d = np.load(CAP / "fm12.npz")
    st, audio, spec = d["stations"], d["audio"], d["spec"]          # spec: (600, 12, 512)
    ns = len(st)
    per = secs / ns
    a_rms = np.array([np.sqrt(np.convolve(a ** 2, np.ones(4800) / 4800, "same")) for a in audio])
    sdb = 10 * np.log10(spec + 1e-3)
    sdb -= np.percentile(sdb, 20)
    n = int(secs * FPS)
    for i in range(n):
        t = i / FPS
        on = min(ns - 1, int(t / per))
        srow = min(len(spec) - 1, int(t / 0.025))
        arow = min(audio.shape[1] - 1, int(t * 48000))
        def draw(fig):
            title(fig, "12 FM stations. One Pluto. All at once.",
                  "128 x 200 kHz bank at 25.6 MS/s; 12 channels streamed (19.2 MB/s) and demodulated live")
            for j in range(ns):
                r, c = divmod(j, 4)
                x0, y0 = 0.05 + c * 0.228, 0.53 - r * 0.2
                ax = fig.add_axes([x0, y0, 0.21, 0.16])
                fz = np.linspace(-100, 100, 512)
                col = ACC[2] if j == on else ACC[0]
                ax.fill_between(fz, sdb[srow, j], -5, color=col, alpha=0.9 if j == on else 0.45, lw=0)
                ax.set_xlim(-100, 100); ax.set_ylim(-5, 45); ax.set_xticks([]); ax.set_yticks([])
                for s in ax.spines.values():
                    s.set_color(ACC[2] if j == on else "#2a3242"); s.set_linewidth(3 if j == on else 1)
                ax.text(0.03, 0.8, f"{st[j]:.1f} MHz", transform=ax.transAxes, fontsize=20, weight="bold",
                        color=FG if j == on else DIM)
                lv = min(1, a_rms[j, arow] * 2.2)
                ax.add_patch(Rectangle((0.03, 0.08), 0.94 * lv, 0.06, transform=ax.transAxes,
                                       color=col, alpha=0.9))
                if j == on:
                    ax.text(0.97, 0.8, "LISTENING", transform=ax.transAxes, ha="right", fontsize=13,
                            color=ACC[2], weight="bold")
            footer(fig, "Real off-air audio from each channel; the soundtrack steps through the stations. "
                        "Bars: each station's audio level, all live at once.")
        w.frame(draw)
    return fm_soundtrack(secs)


def fm_soundtrack(secs=15.0):
    """Each station's off-air audio in turn, with short crossfades."""
    d = np.load(CAP / "fm12.npz")
    audio = d["audio"]
    ns = len(audio)
    per = secs / ns
    sr = 48000
    out = np.zeros(int(secs * sr), np.float32)
    seg, fade = int(per * sr), int(0.08 * sr)
    for j in range(ns):
        a0 = j * seg
        x = audio[j][a0:a0 + seg + fade]
        x = x / (4 * np.std(audio[j]) + 1e-9)
        env = np.ones(len(x)); env[:fade] = np.linspace(0, 1, fade); env[-fade:] = np.linspace(1, 0, fade)
        out[a0:a0 + len(x)] += (x * env * 0.5).astype(np.float32)[: len(out) - a0]
    return np.clip(out, -1, 1)


# ------------------------------------------------------------- scene 4: LTE
def scene_lte(w, secs=14.0):
    b, t, m = load("lte_b13_b14")
    fs, k = 1.5e6, 2
    blocks = b.reshape(len(b), -1, k, 4).astype(np.float32) / 8
    nn = np.arange(blocks.shape[1])
    rot = np.exp(-2j * np.pi * 10e3 * nn / fs)
    corr = []                # per block: {freq: (nid2, curve)}
    for i in range(len(b)):
        row = {}
        for s in range(k):
            x = (blocks[i, :, s, 0] + 1j * blocks[i, :, s, 1]) * rot
            c = lte.pss_search(x, fs)
            nid2 = max(c, key=lambda q: c[q].max())
            row[nid2] = c[nid2]
        corr.append(row)
    cells = {0: ("751 MHz", "Band 13", 294, ACC[0]), 1: ("763 MHz", "Band 14 / FirstNet", 28, ACC[1])}
    sv = np.load(CAP / "survey_b14_850.npz"); sv2 = np.load(CAP / "survey_b12_13.npz")
    f = np.r_[sv2["f"], sv["f"]] / 1e6; p = 10 * np.log10(np.r_[sv2["pw"][0], sv["pw"][0]] / 2048**2)
    o = np.argsort(f); f, p = f[o], p[o]
    sel = (f > 744) & (f < 770)
    n = int(secs * FPS)
    for i in range(n):
        t_ = i / FPS
        bi = min(len(b) - 1, int(t_ / secs * len(b) * 1.0))
        reveal = ease((t_ - 5.0) / 1.5)
        def draw(fig):
            title(fig, "Two LTE cells, decoded in parallel",
                  "16 x 1.5 MHz bank at 24 MS/s; two channels on two carriers 12 MHz apart")
            ax = fig.add_axes([0.06, 0.48, 0.42, 0.3])
            ax.bar(f[sel], p[sel] + 80, bottom=-80, width=0.38, color="#3a4458")
            for nid2, (fl, band, pci, c) in cells.items():
                fc = float(fl.split()[0])
                ax.axvspan(fc - 0.75, fc + 0.75, color=c, alpha=0.35)
                ax.text(fc, -14, fl, ha="center", fontsize=16, color=c, weight="bold")
            ax.set_xlim(744, 770); ax.set_ylim(-80, -10); ax.tick_params(labelsize=13)
            ax.set_xlabel("MHz", fontsize=14); ax.set_ylabel("dBFS / 400 kHz", fontsize=14)
            ax.set_title("Band survey with the two selected 1.5 MHz channels", fontsize=16, color=DIM)
            for j, (nid2, (fl, band, pci, c)) in enumerate(cells.items()):
                ax2 = fig.add_axes([0.55, 0.52 - j * 0.33, 0.4, 0.24])
                cc = corr[bi].get(nid2, np.zeros(10))
                tt = np.arange(len(cc)) / fs * 1e3
                ax2.plot(tt, cc, color=c, lw=1.6)
                ax2.set_ylim(0, 1); ax2.set_xlim(0, tt[-1] if len(tt) > 1 else 1)
                ax2.set_xlabel("ms within capture block", fontsize=13); ax2.tick_params(labelsize=12)
                ax2.set_ylabel("PSS corr.", fontsize=13)
                ax2.text(0.01, 1.05, f"{fl}  {band}", transform=ax2.transAxes, fontsize=18, color=c, weight="bold")
                ax2.text(0.99, 1.05, f"N_ID2 = {nid2}   peaks every 5.000 ms", transform=ax2.transAxes,
                         ha="right", fontsize=14, color=DIM)
                if reveal > 0:
                    fig.text(0.06, 0.33 - j * 0.1, f"Cell {pci}", fontsize=44, weight="bold", color=c, alpha=reveal)
                    fig.text(0.21, 0.335 - j * 0.1, f"{band} - {fl}  (PSS + SSS)", fontsize=20, color=FG, alpha=reveal)
            footer(fig, "Physical cell IDs from the sync signals; both antennas agree in every block")
        w.frame(draw)


# ------------------------------------------------------------- scene 5: BLE
def mask_mac(a):
    return a[:8] + ":xx:xx:xx" if a else ""


def scene_ble(w, secs=15.0):
    b, t, m = load("ble_long")
    d = pickle.load(open(CAP / "ble_long_decoded.pkl", "rb"))
    adv, data = d["adv"], d["data"]
    link = {i for i, s, aa in data if aa == 0x331A3AE2}
    advb = sorted({i for i, s, info in adv})
    # time-compress: walk the blocks that have something on either channel
    show = sorted(set(advb) | link)
    k = 2
    events = []
    for i, s, info in adv:
        nm = info.get("name") or info.get("maker") or ""
        events.append((i, f"ch37  {info['type']:<10s} {mask_mac(info.get('adva', '')):<17s} {nm}", ACC[0]))
    for i, s, aa in data:
        if aa == 0x331A3AE2:
            events.append((i, f"2404  GFSK 1 Mb/s link   addr 0x331A3AE2", ACC[3]))
    events.sort()
    n = int(secs * FPS)
    nfft = 64
    def spec(z):
        z = z[: len(z) // nfft * nfft].reshape(-1, nfft) * np.hanning(nfft)
        return 10 * np.log10(np.abs(np.fft.fftshift(np.fft.fft(z, axis=1), axes=1)) ** 2 + 1e-3)
    for i in range(n):
        pos = min(len(show) - 1, int(i / n * len(show)))
        bi = show[pos]
        w_ = b[bi].reshape(-1, k, 4).astype(np.float32) / 8
        z = w_[..., 0] + 1j * w_[..., 1]
        # label slots by content: the channel-37 slot is the one with advertising packets
        a0 = ble.adv_packets(z[:, 0]) if bi in advb else []
        sa = 0 if (a0 or bi not in advb) else 1
        s37, s04 = spec(z[:, sa]), spec(z[:, 1 - sa])
        log = [e for e in events if e[0] <= bi and e[1].startswith("ch37")][-10:]
        nlink = sum(1 for e in events if e[0] <= bi and not e[1].startswith("ch37"))
        def draw(fig):
            title(fig, "Bluetooth: advertising + data channel at once",
                  "8 x 2 MHz bank at 16 MS/s; 2402 MHz (adv. channel 37) and 2404 MHz (data channel 0)")
            for j, (S, lab, c) in enumerate(((s37, "2402 MHz - BLE advertising ch. 37", ACC[0]),
                                            (s04, "2404 MHz - data channel 0", ACC[3]))):
                ax = fig.add_axes([0.05 + j * 0.24, 0.12, 0.22, 0.62])
                ax.imshow(S, aspect="auto", origin="lower", cmap="magma", vmin=np.percentile(S, 50),
                          vmax=np.percentile(S, 50) + 35, extent=[-1, 1, 0, len(S) * nfft / 2e3])
                ax.set_xlabel("MHz from channel centre", fontsize=13); ax.tick_params(labelsize=12)
                if j == 0:
                    ax.set_ylabel("ms", fontsize=13)
                ax.set_title(lab, fontsize=15, color=c)
            fig.text(0.55, 0.74, "Channel 37: advertising packets, decoded + CRC-checked", fontsize=19, weight="bold",
                     color=ACC[0])
            for r, (_, txt, c) in enumerate(log):
                fig.text(0.55, 0.69 - r * 0.038, txt[6:], fontsize=16, family="DejaVu Sans Mono", color=c,
                         alpha=0.45 + 0.55 * (r == len(log) - 1))
            fig.text(0.55, 0.25, "2404 MHz: 1 Mb/s GFSK link, address 0x331A3AE2", fontsize=19, weight="bold",
                     color=ACC[3])
            fig.text(0.55, 0.19, f"{nlink} bursts so far, one every ~30 ms, same address every time",
                     fontsize=17, color=FG)
            footer(fig, f"{len(adv)} advertising packets from {len({a.get('adva') for _, _, a in adv if 'adva' in a})} devices "
                        f"in 21 s; device addresses masked")
        w.frame(draw)


# ---------------------------------------------- scene: BLE, far-apart channels
def scene_ble_far(w, secs=15.0):
    d = np.load(CAP / "ble_far_iq.npz")
    z0, z7, f0, f7, all_f = d["z0"], d["z7"], float(d["f0"]), float(d["f7"]), d["all_f"]
    cap = float(d["secs"])
    adv = [(s / 2e6, info) for s, info in ble.adv_packets(z0, 37)]
    trains = [s / 2e6 for s, aa in ble.data_packets(z7) if aa in (0x3F8EC52F, 0xC0713AD0, 0x75A0AB41)]
    nfft, fs = 64, 2e6
    def specgram(z):
        n = len(z) // nfft * nfft
        S = np.fft.fftshift(np.fft.fft(z[:n].reshape(-1, nfft) * np.hanning(nfft), axis=1), axes=1)
        return (10 * np.log10(np.abs(S) ** 2 + 1e-3)).astype(np.float32)
    S0, S7 = specgram(z0), specgram(z7)
    lo0, lo7 = np.percentile(S0, 50), np.percentile(S7, 50)
    win = int(0.05 * fs / nfft)                       # 50 ms of waterfall
    n = int(secs * FPS)
    for i in range(n):
        tc = 0.05 + i / n * (cap - 0.051)
        r1 = int(tc * fs / nfft)
        log = [(t, f"{info['type']:<10s} {mask_mac(info.get('adva', '')):<17s} {info.get('name') or info.get('maker') or ''}")
               for t, info in adv if t <= tc and info["type"] != "SCAN_REQ"][-9:]
        ntr = sum(1 for t in trains if t <= tc)
        def draw(fig):
            title(fig, "Bluetooth: two channels 14 MHz apart, at once",
                  "8 x 2 MHz bank at 16 MS/s; only 2402 and 2416 MHz leave the FPGA (8-bit, 8 MB/s, continuous)")
            # band strip
            for j, fc in enumerate(all_f):
                x = 0.05 + j * 0.112
                lit = fc in (f0, f7)
                c = ACC[0] if fc == f0 else ACC[3] if fc == f7 else "#1b2232"
                fig.patches.append(Rectangle((x, 0.71), 0.104, 0.065, transform=fig.transFigure, fc=c,
                                             alpha=0.9 if lit else 1, ec="#2a3242"))
                fig.text(x + 0.052, 0.742, f"{fc / 1e6:.0f}", ha="center", va="center", fontsize=17,
                         color=BG if lit else DIM, weight="bold" if lit else None)
            fig.text(0.05, 0.69, "MHz:  six channels in between are dropped inside the FPGA, never sent",
                     fontsize=15, color=DIM)
            fig.patches.append(Rectangle((0.102, 0.786), 7 * 0.112, 0.004, transform=fig.transFigure, fc=FG, alpha=0.6))
            fig.text(0.5, 0.80, "14 MHz apart", ha="center", fontsize=17, color=FG)
            for j, (S, lo, lab, c) in enumerate(((S0, lo0, "2402 MHz  BLE advertising ch. 37", ACC[0]),
                                                (S7, lo7, "2416 MHz  BLE data channel 6", ACC[3]))):
                ax = fig.add_axes([0.05 + j * 0.24, 0.1, 0.22, 0.5])
                ax.imshow(S[max(0, r1 - win):r1], aspect="auto", origin="lower", cmap="magma", vmin=lo,
                          vmax=lo + 35, extent=[-1, 1, -50, 0], interpolation="antialiased")
                ax.set_xlabel("MHz from channel centre", fontsize=13); ax.tick_params(labelsize=12)
                if j == 0:
                    ax.set_ylabel("ms", fontsize=13)
                ax.set_title(lab, fontsize=15, color=c)
            fig.text(0.55, 0.6, "Ch. 37: advertising packets, decoded + CRC-checked", fontsize=18, weight="bold",
                     color=ACC[0])
            for r, (_, txt) in enumerate(log):
                fig.text(0.55, 0.555 - r * 0.036, txt, fontsize=15, family="DejaVu Sans Mono", color=ACC[0],
                         alpha=0.45 + 0.55 * (r == len(log) - 1))
            fig.text(0.55, 0.2, "2416 MHz: 1 Mb/s GFSK packet trains", fontsize=18, weight="bold", color=ACC[3])
            fig.text(0.55, 0.155, f"{ntr} packets so far, 883 us apart, a train every ~115 ms", fontsize=16, color=FG)
            footer(fig, f"{len(adv)} advertising packets in {cap:.0f} s of continuous capture (scan requests counted, not listed); "
                        "addresses masked")
        w.frame(draw)


# --------------------------------------------- scene: decoded on the Pluto
def scene_device(w, secs=13.0):
    d = pickle.load(open(CAP / "ble_on_device.pkl", "rb"))
    pk = [(t, ble.parse_adv(b[2:])) for t, ch, s, b in d["pk"]]
    span = d["secs"]
    n = int(secs * FPS)
    rows = [("Raw IQ, 2 BLE channels", 32e6, "#3a4458"), ("Packets decoded on the Pluto", d["usb"], ACC[0]),
            ("Raw IQ, 12 FM stations", 19.2e6, "#3a4458"), ("Audio demodulated on the Pluto", 1.22e6, ACC[2])]
    for i in range(n):
        t = i / FPS
        tc = t / secs * span
        log = [f"{p['type']:<10s} {mask_mac(p.get('adva', '')):<17s} {p.get('name') or p.get('maker') or ''}"
               for tt, p in pk if tt <= tc][-8:]
        npk = sum(1 for tt, _ in pk if tt <= tc)
        def draw(fig):
            title(fig, "Decode on the Pluto itself",
                  "Small apps run on the Pluto's ARM cores and send results, not samples. Copy a binary or a .py, run it.")
            ax = fig.add_axes([0.23, 0.36, 0.28, 0.34])
            for r, (lab, v, c) in enumerate(rows):
                g = ease((t - 0.4 * r) / 0.8)
                ax.barh(3 - r, (np.log10(v) - 2) * g, left=2, color=c, height=0.62)
                lbl = f"{v / 1e6:.1f} MB/s" if v >= 1e5 else f"{v / 1e3:.2f} kB/s"
                ax.text(2 + (np.log10(v) - 2) * g + 0.1, 3 - r, lbl, va="center", fontsize=16, weight="bold",
                        color=c if c != "#3a4458" else DIM, alpha=g)
            ax.set_yticks([3, 2, 1, 0]); ax.set_yticklabels([r[0] for r in rows], fontsize=15, color=FG)
            ax.set_xlim(2, 9.4); ax.set_xticks([3, 5, 7])
            ax.set_xticklabels(["1 kB/s", "100 kB/s", "10 MB/s"], fontsize=12)
            ax.set_title("Over USB", fontsize=16, color=DIM)
            fig.text(0.07, 0.27, "12 FM stations to audio on the Pluto: 100% real time, both cores", fontsize=16, color=ACC[2])
            fig.text(0.07, 0.22, "BLE GFSK demod + CRC on the Pluto: only valid packets cross USB", fontsize=16, color=ACC[0])
            fig.text(0.55, 0.7, f"bleadv on the Pluto:  {npk} packets", fontsize=19, weight="bold", color=ACC[0])
            for r, txt in enumerate(log):
                fig.text(0.55, 0.65 - r * 0.04, txt, fontsize=15, family="DejaVu Sans Mono", color=ACC[0],
                         alpha=0.45 + 0.55 * (r == len(log) - 1))
            fig.text(0.55, 0.2, 'pfb.run_app("apps/build/bleadv", "--chan", "37,6")', fontsize=17,
                     family="DejaVu Sans Mono", color=ACC[2])
            footer(fig, "Same apps in Python: pfb.run_app(\"fm_audio.py\") - edit over ssh, run again")
        w.frame(draw)


# ------------------------------------------------------------- scene 6: how
def scene_how(w, secs=10.0):
    code = ['from pluto_channelizer import PlutoChannelizer', '',
            'with PlutoChannelizer(nch=6, spacing=400e3,', '                      bandwidth=300e3, lo=100e6) as pfb:',
            '    pfb.select([0, 2, 5])', '    x = pfb.read(400_000)    # RX1 + RX2, per channel', '',
            '    with pfb.run_app("fm_audio.py") as app:   # or a C binary', '        for msg in app.messages(): ...']
    stages = ["AD9363\nboth RX", "polyphase FIR\n8 taps / phase", "FFT\n2 - 256 points", "channel\nmask", "USB\nonly what you pick"]
    n = int(secs * FPS)
    for i in range(n):
        t = i / FPS
        def draw(fig):
            title(fig, "How it works")
            for j, s in enumerate(stages):
                a = ease((t - 0.3 * j) / 0.5)
                x = 0.05 + j * 0.185
                fig.patches.append(FancyBboxPatch((x, 0.6), 0.16, 0.14, boxstyle="round,pad=0.01",
                                                  transform=fig.transFigure, fc="#151c29", ec=ACC[j % 6], lw=2, alpha=a))
                fig.text(x + 0.08, 0.67, s, ha="center", va="center", fontsize=19, alpha=a)
            for r, line in enumerate(code):
                fig.text(0.07, 0.5 - r * 0.045, line, fontsize=21, family="DejaVu Sans Mono", color=ACC[2],
                         alpha=ease((t - 1.8 - 0.15 * r) / 0.5))
            a = ease((t - 4) / 0.8)
            fig.text(0.07, 0.08, "Set at runtime: channels 2-256, spacing, bandwidth, taps.",
                     fontsize=20, color=DIM, alpha=a)
            credit = _os.environ.get("PFB_MUSIC_CREDIT", "Music: Tokyo Music Walker - Way Home (BreakingCopyright)")
            fig.text(0.95, 0.03, credit, ha="right", fontsize=13, color=DIM, alpha=a)
        w.frame(draw)


SCENES = (("01_title", scene_title, 5), ("02_fm", scene_fm, 15.0), ("03_ble", scene_ble_far, 15.0),
          ("04_lte", scene_lte, 14.0), ("05_device", scene_device, 13.0), ("06_how", scene_how, 10.0))
import os as _os
MUSIC = HERE / "music" / _os.environ.get("PFB_MUSIC", "yt_Q7HjxOAU5Kc/track.opus")   # see music/SOURCE.txt


def music(sr, secs, start=None):
    """`secs` of the music track from `start` (default: its first audible
    0.1 s), stereo float at `sr`."""
    if start is None:
        lead = music(sr, 10, 0.0)
        env = np.sqrt((lead ** 2).mean(1)[: len(lead) // 4800 * 4800].reshape(-1, 4800).mean(1))
        start = int(np.argmax(env > 0.01)) / 10
    r = subprocess.run(["ffmpeg", "-loglevel", "error", "-ss", f"{start:.3f}", "-i", str(MUSIC), "-t", f"{secs:.3f}", "-f", "f32le",
                        "-ac", "2", "-ar", str(sr), "-"], capture_output=True, check=True, env=FFENV)
    return np.frombuffer(r.stdout, np.float32).reshape(-1, 2).copy()


def soundtrack(sr=48000, music_db=-24.0, fm_below_db=5.0, fade=2.0, music_start=22.0):
    """The music track under the whole video, from music_start (where "Way Home"
    leaves its quiet intro), with the FM stations layered on top during the FM
    scene, fm_below_db under the music playing there; fades out at the end."""
    durs = [int(secs * FPS) / FPS for _, _, secs in SCENES]
    starts = np.cumsum([0] + durs)
    total = int(round(starts[-1] * sr))
    fm_i = [n for n, _, _ in SCENES].index("02_fm")
    a, b = int(round(starts[fm_i] * sr)), int(round(starts[fm_i + 1] * sr))
    m = music(sr, total / sr + 1, music_start)[:total]
    # A few dB under the stations (~-19 dB RMS), peaks kept below 0.9.
    m *= min(10 ** (music_db / 20) / (np.sqrt((m ** 2).mean()) + 1e-9), 0.9 / (np.abs(m).max() + 1e-9))
    nf = int(fade * sr)
    m[-nf:] *= np.linspace(1, 0, nf)[:, None]
    out = np.zeros((total, 2), np.float32)
    out[:len(m)] += m
    fm = fm_soundtrack(durs[fm_i])[: b - a]
    under = np.sqrt((m[a:b] ** 2).mean())                         # the music during the FM scene
    fm *= under * 10 ** (-fm_below_db / 20) / (np.sqrt((fm ** 2).mean()) + 1e-9)
    out[a:a + len(fm)] += fm[:, None]
    return np.clip(out, -1, 1)


def main(only=None):
    """Render the scenes (or only the named ones; the rest are reused from out/),
    then the soundtrack, then mux."""
    from scipy.io import wavfile
    OUT.mkdir(exist_ok=True)
    parts = []
    for name, fn, secs in SCENES:
        path = OUT / f"{name}.mp4"
        if only is None or name in only or not path.exists():
            w = Writer(path)
            fn(w)
            w.close()
            print("rendered", path)
        parts.append(path)
    sr = 48000
    wavfile.write(OUT / "soundtrack.wav", sr, (soundtrack(sr) * 32767).astype(np.int16))
    (OUT / "parts.txt").write_text("".join(f"file '{p}'\n" for p in parts))
    final = OUT / "plutosdr_fpga_channelizer.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(OUT / "parts.txt"),
                    "-i", str(OUT / "soundtrack.wav"), "-c:v", "copy", "-c:a", "aac", "-b:a", "192k",
                    "-movflags", "+faststart", "-shortest", str(final)], check=True, env=FFENV)
    print("final", final)


if __name__ == "__main__":
    import sys
    main(set(sys.argv[1:]) or None)
