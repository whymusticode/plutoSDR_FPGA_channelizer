"""Host interface to the dual-RX PFB channelizer firmware (project/fpga_channelize).

    from pluto_channelizer import PlutoChannelizer

    with PlutoChannelizer(nch=6, spacing=400e3, bandwidth=300e3, lo=100e6) as pfb:
        pfb.select([0, 2, 5])                    # or pfb.select(freqs=[99.2e6, 100.4e6])
        x = pfb.read(400_000)                    # {channel: (2, n) complex64}, RX1/RX2

Channels are 0..nch-1, lowest frequency first, `spacing` apart, centred at
lo + (k - nch//2) * spacing; each comes out at `spacing` samples/s (critically
sampled). `bandwidth` is the band around each centre kept free of aliases:
the prototype passes +-bandwidth/2 and stops from spacing - bandwidth/2.

The FFT runs at n = nch rounded up to a power of two (2..256), and the radio at
fs = n * spacing; the n - nch extra channels sit at the band edges, unused.
The host designs the prototype (filter/pfb_filter.py) and loads it.

nch=1 is raw mode: the bank is bypassed and both receivers stream as the radio
delivers them, at fs = spacing (up to 30.72 MS/s), centred on lo; bandwidth
and taps do not apply. Firmware from before the bypass bit refuses it.

Firmware facts this relies on (see ../README.md and hdl/pfb_ctrl.v):
* Control registers at 0x41210000 (hdl/pfb_ctrl.v) with no kernel driver, so
  they are written with devmem over ssh; a reconfiguration is one ssh call.
* The RX DMA carries one word per selected channel per frame of n samples,
  {rx1 i, rx1 q, rx2 i, rx2 q} int16 at Q18 (8x ADC-code scale), in
  bit-reversed FFT order, with no frame marker.
"""
import os
import stat
import subprocess
import sys
import tempfile
import warnings
from pathlib import Path

import iio
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "filter"))
import pfb_filter  # noqa: E402

REG_BASE = 0x41210000
REG_ID, REG_BUILD, REG_CONTROL, REG_STATUS = 0x00, 0x04, 0x08, 0x0C
REG_COEF_IDX, REG_COEF_DATA, REG_COMMIT, REG_MASK = 0x10, 0x14, 0x18, 0x40
FW_ID = 0x50464232       # "PFB2"
N_MAX, TAPS_MAX = 256, 8
OUT_SCALE = 8.0          # Q18 output -> 12-bit ADC-code units
LINK_BYTES_PER_S = 16e6  # measured network-backend ceiling, roughly
# Sustained input rate per FFT size at the 200 MHz fabric clock, from the RTL
# simulation (test/sim/check_rtl.py); past it the ingress FIFO drops samples.
FS_MAX = {2: 18.2e6, 4: 22.2e6, 8: 27.2e6, 16: 33.1e6, 32: 38.2e6,
          64: 38.5e6, 128: 36.9e6, 256: 35.9e6}
FS_RAW_MAX = 30.72e6     # the AD9361 with both receivers on
FS_MIN_NO_FIR = 2.0833e6  # below this the AD9363 needs its FIR decimation (not set up here)


def fft_size(nch):
    """FFT size for nch channels; 1 (no FFT) for raw mode."""
    if not 1 <= nch <= N_MAX:
        raise ValueError(f"nch must be 1..{N_MAX}")
    return 1 << (int(nch) - 1).bit_length()


class PlutoChannelizer:
    def __init__(self, nch=64, spacing=400e3, bandwidth=None, taps=TAPS_MAX, channels=None,
                 lo=90.7e6, rf_bandwidth=None, gains=(40, 40), uri="ip:192.168.2.1",
                 host=None, password="analog", timeout_ms=5000):
        self.nch, self.spacing = int(nch), float(spacing)
        self.n = fft_size(self.nch)
        self.raw = self.n == 1
        self.logn = max(1, self.n.bit_length() - 1)
        self.fs = self.n * self.spacing
        self.bandwidth = float(bandwidth if bandwidth is not None else spacing / 2)
        self.taps = int(taps)
        if not 1 <= self.taps <= TAPS_MAX:
            raise ValueError(f"taps must be 1..{TAPS_MAX}")
        if self.raw:
            if self.fs > FS_RAW_MAX:
                raise ValueError(f"raw mode at {self.fs/1e6:.2f} MS/s exceeds the radio's "
                                 f"{FS_RAW_MAX/1e6:.2f} MS/s with both receivers")
            self.bandwidth = self.spacing
            self.prototype = self._q = self.filter_metrics = None
        else:
            if self.fs > FS_MAX[self.n]:
                raise ValueError(f"{self.n}-point bank at fs = {self.n} x {spacing/1e3:g} kHz = "
                                 f"{self.fs/1e6:.2f} MS/s exceeds the FPGA's {FS_MAX[self.n]/1e6:.1f} MS/s")
            # Design and check the prototype before touching the radio.
            self.prototype = pfb_filter.design(self.n, self.spacing, self.bandwidth, self.taps)
            self._q = pfb_filter.phase_taps(self.prototype, self.n)
            self.filter_metrics = pfb_filter.metrics(self.prototype, self.bandwidth / 2, self.fs, self.n)

        self.uri = uri
        self.host = host or uri.split(":", 1)[1]
        self.password = password
        self.ctx = iio.Context(uri)
        self.ctx.set_timeout(timeout_ms)
        fw = self.ctx.attrs.get("fw_version", "")
        self.phy = self.ctx.find_device("ad9361-phy")
        self.rx = self.ctx.find_device("cf-ad9361-lpc")
        # Identify the build before touching the registers: on stock firmware
        # nothing answers at that address, and probing an absent PL slave can
        # fault the board. The channelizer images are built with the TX DDS
        # disabled, which the TX core reports in REG_CONFIG bit 6.
        tx = self.ctx.find_device("cf-ad9361-dds-core-lpc")
        if not tx.reg_read(0x0C) & 0x40:
            raise RuntimeError(f"fw {fw} is not the channelizer image (TX DDS present); not "
                               f"touching {REG_BASE:#x}")
        ident = self._reg(REG_ID)
        if ident is None:
            raise RuntimeError(f"cannot read {REG_BASE:#x} over ssh root@{self.host}")
        if ident != FW_ID:
            raise RuntimeError(f"register block id {ident:#010x}, want {FW_ID:#010x}: this is the "
                               "fixed 64-channel image (build-fixed64/) or another build")
        build = self._reg(REG_BUILD)
        if (build & 0xFF, (build >> 8) & 0xFF) != (N_MAX.bit_length() - 1, TAPS_MAX):
            raise RuntimeError(f"firmware build register {build:#x} does not match this host code")
        if self.raw and not build >> 24 & 1:
            raise RuntimeError("nch=1 (raw mode) needs firmware with the bypass bit; flash a newer image")

        self._lo = self.phy.find_channel("altvoltage0", True)
        self._rxc = [self.phy.find_channel(f"voltage{i}", False) for i in range(2)]
        self._set_rate(rf_bandwidth)
        self.lo = lo
        self.gains = gains
        for i in range(4):  # the stream always carries both receivers
            self.rx.find_channel(f"voltage{i}").enabled = True
        self._buf = None
        self.channels, self.order, self.rotation = [], [], 0
        self._spill = np.empty((0, 4), np.int16)
        self._configure()
        # By default stream every channel, when the link can carry them all;
        # otherwise wait for select().
        if channels is None and self.nch * self.spacing * 8 <= LINK_BYTES_PER_S:
            channels = range(self.nch)
        if channels is not None:
            self.select(channels)

    # ---- radio -------------------------------------------------------------------
    def _set_rate(self, rf_bandwidth):
        try:
            self._rxc[0].attrs["sampling_frequency"].value = str(int(round(self.fs)))
        except OSError as e:
            if self.fs < FS_MIN_NO_FIR:
                raise RuntimeError(f"the AD9363 refused fs = {self.fs/1e6:.3f} MS/s: below "
                                   f"{FS_MIN_NO_FIR/1e6:.3f} MS/s it needs its FIR decimation, which "
                                   f"this class does not set up yet; use spacing >= "
                                   f"{FS_MIN_NO_FIR/self.n/1e3:.1f} kHz") from e
            raise
        actual = float(self._rxc[0].attrs["sampling_frequency"].value)
        if abs(actual - self.fs) > 1e-4 * self.fs:
            warnings.warn(f"radio runs at {actual:.0f} S/s, asked {self.fs:.0f}")
        bw = rf_bandwidth if rf_bandwidth is not None else min(max(self.fs, 200e3), 20e6)
        self._rxc[0].attrs["rf_bandwidth"].value = str(int(bw))

    @property
    def lo(self):
        return float(self._lo.attrs["frequency"].value)

    @lo.setter
    def lo(self, hz):
        self._lo.attrs["frequency"].value = str(int(hz))

    @property
    def gains(self):
        return tuple(float(c.attrs["hardwaregain"].value.split()[0]) for c in self._rxc)

    @gains.setter
    def gains(self, db):
        for c, g in zip(self._rxc, db):
            c.attrs["gain_control_mode"].value = "manual"
            c.attrs["hardwaregain"].value = str(g)

    def channel_freq(self, k):
        return self.lo + (k - self.nch // 2) * self.spacing

    def channel_for(self, freq_hz):
        """Nearest channel to an RF frequency (warns when it sits near an edge)."""
        off = (freq_hz - self.lo) / self.spacing
        k = int(round(off)) + self.nch // 2
        if not 0 <= k < self.nch:
            raise ValueError(f"{freq_hz/1e6:.3f} MHz is outside the {self.nch} channels")
        if abs(off - round(off)) * self.spacing > self.bandwidth / 2:
            warnings.warn(f"{freq_hz/1e6:.3f} MHz is {abs(off - round(off))*self.spacing/1e3:.0f} kHz "
                          f"from channel {k}'s centre, outside its +-{self.bandwidth/2e3:g} kHz clean band")
        return k

    # ---- channel selection -----------------------------------------------------
    def _bin(self, k):
        """fftshift position in the n-point bank (n/2 is the LO) of channel k."""
        return self.n // 2 - self.nch // 2 + k

    def dma_order(self, channels):
        """Slot order: the selector emits channels in bit-reversed FFT order."""
        def position(k):
            natural = (self._bin(k) - self.n // 2) % self.n
            return int(f"{natural:0{self.logn}b}"[::-1], 2)
        return sorted(channels, key=position)

    def select(self, channels=None, freqs=None):
        if freqs is not None:
            channels = [self.channel_for(f) for f in freqs]
        channels = sorted({int(c) for c in channels})
        if not channels or channels[0] < 0 or channels[-1] >= self.nch:
            raise ValueError(f"select one or more channels in 0..{self.nch - 1}")
        rate = len(channels) * self.spacing * 8
        if rate > LINK_BYTES_PER_S:
            warnings.warn(f"{len(channels)} channels need {rate/1e6:.1f} MB/s; "
                          f"the link sustains about {LINK_BYTES_PER_S/1e6:.0f} MB/s, expect gaps")
        self._close_buffer()
        if self.raw:
            return self._open(channels, [0])
        bits = sum(1 << self._bin(k) for k in channels)
        words = [(bits >> (32 * w)) & 0xFFFFFFFF for w in range(N_MAX // 32)]
        a = lambda r: f"{REG_BASE + r:#x}"
        back = self._shell("\n".join([f"devmem {a(REG_MASK + 4 * w)} 32 {v}" for w, v in enumerate(words)]
                                     + [f"devmem {a(REG_COMMIT)} 32 1"]
                                     + [f"devmem {a(REG_MASK + 4 * w)} 32" for w in range(len(words))]))
        if [int(v, 16) for v in back.split()] != words:
            raise RuntimeError("mask readback mismatch")
        return self._open(channels, self.dma_order(channels))

    def _open(self, channels, order):
        self.channels, self.order, self.rotation = channels, order, 0
        # Whole frames per block, so blocks never split a frame and the discard
        # below keeps the slot alignment.
        k = len(self.order)
        self._block = k * max(256, min(16384, (1 << 19) // k))
        self._buf = iio.Buffer(self.rx, self._block)
        self._buf.refill()   # may straddle the mask change
        return self.order

    # ---- data --------------------------------------------------------------------
    def read(self, n):
        """n samples (at `spacing` S/s) per selected channel, contiguous with the previous read.

        Returns {channel: complex64 array (2, n)}, rows RX1 and RX2, in 12-bit ADC-code
        units (full scale 2048).
        """
        if self._buf is None:
            raise RuntimeError("select() channels first")
        k = len(self.order)
        need = n * k
        words = [self._spill]
        have = len(self._spill)
        while have < need:
            self._buf.refill()
            blk = np.frombuffer(self._buf.read(), np.int16).reshape(-1, 4).copy()
            words.append(blk)
            have += len(blk)
        w = np.concatenate(words)
        self._spill = w[need:]
        w = w[:need].reshape(n, k, 4).astype(np.float32) / OUT_SCALE
        w = np.roll(w, -self.rotation, axis=1)
        return {ch: np.stack([w[:, i, 0] + 1j * w[:, i, 1], w[:, i, 2] + 1j * w[:, i, 3]])
                for i, ch in enumerate(self.order)}

    def align(self, channel, rx=0, n=65536):
        """Fix the slot rotation using a channel known to be the strongest on `rx`.

        The stream has no frame marker; fresh buffers have started on slot 0 in
        every test so far, but that isn't guaranteed. Returns the rotation found.
        """
        if channel not in self.order:
            raise ValueError(f"channel {channel} is not selected")
        saved, self.rotation = self.rotation, 0
        x = self.read(n)
        power = [np.mean(np.abs(x[ch][rx]) ** 2) for ch in self.order]
        self.rotation = (int(np.argmax(power)) - self.order.index(channel)) % len(self.order)
        if self.rotation != saved:
            warnings.warn(f"slot rotation {self.rotation}; labels corrected from here on")
        return self.rotation

    def status(self, clear=True):
        """Sticky drop flags since the last clear: input FIFO (radio faster than
        the bank) and output FIFO (DMA slower than the selected channels)."""
        s = self._reg(REG_STATUS)
        if clear:
            self._reg(REG_STATUS, 3)
        return {"input_dropped": bool(s & 1), "output_dropped": bool(s & 2)}

    # ---- plumbing ----------------------------------------------------------------
    def _configure(self):
        """Size and taps in one ssh call: hold, load, release (pfb_ctrl.v). Raw
        mode instead holds the bank in reset and sets the bypass."""
        a = lambda r: f"{REG_BASE + r:#x}"
        if self.raw:
            control = 3 | self.logn << 8
            back = self._shell(f"devmem {a(REG_CONTROL)} 32 {control}\ndevmem {a(REG_CONTROL)} 32")
            if int(back, 16) != control:
                raise RuntimeError(f"bypass readback {back.strip()} mismatch")
            return
        lines = [f"devmem {a(REG_CONTROL)} 32 {1 | self.logn << 8}",
                 f"devmem {a(REG_COEF_IDX)} 32 0"]
        mask = (1 << pfb_filter.COEF_W) - 1
        lines += [f"devmem {a(REG_COEF_DATA)} 32 {int(v) & mask}" for v in self._q.reshape(-1)]
        lines += [f"devmem {a(REG_CONTROL)} 32 {self.logn << 8}",
                  f"devmem {a(REG_CONTROL)} 32", f"devmem {a(REG_COEF_IDX)} 32"]
        back = [int(v, 16) for v in self._shell("\n".join(lines)).split()]
        if back != [self.logn << 8, (self.n * TAPS_MAX) % 2048]:
            raise RuntimeError(f"channelizer configuration readback {back} mismatch")

    def _reg(self, offset, value=None):
        """Read (or write) one register. None if the read fails."""
        cmd = f"devmem {REG_BASE + offset:#x} 32" + ("" if value is None else f" {value}")
        try:
            out = self._shell(cmd)
        except RuntimeError:
            if value is not None:
                raise
            return None
        return int(out.strip(), 16) if value is None else value

    def _ssh_argv(self, command):
        """ssh argv and env (password via SSH_ASKPASS); the caller deletes env['SSH_ASKPASS']."""
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".sh") as f:
            f.write(f"#!/bin/sh\necho '{self.password}'\n")
        os.chmod(f.name, stat.S_IRWXU)
        env = dict(os.environ, SSH_ASKPASS=f.name, SSH_ASKPASS_REQUIRE="force", DISPLAY=":0")
        return ["ssh", "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
                "-o", "LogLevel=ERROR", "-o", "ConnectTimeout=5", f"root@{self.host}", command], env

    def _shell(self, script, data=None):
        """Run a shell script on the Pluto as root over one ssh connection, or, with
        `data` (bytes), run `script` as a command with `data` on its stdin."""
        argv, env = self._ssh_argv("sh -e" if data is None else script)
        try:
            r = subprocess.run(argv, input=(script + "\n").encode() if data is None else data,
                               env=env, capture_output=True, timeout=120)
        finally:
            os.unlink(env["SSH_ASKPASS"])
        if r.returncode != 0:
            raise RuntimeError(f"ssh/devmem failed: {r.stderr.decode(errors='replace').strip()}")
        return r.stdout.decode(errors="replace")

    # ---- on-Pluto apps -----------------------------------------------------------
    def run_app(self, path, *args, port=30500, frames=None, local=False):
        """Run a channel app on the Pluto's ARM against the current selection.

        `path` is a binary built from apps/ (or, on firmware with Python, a
        .py script using apps/pfbapp.py). It is copied to /tmp, started with
        the selection's slot count and rate, and connected to; returns an
        AppStream (see pfb_apps.py). The host's own IIO buffer is released
        first; select() again afterwards to stream raw IQ to the host.
        local=True runs a .py app on this machine against the Pluto instead,
        for developing it before it goes on the board.
        """
        from pfb_apps import start_app
        if not self.order:
            raise RuntimeError("select() channels first")
        self._close_buffer()
        return start_app(self, Path(path), list(map(str, args)), port, frames, local)

    def _close_buffer(self):
        self._buf = None
        self._spill = np.empty((0, 4), np.int16)

    def close(self):
        # libiio crashes if the context is released before its buffers.
        self._close_buffer()
        self.rx = self.phy = None
        self.ctx = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
