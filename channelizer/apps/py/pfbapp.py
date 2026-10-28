"""Python channel apps: the same stream as apps/src/pfbapp.c, in numpy.

    from pfbapp import App, AUDIO
    app = App()                                   # parses --port/--slots/--rate/--frames
    app.meta(app="mine", rate=app.rate)
    for first, x in app.blocks():                 # x: (frames, slots, 4) int16
        z = app.iq(x, slot=0, rx=0)               # complex64 for one channel
        app.send(AUDIO, slot=0, sample=first, payload=some_int16_array)

On the Pluto, PlutoChannelizer.run_app("app.py") copies this file and the app
to /tmp and runs them with python3 (firmware with Python). On a PC, run_app(...,
local=True) runs the same script here against the Pluto over the network
(--uri ip:192.168.2.1), which is the quickest way to develop one.
"""
import argparse
import json
import signal
import socket
import struct
import sys

import numpy as np
import iio

META, IQ16, IQ8, IQ4, IQ1, AUDIO, PACKET, TEXT = range(8)
HDR = struct.Struct("<HBBHHIQ")


class App:
    def __init__(self, argv=None):
        p = argparse.ArgumentParser()
        p.add_argument("--port", type=int, default=30500)
        p.add_argument("--slots", type=int, required=True)
        p.add_argument("--rate", type=float, required=True)
        p.add_argument("--frames", type=int, default=0)
        p.add_argument("--uri", default=None, help="run off-board against ip:<pluto>")
        self.opts, rest = p.parse_known_args(argv)
        self.args = rest                         # the app's own arguments
        self.slots, self.rate = self.opts.slots, self.opts.rate
        self.frames = self.opts.frames or max(256, int(self.rate / 100))
        self.stop = False
        signal.signal(signal.SIGTERM, self._on_signal)
        signal.signal(signal.SIGINT, self._on_signal)
        self.ctx = iio.Context(self.opts.uri) if self.opts.uri else iio.LocalContext()
        self.dev = self.ctx.find_device("cf-ad9361-lpc")
        for i in range(4):
            self.dev.find_channel(f"voltage{i}").enabled = True
        srv = socket.socket()
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("", self.opts.port))
        srv.listen(1)
        print(f"pfbapp: listening on {self.opts.port}", file=sys.stderr, flush=True)
        self.sock, _ = srv.accept()
        srv.close()
        self.buf = iio.Buffer(self.dev, self.frames * self.slots)
        self.seq = 0
        self.frame = 0

    def _on_signal(self, *_):
        self.stop = True

    def arg(self, name, default=None, type=str):
        """The app's own --name value."""
        if name in self.args and self.args.index(name) + 1 < len(self.args):
            return type(self.args[self.args.index(name) + 1])
        return default

    def blocks(self):
        while not self.stop:
            self.buf.refill()
            x = np.frombuffer(self.buf.read(), np.int16).reshape(-1, self.slots, 4)
            first = self.frame
            self.frame += len(x)
            yield first, x

    @staticmethod
    def iq(x, slot, rx=0):
        """One channel's samples as complex64 in ADC-code units."""
        v = x[:, slot, 2 * rx:2 * rx + 2].astype(np.float32) / 8.0
        return v[:, 0] + 1j * v[:, 1]

    def send(self, type, slot=0xFFFF, sample=0, payload=b"", rx=0):
        if isinstance(payload, np.ndarray):
            payload = payload.tobytes()
        elif isinstance(payload, str):
            payload = payload.encode()
        try:
            self.sock.sendall(HDR.pack(0x4250, type, rx, slot, self.seq & 0xFFFF, len(payload), sample) + payload)
        except OSError:
            self.stop = True
        self.seq += 1

    def meta(self, **fields):
        fields.setdefault("slots", self.slots)
        self.send(META, payload=json.dumps(fields))

    def close(self):
        self.buf = None                      # release the DMA buffer before the context
        self.sock.close()
