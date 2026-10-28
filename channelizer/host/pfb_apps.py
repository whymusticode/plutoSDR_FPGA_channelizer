"""Host side of the on-Pluto channel apps (apps/src/pfbapp.h describes the stream).

    with PlutoChannelizer(nch=128, spacing=200e3, lo=98.7e6, channels=[...]) as pfb:
        with pfb.run_app("../apps/build/fmaudio") as app:
            for m in app.messages():          # Message(type, channel, rx, sample, data)
                ...

AppStream.messages() yields decoded messages: IQ types as complex64 arrays in
the int16 stream's units (whatever the bit depth on the wire), audio as int16,
packets as bytes, meta as a dict (also kept in AppStream.meta).
"""
import json
import os
import select
import socket
import struct
import subprocess
import time
from collections import namedtuple

import numpy as np

HDR = struct.Struct("<HBBHHIQ")
META, IQ16, IQ8, IQ4, IQ1, AUDIO, PACKET, TEXT = range(8)
Message = namedtuple("Message", "type channel rx sample data")


def stop_cmd(stem):
    """Shell: stop the app recorded in its pidfile; SIGTERM lets it release the
    DMA buffer, SIGKILL after 3 s. Always succeeds."""
    pid = f"/tmp/pfb_{stem}.pid"
    return (f"if [ -f {pid} ]; then p=$(cat {pid}); kill $p 2>/dev/null || true; i=0; "
            f"while kill -0 $p 2>/dev/null && [ $i -lt 30 ]; do sleep 0.1; i=$((i+1)); done; "
            f"kill -9 $p 2>/dev/null || true; rm -f {pid}; fi; true")


def start_app(pfb, path, args, port, frames, local=False):
    if not path.exists():
        raise FileNotFoundError(path)
    if local:
        return _start_local(pfb, path, args, port, frames)
    remote = f"/tmp/pfb_{path.stem}"
    if path.suffix == ".py":
        lib = path.resolve().parent / "pfbapp.py"
        if not lib.exists():
            lib = path.resolve().parents[1] / "pfbapp.py"
        pfb._shell("cat > /tmp/pfbapp.py", lib.read_bytes())
        remote += ".py"
        run = f"python3 {remote}"
    else:
        run = remote
    # Stop any previous instance, then replace the file by rename: a running
    # binary cannot be overwritten in place ("Text file busy").
    pfb._shell(f"{stop_cmd(path.stem)}; cat > {remote}.new && chmod +x {remote}.new && "
               f"mv -f {remote}.new {remote}", path.read_bytes())
    # The launching shell records its pid, then becomes the app.
    cmd = (f"echo $$ > /tmp/pfb_{path.stem}.pid; "
           f"exec {run} --port {port} --slots {len(pfb.order)} --rate {pfb.spacing:.3f}")
    if frames:
        cmd += f" --frames {frames}"
    cmd += "".join(f" {a}" for a in args)
    argv, env = pfb._ssh_argv(cmd)
    proc = subprocess.Popen(argv, env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    try:
        t0, err = time.time(), b""
        while b"listening" not in err:
            if proc.poll() is not None or time.time() - t0 > 20:
                err += proc.stderr.read() if proc.poll() is not None else b""
                proc.kill()
                raise RuntimeError(f"{path.name} did not start: {err.decode(errors='replace').strip()}")
            if select.select([proc.stderr], [], [], 0.2)[0]:
                err += os.read(proc.stderr.fileno(), 4096)
    finally:
        os.unlink(env["SSH_ASKPASS"])
    sock = socket.create_connection((pfb.host, port), timeout=10)
    return AppStream(pfb, path.stem, proc, sock)


def _start_local(pfb, path, args, port, frames):
    """A .py app on this machine, against the Pluto over the network."""
    if path.suffix != ".py":
        raise ValueError("local=True runs Python apps only")
    import sys
    lib = path.resolve().parent
    argv = [sys.executable, str(path), "--uri", f"ip:{pfb.host}", "--port", str(port),
            "--slots", str(len(pfb.order)), "--rate", f"{pfb.spacing:.3f}"]
    if frames:
        argv += ["--frames", str(frames)]
    env = dict(os.environ, PYTHONPATH=f"{lib}{os.pathsep}{lib.parent / 'py'}{os.pathsep}{os.environ.get('PYTHONPATH', '')}")
    proc = subprocess.Popen(argv + args, env=env, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    t0, err = time.time(), b""
    while b"listening" not in err:
        if proc.poll() is not None or time.time() - t0 > 20:
            proc.kill()
            raise RuntimeError(f"{path.name} did not start: {(err + proc.stderr.read()).decode(errors='replace')}")
        if select.select([proc.stderr], [], [], 0.2)[0]:
            err += os.read(proc.stderr.fileno(), 4096)
    return AppStream(pfb, path.stem, proc, socket.create_connection(("127.0.0.1", port), timeout=10), local=True)


class AppStream:
    def __init__(self, pfb, name, proc, sock, local=False):
        self.pfb, self.name, self.proc, self.sock, self.local = pfb, name, proc, sock, local
        self.order = list(pfb.order)
        self.meta = {}
        self.bytes = 0
        self._f = sock.makefile("rb", buffering=1 << 20)

    def _read(self, n):
        b = self._f.read(n)
        if len(b) < n:
            raise EOFError(f"{self.name} closed the stream")
        self.bytes += n
        return b

    def messages(self):
        """Decoded messages until the app stops or close() is called."""
        while True:
            try:
                magic, typ, rx, slot, seq, ln, sample = HDR.unpack(self._read(HDR.size))
                if magic != 0x4250:
                    raise ValueError(f"bad message header from {self.name}")
                raw = self._read(ln)
            except (EOFError, OSError, ValueError):
                return
            ch = self.order[slot] if slot != 0xFFFF and slot < len(self.order) else None
            yield Message(typ, ch, rx, sample, self._decode(typ, raw, rx))

    def _decode(self, typ, raw, rx=0):
        if typ == META:
            self.meta = json.loads(raw)
            return self.meta
        if typ == IQ16:
            v = np.frombuffer(raw, np.int16).astype(np.float32) / 8.0
            if rx == 2:                          # both receivers, {i1, q1, i2, q2}
                v = v.reshape(-1, 4)
                return np.stack([v[:, 0] + 1j * v[:, 1], v[:, 2] + 1j * v[:, 3]])
            return v[0::2] + 1j * v[1::2]
        if typ in (IQ8, IQ4, IQ1):
            scale = np.frombuffer(raw[:4], np.float32)[0] / 8.0   # Q18 -> ADC codes
            b = np.frombuffer(raw[4:], np.uint8)
            if typ == IQ8:
                v = b.view(np.int8).astype(np.float32)
            elif typ == IQ4:
                v = np.stack([b & 0xF, b >> 4], 1).ravel().astype(np.int8)
                v = np.where(v > 7, v - 16, v).astype(np.float32)
            else:
                v = np.unpackbits(b, bitorder="little").astype(np.float32) * 2 - 1
            v = v[: len(v) // 2 * 2] * scale
            return v[0::2] + 1j * v[1::2]
        if typ == AUDIO:
            return np.frombuffer(raw, np.int16)
        if typ == TEXT:
            return raw.decode(errors="replace")
        return raw

    def close(self):
        try:
            self.sock.close()
        finally:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
            # The app may outlive its ssh session; it must release the DMA buffer.
            if not self.local:
                self.pfb._shell(stop_cmd(self.name))

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
