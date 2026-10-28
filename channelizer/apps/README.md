# Channel apps: process the selected channels on the Pluto's ARM

The FPGA picks the channels; an app on the Pluto's ARM reads them straight from
the DMA (no USB limit there, ~245 MB/s) and sends only its results to the host.
Copy-and-run: nothing to flash for a new app.

```python
from pluto_channelizer import PlutoChannelizer          # host/

with PlutoChannelizer(nch=128, spacing=200e3, lo=98.7e6, channels=None) as pfb:
    pfb.select([pfb.channel_for(f) for f in (91.5e6, 99.5e6, 104.5e6)])
    with pfb.run_app("apps/build/fmaudio") as app:       # or "apps/py/fm_audio.py"
        for m in app.messages():                         # (type, channel, rx, sample, data)
            ...
```

`run_app` copies the app to `/tmp` on the Pluto, starts it with the selection's
slot count and rate, and connects to it (TCP 30500). `local=True` runs a Python
app on your PC against the Pluto instead — develop there, then drop `local`.
To iterate on the board itself: `ssh root@192.168.2.1`, edit `/tmp/pfb_<app>.py`,
call `run_app` again with the same file, or start it by hand.

## Included

| app | does | measured |
|---|---|---|
| `src/requant.c` | `--bits 8\|4\|1 --rx 1\|2\|3`: IQ at a lower bit depth, per-block scaling (NEON) | BLE 2 x 2 MS/s: 32 MB/s DMA -> 8 MB/s USB at 8 bits, 100% real time |
| `src/fmaudio.c` | wideband FM audio for every selected channel | 12 stations: 19.2 MB/s -> 1.2 MB/s, 96% of real time on one core |
| `src/bleadv.c` | BLE GFSK demod, access-address search, de-whitening, CRC on every slot | 2 channels at 2 MS/s: 14.8 packets/s over 0.8 kB/s |
| `src/null.c` | throughput probe | DMA 246 MB/s, copy 210 MB/s, one float op/sample 78 MB/s |
| `py/fm_audio.py`, `py/squelch.py` | the same idea in numpy | real time on a PC; on the board needs the Python firmware |

## Writing one

C: include `src/pfbapp.h`; `pfb_open`, loop over `pfb_next` (frames x slots x
{rx1 i, rx1 q, rx2 i, rx2 q} int16), `pfb_send` typed messages, `pfb_close`.
`make` cross-compiles everything in `src/` with the firmware build's toolchain.
Python: `from pfbapp import App` (`py/pfbapp.py`); same messages.

Budget: one Cortex-A9 at 667 MHz (two with the Python firmware). Scalar float
code costs ~17 cycles per int16 value, so anything per-sample at tens of MB/s
wants NEON or integer arithmetic; copy the DMA block once (it is uncached).

## Notes

- 8 bits keeps BLE lossless; 4 bits lost ~25% of packets with RMS scaling
  (strong packets clip and I/Q clipping bends the phase); 1 and 2 bits decode
  nothing, even oversampled 4x with an IF offset: at 15-30 dB SNR there is no
  noise to dither the quantiser. For FSK, sending phase only (e.g. 4 bits of
  phase) is the better reduction — not implemented yet.
- iiod owns TCP 30431; apps default to 30500.
