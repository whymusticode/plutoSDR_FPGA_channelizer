# Pluto dual-RX polyphase channelizer

Custom ADALM-Pluto firmware that splits both coherent receivers into a bank
of critically sampled channels in the FPGA and streams only the channels you
select over USB. The channel count, spacing, usable bandwidth and filter
length are set from the host at runtime; no rebuild needed.

```python
from pluto_channelizer import PlutoChannelizer      # host/

with PlutoChannelizer(nch=6, spacing=400e3, bandwidth=300e3, taps=8, lo=100e6) as pfb:
    pfb.select([0, 2, 5])                    # or pfb.select(freqs=[99.2e6, 100.4e6])
    x = pfb.read(400_000)                    # {channel: (2, n) complex64}, RX1/RX2
```

- **Channels** are `0..nch-1`, lowest frequency first, centred at
  `lo + (k - nch//2) * spacing`; each streams at `spacing` samples/s.
- **FFT size** is `n` = `nch` rounded up to a power of two (2..256). The radio
  runs at `fs = n * spacing`; the `n - nch` spare channels sit at the band
  edges, where the radio's analog filter rolls off anyway.
- **`bandwidth`** is the band around each centre that stays free of aliasing.
  The host designs an equiripple prototype that passes `±bandwidth/2` and stops
  from `spacing - bandwidth/2`, the part of the neighbour that folds onto it
  when critically sampled. `bandwidth` close to `spacing` costs stopband: at 8
  taps, `spacing/2` gives about 78 dB of alias rejection and `0.75 * spacing`
  about 49 dB (`pfb.filter_metrics`).
- **Samples** are int16 at Q18 (8 × ADC-code scale; the 64-way bank's
  processing gain makes the 3 extra bits real), converted to ADC-code units
  (full scale 2048).

## Limits

| | |
|---|---|
| FFT size | 2..256, any power of two |
| taps per phase | 1..8 (a shorter prototype is zero padded) |
| fs (FPGA) | 18.2 / 22.2 / 27.2 / 33.1 / 38.2 / 38.5 / 36.9 / 35.9 MS/s for n = 2..256 (`FS_MAX`, from simulation) |
| fs (radio) | ≥ 2.083 MS/s: below that the AD9363 needs its FIR decimation, which the host class does not set up yet |
| USB | ~16 MB/s over the network backend = ~2 MS/s of selected channel samples in total (8 bytes each) |

## Layout

- `rtl/pfb_channelizer.sv` — the bank: polyphase FIR (8 taps/phase, folded
  over 16 DSPs), in-place radix-2 DIF FFT and output stage, sized for 256
  channels and run at `2**cfg_logn`. Taps live in a writable RAM that boots
  with the 64-channel `pfb_coeffs.mem`; `twiddle.mem` is the 256-point table.
- `hdl/` — the Pluto block-design edit (`bd.tcl`), the dual-RX ingress, the
  channel selector, and `pfb_ctrl.v`, the register block at `0x41210000`
  (map in its header).
- `filter/pfb_filter.py` — prototype design, quantization and headroom
  checks; `host/` — `PlutoChannelizer` and a hardware test.
- `test/pfb_model.py` — bit-exact model of the RTL; `test/sim/check_rtl.py`
  runs every size under iverilog against it (plus throughput),
  `test/sim/check_system.py` the registers, clock crossings and selector.

## Control path

The register block has no kernel driver, so the host writes it with `devmem`
over ssh, one ssh call per reconfiguration or channel selection (about
1–3 s at n = 256, which is 2048 tap writes). Before touching the registers
the host checks that the board runs this image (TX DDS disabled, then the
ID register): probing an absent AXI slave on stock firmware can fault the
board. An IIO driver exposing the registers would remove ssh from the path.

## Build

```sh
python3 test/sim/check_rtl.py && python3 test/sim/check_system.py
```

From the repository root, `make` builds this directory into `build/pluto.frm`
(Vivado 2023.2; see the top-level README). Check the routed timing
(`hdl/projects/pluto/pluto.runs/impl_1/system_top_timing_summary_routed.rpt`,
WNS must be >= 0) before flashing.
