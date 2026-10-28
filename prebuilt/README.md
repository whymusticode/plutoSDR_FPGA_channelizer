# Prebuilt firmware

Built from commit 3faf220 of this branch with `make build/pluto.frm build/pluto.dfu`
(Vivado 2023.2, routed WNS +0.188 ns).

- `pluto.frm`: copy to the Pluto's mass-storage drive, eject, wait for the LED to stop blinking.
- `pluto.dfu`: for DFU mode (`dfu-util -a firmware.dfu -D pluto.dfu`).

Checksums in `SHA256SUMS`. Back to stock: flash ADI's v0.39 `pluto.frm` the same way.
