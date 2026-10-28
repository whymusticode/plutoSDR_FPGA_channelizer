"""Forward a channel's IQ only while it is active: a few lines to make your own.

    --open DB   open when a block's power is this far above the channel's
                running noise floor (default 10)

Quiet channels cost nothing on the link; each active block goes out as int16
IQ (both receivers) tagged with its sample index, so gaps are explicit.
"""
import numpy as np
from pfbapp import App, IQ16

app = App()
open_db = app.arg("--open", 10.0, float)
app.meta(app="squelch", open_db=open_db, rate=app.rate)
floor = np.full(app.slots, np.inf)
for first, x in app.blocks():
    p = (x[:, :, :2].astype(np.float32) ** 2).sum(2).mean(0)        # RX1 power per slot
    floor = np.minimum(floor * 1.02, p)                             # slow-rising minimum
    for s in np.nonzero(p > floor * 10 ** (open_db / 10))[0]:
        app.send(IQ16, int(s), first, np.ascontiguousarray(x[:, s, :]), rx=2)
app.close()
