"""Power of all channels of a bank, a link-sized group at a time, so labels are exact."""
import sys, numpy as np
from capture import PlutoChannelizer, OUT


def survey(name, lo, nch=64, spacing=400e3, group=8, n=40000, gains=(40, 40)):
    with PlutoChannelizer(nch=nch, spacing=spacing, bandwidth=spacing / 2, lo=lo, gains=gains,
                          channels=None, rf_bandwidth=min(nch * spacing, 56e6)) as pfb:
        pw = np.zeros((2, nch))
        for g in range(0, nch, group):
            pfb.select(range(g, g + group))
            x = pfb.read(n)
            for c in x:
                pw[:, c] = (np.abs(x[c]) ** 2).mean(1)
        f = np.array([pfb.channel_freq(c) for c in range(nch)])
    np.savez(OUT / f"survey_{name}.npz", f=f, pw=pw)
    return f, pw


if __name__ == "__main__":
    name, lo = sys.argv[1], float(sys.argv[2])
    f, pw = survey(name, lo)
    db = 10 * np.log10(pw[0] / 2048**2)
    floor = np.median(db)
    print(f"{name} LO {lo/1e6:.1f} MHz, floor {floor:.1f} dBFS")
    line = "".join(" .:-=+*#%@"[min(9, max(0, int((d - floor) / 3)))] for d in db)
    print(f"  {f[0]/1e6:7.1f} |{line}| {f[-1]/1e6:.1f}")
