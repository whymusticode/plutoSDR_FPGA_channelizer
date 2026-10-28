"""Bluetooth LE 1M PHY receiver for one channelizer slot (2 MS/s, 2 samples/bit).

GFSK -> FM discriminator -> bit slicing at both sample phases -> search for
an access address after the 0x55/0xAA preamble. Advertising packets (AA
0x8E89BED6) are de-whitened with the channel index, CRC-checked (init
0x555555) and parsed; on data channels the AA is connection-specific, so a
packet is reported by its AA and a repeat of the same AA is the evidence.
"""
import numpy as np

ADV_AA = 0x8E89BED6
COMPANIES = {0x004C: "Apple", 0x0006: "Microsoft", 0x00E0: "Google", 0x0075: "Samsung",
             0x0087: "Garmin", 0x0157: "Huami", 0x038F: "Xiaomi", 0x0171: "Amazon",
             0x02E5: "Espressif", 0x0059: "Nordic", 0x000F: "Broadcom", 0x0499: "Ruuvi",
             0x0822: "Adaptive Bluetooth", 0x01DA: "Logitech", 0x00D2: "Dialog", 0x0310: "SGL Italia",
             0x0301: "Sony", 0x0002: "Intel", 0x0003: "IBM", 0x0131: "Cypress", 0x0046: "MediaTek"}


def bits_lsb(value, n):
    return [(value >> i) & 1 for i in range(n)]


def whiten(bits, chan):
    """BLE data whitening, x^7 + x^4 + 1, register seeded with 1 | chan."""
    lfsr = (chan & 0x3F) | 0x40
    out = []
    for b in bits:
        w = lfsr & 1   # position 6 of the spec's register, stored reversed
        out.append(b ^ w)
        lfsr >>= 1
        if w:
            lfsr ^= 0x44
    return out


def _whiten_spec(bits, chan):
    # Reference implementation straight from the spec (Vol 6, Part B, 3.2).
    reg = [1] + [(chan >> (5 - i)) & 1 for i in range(6)]   # positions 0..6
    out = []
    for b in bits:
        o = reg[6]
        out.append(b ^ o)
        reg = [o] + reg[:6]
        reg[4] ^= o
    return out


def crc24(bits, init=0x555555):
    reg = init
    for b in bits:
        fb = ((reg >> 23) & 1) ^ b
        reg = (reg << 1) & 0xFFFFFF
        if fb:
            reg ^= 0x00065B
    # transmitted MSB of the register first
    return [(reg >> (23 - i)) & 1 for i in range(24)]


def to_bytes(bits):
    return bytes(sum(bits[8 * i + j] << j for j in range(8)) for i in range(len(bits) // 8))


def demod(z):
    """Hard bits at both sample phases; z is complex at 2 MS/s."""
    d = np.angle(z[1:] * np.conj(z[:-1]))
    return [(d[p::2] > 0).astype(np.uint8) for p in (0, 1)], d


def find_aa(bits, aa=None):
    """Indices just past the access address. aa=None: any AA after a preamble."""
    s = np.asarray(bits, np.uint8)
    if aa is not None:
        pat = np.array(bits_lsb(aa, 32), np.uint8)
        n = len(s) - 32
        if n <= 0:
            return []
        win = np.lib.stride_tricks.sliding_window_view(s, 32)
        hit = np.nonzero((win == pat).all(1))[0]
        return [int(h) + 32 for h in hit]
    pre = np.array([0, 1] * 4, np.uint8)
    win = np.lib.stride_tricks.sliding_window_view(s, 8)
    idx = np.nonzero((win == pre).all(1) | (win == 1 - pre).all(1))[0]
    return [int(i) + 8 + 32 for i in idx if i + 8 + 32 + 16 <= len(s)]


def parse_adv(payload):
    """AdvA plus name / manufacturer from AD structures."""
    hdr, ln = payload[0], payload[1]
    ptype = hdr & 0x0F
    names = {0: "ADV_IND", 1: "ADV_DIRECT_IND", 2: "ADV_NONCONN_IND", 3: "SCAN_REQ",
             4: "SCAN_RSP", 5: "CONNECT_IND", 6: "ADV_SCAN_IND", 7: "ADV_EXT_IND"}
    info = dict(type=names.get(ptype, hex(ptype)), length=ln)
    body = payload[2:2 + ln]
    if ptype in (0, 2, 4, 6) and len(body) >= 6:
        info["adva"] = ":".join(f"{b:02X}" for b in body[5::-1])
        info["random"] = bool(hdr & 0x40)
        ad = body[6:]
        i = 0
        while i + 1 < len(ad):
            l = ad[i]
            if l == 0 or i + 1 + l > len(ad):
                break
            t, v = ad[i + 1], ad[i + 2:i + 1 + l]
            if t in (0x08, 0x09):
                info["name"] = v.decode("utf-8", "replace")
            elif t == 0xFF and len(v) >= 2:
                cid = v[0] | v[1] << 8
                info["maker"] = COMPANIES.get(cid, f"0x{cid:04X}")
            i += 1 + l
    elif ptype == 5 and len(body) >= 22:
        info["initA"] = ":".join(f"{b:02X}" for b in body[5::-1])
        info["advA"] = ":".join(f"{b:02X}" for b in body[11:5:-1])
        info["conn_aa"] = f"0x{int.from_bytes(body[12:16], 'little'):08X}"
    return info


def adv_packets(z, chan=37):
    """Decoded, CRC-valid advertising packets: list of (sample index, info)."""
    out = []
    phases, _ = demod(z)
    for p, bits in enumerate(phases):
        for end in find_aa(bits, ADV_AA):
            hdr = whiten(list(bits[end:end + 16]), chan)
            ln = to_bytes(hdr)[1]
            if ln > 37:
                continue
            nbits = 16 + 8 * ln + 24
            raw = list(bits[end:end + nbits])
            if len(raw) < nbits:
                continue
            dw = whiten(raw, chan)
            pdu, crc = dw[:16 + 8 * ln], dw[16 + 8 * ln:]
            if crc24(pdu) != crc:
                continue
            out.append(((end - 40) * 2 + p, parse_adv(to_bytes(pdu))))
    out.sort(key=lambda x: x[0])
    dedup = []
    for s, info in out:          # the two sample phases often both decode
        if not dedup or s - dedup[-1][0] > 40:
            dedup.append((s, info))
    return dedup


def data_packets(z, min_len_bits=80):
    """Candidate data-channel packets: (sample index, AA) after a preamble with energy."""
    phases, d = demod(z)
    p = np.abs(z) ** 2
    floor = np.median(p)
    out = []
    for ph, bits in enumerate(phases):
        for end in find_aa(bits):
            s = (end - 40) * 2 + ph
            if s < 0 or s + 2 * min_len_bits > len(p) or p[s:s + 2 * min_len_bits].mean() < 30 * floor:
                continue
            aa = int.from_bytes(to_bytes(list(bits[end - 32:end])), "little")
            if aa == ADV_AA:
                continue
            out.append((s, aa))
    out.sort()
    dedup = []
    for s, aa in out:
        if not dedup or s - dedup[-1][0] > 200:
            dedup.append((s, aa))
    return dedup


assert all(whiten(b, c) == _whiten_spec(b, c) for c in (0, 5, 37, 39)
           for b in [list(np.random.default_rng(c).integers(0, 2, 64))])
