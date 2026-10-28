/* bleadv: BLE 1M advertising-packet receiver on every selected channel.
 *
 *   --chan 37,38   BLE channel index of each slot, in slot order (whitening
 *                  seed); default 37 for all
 *   --aa HEX       access address to search for (default 8E89BED6, advertising)
 *   --rx 1|2       receiver (default 1)
 *
 * The slot rate must be a whole number of samples per bit (2 MS/s -> 2).
 * Per slot and per sample phase: sign of the phase step (no atan needed for
 * GFSK), a 32-bit shift register matched against the access address, then
 * the PDU header gives the length; de-whiten, check CRC-24 (init 0x555555)
 * and send the PDU. Payload: u8 channel, u8 pdu length, PDU bytes.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pfbapp.h"

#define MAXBITS (16 + 8 * 255 + 24)

typedef struct {
    uint32_t sr;
    int collecting, n, need;
    uint64_t start;
    uint8_t bits[MAXBITS];
} phase_state;

typedef struct {
    float pi, pq;
    int chan, phase;
    phase_state ph[8];
} slot_state;

static void whiten(uint8_t *b, int n, int chan)
{
    int reg[7] = {1, (chan >> 5) & 1, (chan >> 4) & 1, (chan >> 3) & 1, (chan >> 2) & 1, (chan >> 1) & 1, chan & 1};
    for (int i = 0; i < n; i++) {
        int o = reg[6];
        b[i] ^= (uint8_t)o;
        for (int k = 6; k > 0; k--) reg[k] = reg[k - 1];
        reg[0] = o;
        reg[4] ^= o;
    }
}

static int crc_ok(const uint8_t *b, int pdu_bits)
{
    uint32_t reg = 0x555555;
    for (int i = 0; i < pdu_bits; i++) {
        uint32_t fb = ((reg >> 23) & 1) ^ b[i];
        reg = (reg << 1) & 0xFFFFFF;
        if (fb) reg ^= 0x00065B;
    }
    for (int i = 0; i < 24; i++)
        if (((reg >> (23 - i)) & 1) != b[pdu_bits + i]) return 0;
    return 1;
}

static int to_byte(const uint8_t *b)
{
    int v = 0;
    for (int j = 0; j < 8; j++) v |= b[j] << j;
    return v;
}

int main(int argc, char **argv)
{
    pfb_app app;
    if (pfb_open(&app, argc, argv)) return 1;
    int K = app.slots, sps = (int)(app.rate / 1e6 + 0.5);
    int rx = (int)pfb_arg(&app, "--rx", 1) - 1;
    uint32_t aa = 0x8E89BED6;
    slot_state *st = calloc(K, sizeof *st);
    for (int s = 0; s < K; s++) st[s].chan = 37;
    for (int i = 0; i + 1 < app.argc; i++) {
        if (!strcmp(app.argv[i], "--aa")) aa = (uint32_t)strtoul(app.argv[i + 1], NULL, 16);
        if (!strcmp(app.argv[i], "--chan")) {
            char *p = app.argv[i + 1];
            for (int s = 0; s < K && *p; s++) {
                st[s].chan = (int)strtol(p, &p, 10);
                if (*p == ',') p++;
            }
        }
    }
    if (sps < 1 || sps > 8 || (int)(sps * 1e6) != (int)app.rate) {
        fprintf(stderr, "bleadv: rate %.0f is not a whole number of samples per bit\n", app.rate);
        return 1;
    }
    char meta[256];
    snprintf(meta, sizeof meta, "{\"app\":\"bleadv\",\"aa\":\"%08X\",\"sps\":%d,\"slots\":%d}", aa, sps, K);
    pfb_meta(&app, meta);

    uint8_t pkt[2 + 255];
    const int16_t *b;
    size_t nf;
    uint64_t first;
    while ((b = pfb_next(&app, &nf, &first))) {
        for (int s = 0; s < K; s++) {
            slot_state *d = &st[s];
            const int16_t *p = b + s * 4 + rx * 2;
            for (size_t f = 0; f < nf; f++) {
                float i = p[f * K * 4], q = p[f * K * 4 + 1];
                uint8_t bit = (q * d->pi - i * d->pq) > 0;
                d->pi = i;
                d->pq = q;
                phase_state *h = &d->ph[d->phase];
                if (++d->phase == sps) d->phase = 0;
                if (!h->collecting) {
                    h->sr = (h->sr >> 1) | ((uint32_t)bit << 31);
                    if (h->sr == aa) {
                        h->collecting = 1;
                        h->n = 0;
                        h->need = 16;
                        h->start = first + f;
                    }
                    continue;
                }
                h->bits[h->n++] = bit;
                if (h->n == 16) {                      /* header: length known */
                    uint8_t hdr[16];
                    memcpy(hdr, h->bits, 16);
                    whiten(hdr, 16, d->chan);
                    h->need = 16 + 8 * to_byte(hdr + 8) + 24;
                }
                if (h->n < h->need) continue;
                h->collecting = 0;
                h->sr = 0;
                whiten(h->bits, h->n, d->chan);
                int pdu_bits = h->n - 24;
                if (!crc_ok(h->bits, pdu_bits)) continue;
                pkt[0] = (uint8_t)d->chan;
                pkt[1] = (uint8_t)(pdu_bits / 8);
                for (int k = 0; k < pdu_bits / 8; k++) pkt[2 + k] = (uint8_t)to_byte(h->bits + 8 * k);
                if (pfb_send(&app, PFB_PACKET, rx, s, h->start, pkt, 2 + pdu_bits / 8)) goto done;
            }
        }
    }
done:
    free(st);
    pfb_close(&app);
    return 0;
}
