/* requant: lower the bitrate of the selected channels before USB.
 *
 *   --bits 8|4|1   bits per I and per Q (default 8)
 *   --rx 1|2|3     receivers to send: 1 = RX1, 2 = RX2, 3 = both (default 3)
 *
 * Each block, per slot and receiver: a float32 scale then the samples, with
 * the scale set from the block's own RMS so the signal sits at a fixed level
 * (clipping 4 sigma above it at 8 and 4 bits; 1 bit is the sign alone).
 * sample = code * scale reproduces the int16 stream's units.
 *
 * NEON integer throughout: scalar float code on this core manages ~17 cycles
 * a sample, which is a third of what the DMA delivers. The block is first
 * deinterleaved straight out of the (uncached) DMA buffer into one contiguous
 * I,Q array per slot and receiver.
 */
#include <arm_neon.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pfbapp.h"

/* x * g, g = gm / 32768 * 2^sh, rounded and saturated, for a vector of 8. */
static inline int16x8_t scale8(int16x8_t x, int16x8_t gm, int16x8_t sh)
{
    return vqrshlq_s16(vqrdmulhq_s16(x, gm), sh);
}

int main(int argc, char **argv)
{
    pfb_app app;
    if (pfb_open(&app, argc, argv)) return 1;
    int bits = (int)pfb_arg(&app, "--bits", 8), rxmask = (int)pfb_arg(&app, "--rx", 3);
    if (bits != 8 && bits != 4 && bits != 1) bits = 8;
    int type = bits == 8 ? PFB_IQ8 : bits == 4 ? PFB_IQ4 : PFB_IQ1;
    char meta[256];
    snprintf(meta, sizeof meta, "{\"app\":\"requant\",\"bits\":%d,\"rx\":%d,\"rate\":%.1f,\"slots\":%d}",
             bits, rxmask, app.rate, app.slots);
    pfb_meta(&app, meta);

    int K = app.slots;
    size_t maxf = (size_t)app.frames + 16;
    uint32_t *lanes = aligned_alloc(16, sizeof(uint32_t) * maxf * K * 2);   /* [slot*2+rx][frame] I,Q */
    int8_t *q8 = aligned_alloc(16, maxf * 2 + 32);
    uint8_t *out = malloc(4 + maxf * 2 + 32);
    const int16_t *b;
    size_t nf;
    uint64_t first;
    while ((b = pfb_next_raw(&app, &nf, &first))) {
        /* deinterleave: one sequential read of the DMA buffer */
        const uint32_t *in = (const uint32_t *)b;
        for (size_t f = 0; f < nf; f++)
            for (int j = 0; j < K * 2; j++)
                lanes[j * maxf + f] = in[f * K * 2 + j];
        size_t nv = nf * 2;                                /* int16 values per lane */
        size_t nv8 = nv & ~(size_t)7;
        for (int s = 0; s < K; s++)
            for (int r = 0; r < 2; r++) {
                if (!(rxmask & (1 << r))) continue;
                const int16_t *x = (const int16_t *)(lanes + (size_t)(s * 2 + r) * maxf);
                int64x2_t acc = vdupq_n_s64(0);
                for (size_t i = 0; i < nv8; i += 8) {
                    int16x8_t v = vld1q_s16(x + i);
                    acc = vpadalq_s32(acc, vmull_s16(vget_low_s16(v), vget_low_s16(v)));
                    acc = vpadalq_s32(acc, vmull_s16(vget_high_s16(v), vget_high_s16(v)));
                }
                double ms = (double)(vgetq_lane_s64(acc, 0) + vgetq_lane_s64(acc, 1)) / (nv8 ? nv8 : 1);
                float rms = (float)sqrt(ms) + 1e-3f;
                float full = bits == 8 ? 127.f : 7.f;
                float scale = bits == 1 ? rms : rms * 4.f / full;   /* int16 units per code */
                /* 1/scale as gm/32768 * 2^sh with gm in [16384, 32768) */
                int e;
                float m = frexpf(1.f / scale, &e);                  /* 1/scale = m 2^e, m in [0.5,1) */
                int16x8_t gm = vdupq_n_s16((int16_t)lrintf(m * 32767.f));
                int16x8_t sh = vdupq_n_s16((int16_t)e);
                memcpy(out, &scale, 4);
                size_t len = 4;
                if (bits == 8 || bits == 4) {
                    int8x16_t lim = vdupq_n_s8(bits == 8 ? 127 : 7);
                    for (size_t i = 0; i < nv8; i += 16) {
                        int8x8_t a = vqmovn_s16(scale8(vld1q_s16(x + i), gm, sh));
                        int8x8_t c = vqmovn_s16(scale8(vld1q_s16(x + i + 8), gm, sh));
                        int8x16_t v = vcombine_s8(a, c);
                        v = vmaxq_s8(vminq_s8(v, lim), vnegq_s8(lim));
                        vst1q_s8(q8 + i, v);
                    }
                    if (bits == 8) {
                        memcpy(out + 4, q8, nv8);
                        len += nv8;
                    } else {                                        /* I low nibble, Q high */
                        uint8x16_t lo = vdupq_n_u8(0x0F);
                        for (size_t i = 0; i < nv8; i += 32) {
                            int8x16x2_t iq = vld2q_s8(q8 + i);      /* even = I, odd = Q */
                            uint8x16_t v = vorrq_u8(vandq_u8(vreinterpretq_u8_s8(iq.val[0]), lo),
                                                    vshlq_n_u8(vreinterpretq_u8_s8(iq.val[1]), 4));
                            vst1q_u8(out + 4 + i / 2, v);
                        }
                        len += nv8 / 2;
                    }
                } else {                                            /* sign bits, LSB first */
                    static const uint8_t w[8] = {1, 2, 4, 8, 16, 32, 64, 128};
                    uint8x8_t wt = vld1_u8(w);
                    for (size_t i = 0; i < nv8; i += 8) {
                        uint8x8_t ge = vmovn_u16(vcgeq_s16(vld1q_s16(x + i), vdupq_n_s16(0)));
                        uint8x8_t v = vand_u8(ge, wt);
                        v = vpadd_u8(v, v);
                        v = vpadd_u8(v, v);
                        v = vpadd_u8(v, v);
                        out[4 + i / 8] = vget_lane_u8(v, 0);
                    }
                    len += nv8 / 8;
                }
                if (pfb_send(&app, type, r, s, first, out, (uint32_t)len)) goto done;
            }
    }
done:
    free(out);
    free(q8);
    free(lanes);
    pfb_close(&app);
    return 0;
}
