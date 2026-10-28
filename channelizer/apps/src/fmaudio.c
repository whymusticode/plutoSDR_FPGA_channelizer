/* fmaudio: wideband-FM mono demodulation of every selected channel on the ARM.
 *
 *   --rx 1|2      receiver to demodulate (default 1)
 *   --decim D     audio rate = rate / D (default: nearest to 48 kHz)
 *
 * Per slot: polar discriminator (phase step of z[n] conj(z[n-1])), 75 us
 * de-emphasis, then a 15 kHz windowed-sinc low-pass evaluated only at the
 * decimated outputs (it keeps the 19 kHz pilot and 38 kHz stereo subcarrier
 * out of the audio) and decimation by D. Sends int16
 * audio per slot per block; full scale is +-75 kHz deviation.
 *
 * Slots are split between two threads (the Pluto has two cores); the main
 * thread sends every slot's audio once both halves of a block are done.
 */
#include <pthread.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "pfbapp.h"

#define HIST 256              /* power of two >= taps */

typedef struct {
    float pi, pq, de;
    float hist[HIST];
    unsigned w;
    int n;
} demod_state;

typedef struct {
    demod_state *st;
    const int16_t *b;
    size_t nf;
    int K, rx, D, L, s0, s1;
    float a, gain;
    const float *h;
    int16_t **out;
    size_t *m;
} job;

static void demod_slots(job *j)
{
    for (int s = j->s0; s < j->s1; s++) {
        demod_state *d = &j->st[s];
        const int16_t *p = j->b + s * 4 + j->rx * 2;
        size_t m = 0;
        for (size_t f = 0; f < j->nf; f++) {
            float i = p[f * j->K * 4], q = p[f * j->K * 4 + 1];
            /* z conj(prev): re = i pi + q pq, im = q pi - i pq */
            float re = i * d->pi + q * d->pq, im = q * d->pi - i * d->pq;
            d->pi = i;
            d->pq = q;
            float ph = pfb_atan2(im, re);
            d->de = ph + j->a * (d->de - ph);       /* de-emphasis, one pole */
            d->hist[d->w++ & (HIST - 1)] = d->de;
            if (++d->n == j->D) {
                float v = 0;
                for (int k = 0; k < j->L; k++) v += j->h[k] * d->hist[(d->w - 1 - k) & (HIST - 1)];
                v *= j->gain;
                j->out[s][m++] = (int16_t)(v > 32767 ? 32767 : v < -32767 ? -32767 : v);
                d->n = 0;
            }
        }
        j->m[s] = m;
    }
}

static void *worker(void *arg)
{
    demod_slots(arg);
    return NULL;
}

int main(int argc, char **argv)
{
    pfb_app app;
    if (pfb_open(&app, argc, argv)) return 1;
    int rx = (int)pfb_arg(&app, "--rx", 1) - 1;
    int D = (int)pfb_arg(&app, "--decim", 0);
    if (D <= 0) D = (int)lrint(app.rate / 48000.0);
    if (D < 1) D = 1;
    double arate = app.rate / D;
    char meta[256];
    snprintf(meta, sizeof meta, "{\"app\":\"fmaudio\",\"rx\":%d,\"audio_rate\":%.3f,\"slots\":%d}",
             rx + 1, arate, app.slots);
    pfb_meta(&app, meta);

    int K = app.slots;
    demod_state *st = calloc(K, sizeof *st);
    float a = expf(-1.f / (float)(app.rate * 75e-6));
    float gain = (float)(app.rate / (2 * M_PI * 75e3)) * 32767.f;
    int L = 8 * D + 1;
    if (L > HIST) L = HIST - 1;
    float *h = malloc(sizeof(float) * L), hs = 0;
    for (int k = 0; k < L; k++) {
        double t = k - (L - 1) / 2.0, fc = 15e3 / app.rate;
        double v = t == 0 ? 2 * fc : sin(2 * M_PI * fc * t) / (M_PI * t);
        h[k] = (float)(v * (0.54 - 0.46 * cos(2 * M_PI * k / (L - 1))));
        hs += h[k];
    }
    for (int k = 0; k < L; k++) h[k] /= hs;
    int16_t **out = malloc(sizeof(int16_t *) * K);
    size_t *m = calloc(K, sizeof *m);
    for (int s = 0; s < K; s++) out[s] = malloc(sizeof(int16_t) * (app.frames / D + 2));
    const int16_t *b;
    size_t nf;
    uint64_t first;
    while ((b = pfb_next(&app, &nf, &first))) {
        job lo = {st, b, nf, K, rx, D, L, 0, K / 2, a, gain, h, out, m}, hi = lo;
        hi.s0 = K / 2;
        hi.s1 = K;
        pthread_t t;
        int threaded = K > 1 && !pthread_create(&t, NULL, worker, &hi);
        demod_slots(&lo);
        if (threaded) pthread_join(t, NULL);
        else demod_slots(&hi);
        for (int s = 0; s < K; s++)
            if (pfb_send(&app, PFB_AUDIO, rx, s, first / D, out[s], (uint32_t)(m[s] * 2))) goto done;
    }
done:
    for (int s = 0; s < K; s++) free(out[s]);
    free(out);
    free(m);
    free(h);
    free(st);
    pfb_close(&app);
    return 0;
}
