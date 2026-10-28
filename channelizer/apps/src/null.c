/* null: measure what the ARM can sustain. --mode 0: refill only; 1: refill and
 * copy (pfb_next); 2: copy and one float pass over every sample. Reports
 * frames per second as text once a second. */
#include <stdio.h>
#include <time.h>

#include "pfbapp.h"

int main(int argc, char **argv)
{
    pfb_app app;
    if (pfb_open(&app, argc, argv)) return 1;
    int mode = (int)pfb_arg(&app, "--mode", 1);
    pfb_meta(&app, "{\"app\":\"null\"}");
    struct timespec t0, t;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    size_t nf, total = 0;
    uint64_t first;
    const int16_t *b;
    volatile float sink = 0;
    while ((b = mode ? pfb_next(&app, &nf, &first) : pfb_next_raw(&app, &nf, &first))) {
        if (mode == 2) {
            float acc = 0;
            for (size_t i = 0; i < nf * app.slots * 4; i++) acc += b[i] * (float)b[i];
            sink = acc;
        }
        total += nf;
        clock_gettime(CLOCK_MONOTONIC, &t);
        double el = (t.tv_sec - t0.tv_sec) + 1e-9 * (t.tv_nsec - t0.tv_nsec);
        if (el >= 1) {
            char line[96];
            int n = snprintf(line, sizeof line, "%.0f frames/s = %.1f MB/s from DMA", total / el, total / el * app.slots * 8 / 1e6);
            if (pfb_send(&app, PFB_TEXT, 0, 0xFFFF, first, line, (uint32_t)n)) break;
            total = 0;
            t0 = t;
        }
    }
    (void)sink;
    pfb_close(&app);
    return 0;
}
