/* On-Pluto channel apps: read the channelizer's DMA stream locally, process
 * the selected channels on the ARM, and send only results to the host.
 *
 * The host (PlutoChannelizer.run_app) configures the bank and the channel
 * selection, copies the app to /tmp, starts it with the common arguments
 * below plus the app's own, and connects to its TCP port. Common arguments:
 *   --port P     TCP port to listen on, one client (default 30500; iiod has 30431)
 *   --slots K    selected channels per frame, in DMA slot order
 *   --rate R     samples per second per channel (the bank spacing)
 *   --frames F   frames per DMA block (default: ~10 ms worth)
 *
 * Stream: a sequence of messages, each a pfb_msg header then `len` bytes.
 * The first message is PFB_META with a JSON object describing the app's
 * output; everything after is the app's own. Integers are little endian.
 */
#ifndef PFBAPP_H
#define PFBAPP_H
#include <stdint.h>
#include <stddef.h>

enum {
    PFB_META = 0,    /* JSON text */
    PFB_IQ16 = 1,    /* int16 I,Q per sample; payload per slot */
    PFB_IQ8 = 2,     /* float32 scale, then int8 I,Q */
    PFB_IQ4 = 3,     /* float32 scale, then 4-bit I,Q packed (I low nibble) */
    PFB_IQ1 = 4,     /* float32 scale, then sign bits of I,Q packed LSB first */
    PFB_AUDIO = 5,   /* int16 samples */
    PFB_PACKET = 6,  /* app-defined bytes */
    PFB_TEXT = 7,    /* UTF-8 line */
};

typedef struct __attribute__((packed)) {
    uint16_t magic;      /* 0x4250, "PB" */
    uint8_t type;
    uint8_t rx;          /* receiver: 0 = RX1, 1 = RX2, 2 = both interleaved */
    uint16_t slot;       /* DMA slot the payload belongs to, 0xFFFF = none */
    uint16_t seq;        /* per-message counter, wraps */
    uint32_t len;        /* payload bytes */
    uint64_t sample;     /* index of the payload's first sample in the slot's stream */
} pfb_msg;

typedef struct {
    int slots, frames, port;
    double rate;
    int argc;            /* the app's own arguments, common ones removed */
    char **argv;
    /* private */
    void *ctx, *dev, *buf;
    int16_t *copy;
    int fd;
    uint16_t seq;
    uint64_t frame_index;
} pfb_app;

/* Parse common arguments, open the DMA stream, wait for the host to connect. */
int pfb_open(pfb_app *app, int argc, char **argv);
/* Next block: returns a pointer to `*nframes` frames of `slots` x 4 int16
 * {rx1 i, rx1 q, rx2 i, rx2 q}; *first is the block's first frame index.
 * NULL when the stream ends (host gone or a signal). */
const int16_t *pfb_next(pfb_app *app, size_t *nframes, uint64_t *first);
/* As pfb_next, but the DMA buffer itself (uncached: read it once, sequentially). */
const int16_t *pfb_next_raw(pfb_app *app, size_t *nframes, uint64_t *first);
/* Send one message; returns 0, or -1 once the host has gone. */
int pfb_send(pfb_app *app, int type, int rx, int slot, uint64_t sample,
             const void *payload, uint32_t len);
int pfb_meta(pfb_app *app, const char *json);

/* atan2 to ~0.004 rad, several times faster than libm's on the Cortex-A9. */
static inline float pfb_atan2(float y, float x)
{
    float ax = x < 0 ? -x : x, ay = y < 0 ? -y : y;
    float mx = ax > ay ? ax : ay, mn = ax > ay ? ay : ax;
    if (mx == 0) return 0;
    float z = mn / mx;
    float r = z * (0.78539816f + 0.273f * (1 - z));
    if (ay > ax) r = 1.57079633f - r;
    if (x < 0) r = 3.14159265f - r;
    return y < 0 ? -r : r;
}
void pfb_close(pfb_app *app);
/* An app argument "--name value" as a double, or `dflt`. */
double pfb_arg(const pfb_app *app, const char *name, double dflt);
#endif
