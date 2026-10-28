#include "pfbapp.h"

#include <errno.h>
#include <iio.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }

static int listen_once(int port)
{
    int s = socket(AF_INET, SOCK_STREAM, 0), one = 1;
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(port),
                            .sin_addr.s_addr = htonl(INADDR_ANY)};
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    if (bind(s, (struct sockaddr *)&a, sizeof a) || listen(s, 1)) {
        perror("pfbapp: listen");
        return -1;
    }
    fprintf(stderr, "pfbapp: listening on %d\n", port);
    int c = accept(s, NULL, NULL);
    close(s);
    if (c >= 0) {
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        int sz = 1 << 20;
        setsockopt(c, SOL_SOCKET, SO_SNDBUF, &sz, sizeof sz);
    }
    return c;
}

double pfb_arg(const pfb_app *app, const char *name, double dflt)
{
    for (int i = 0; i + 1 < app->argc; i++)
        if (!strcmp(app->argv[i], name))
            return atof(app->argv[i + 1]);
    return dflt;
}

int pfb_open(pfb_app *app, int argc, char **argv)
{
    memset(app, 0, sizeof *app);
    app->port = 30500;   /* iiod has 30431 */
    app->argv = calloc(argc + 1, sizeof *argv);
    for (int i = 1; i < argc; i++) {
        if (i + 1 < argc && !strcmp(argv[i], "--port")) app->port = atoi(argv[++i]);
        else if (i + 1 < argc && !strcmp(argv[i], "--slots")) app->slots = atoi(argv[++i]);
        else if (i + 1 < argc && !strcmp(argv[i], "--rate")) app->rate = atof(argv[++i]);
        else if (i + 1 < argc && !strcmp(argv[i], "--frames")) app->frames = atoi(argv[++i]);
        else app->argv[app->argc++] = argv[i];
    }
    if (app->slots < 1 || app->rate <= 0) {
        fprintf(stderr, "pfbapp: need --slots and --rate\n");
        return -1;
    }
    if (app->frames <= 0) {               /* ~10 ms per block, at least 256 frames */
        app->frames = (int)(app->rate / 100);
        if (app->frames < 256) app->frames = 256;
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    struct iio_context *ctx = iio_create_local_context();
    struct iio_device *dev = ctx ? iio_context_find_device(ctx, "cf-ad9361-lpc") : NULL;
    if (!dev) {
        fprintf(stderr, "pfbapp: no cf-ad9361-lpc\n");
        return -1;
    }
    for (int i = 0; i < 4; i++) {
        char name[16];
        snprintf(name, sizeof name, "voltage%d", i);
        iio_channel_enable(iio_device_find_channel(dev, name, false));
    }
    app->ctx = ctx;
    app->dev = dev;
    /* Connect first: the DMA starts with the buffer, and blocks queued while
     * nobody reads would only overflow. */
    app->fd = listen_once(app->port);
    if (app->fd < 0) return -1;
    app->buf = iio_device_create_buffer(dev, (size_t)app->frames * app->slots, false);
    if (!app->buf) {
        fprintf(stderr, "pfbapp: buffer: %s (is the host still holding one?)\n", strerror(errno));
        return -1;
    }
    return 0;
}

const int16_t *pfb_next_raw(pfb_app *app, size_t *nframes, uint64_t *first)
{
    if (stop) return NULL;
    ssize_t n = iio_buffer_refill(app->buf);
    if (n < 0 || stop) return NULL;
    *nframes = (size_t)n / 8 / app->slots;
    *first = app->frame_index;
    app->frame_index += *nframes;
    return iio_buffer_start(app->buf);
}

const int16_t *pfb_next(pfb_app *app, size_t *nframes, uint64_t *first)
{
    if (stop) return NULL;
    ssize_t n = iio_buffer_refill(app->buf);
    if (n < 0 || stop) return NULL;
    *nframes = (size_t)n / 8 / app->slots;
    *first = app->frame_index;
    app->frame_index += *nframes;
    /* The DMA buffer is uncached; one sequential copy is far cheaper than
     * an app striding through it once per slot. */
    if (!app->copy) app->copy = malloc((size_t)n);
    memcpy(app->copy, iio_buffer_start(app->buf), (size_t)n);
    return app->copy;
}

static int write_all(int fd, const void *p, size_t n)
{
    const char *c = p;
    while (n) {
        ssize_t w = write(fd, c, n);
        if (w < 0 && errno == EINTR && !stop) continue;
        if (w <= 0) return -1;
        c += w;
        n -= (size_t)w;
    }
    return 0;
}

int pfb_send(pfb_app *app, int type, int rx, int slot, uint64_t sample,
             const void *payload, uint32_t len)
{
    pfb_msg h = {.magic = 0x4250, .type = (uint8_t)type, .rx = (uint8_t)rx,
                 .slot = (uint16_t)slot, .seq = app->seq++, .len = len, .sample = sample};
    if (write_all(app->fd, &h, sizeof h) || write_all(app->fd, payload, len)) {
        stop = 1;
        return -1;
    }
    return 0;
}

int pfb_meta(pfb_app *app, const char *json)
{
    return pfb_send(app, PFB_META, 0, 0xFFFF, 0, json, (uint32_t)strlen(json));
}

void pfb_close(pfb_app *app)
{
    if (app->buf) iio_buffer_destroy(app->buf);
    if (app->ctx) iio_context_destroy(app->ctx);
    if (app->fd > 0) close(app->fd);
    free(app->copy);
    free(app->argv);
}
