#include "video_rx.h"
#include "latency.h"
#include "seeker_log.h"
#include "seeker_time.h"
#include "seeker_udp.h"
#include "video_decode.h"
#include "video_proto.h"

#include <errno.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define CHUNK 65536
#define DRAIN_MAX_CHUNKS 64 /* up to 4 MB per pass, then decode, so a flood cannot starve it */

struct video_rx {
    int lfd[2], nl, port, ipv6;
    int cfd;
    uint32_t conn; /* receiver thread; copied to stats.conn under mu */
    char peer[SEEKER_UDP_ADDRSTRLEN];
    pthread_t th;
    atomic_int stop;
    skv2_parser parser;
    int out_w, out_h;
    uint32_t *pix[3];
    video_frame slot[3];
    int back, ready, front, fresh;
    pthread_mutex_t mu;
    pthread_cond_t cv; /* signalled (under mu) when fresh is set */
    video_stats stats; /* guarded by mu */
    int64_t arrive_ms; /* receiver thread: when the newest complete frame's last byte came */
    video_stats cache; /* draw thread only */
    /* receiver thread only */
    int first_frame;
    int64_t fps_t, log_t;
    uint64_t rx_frames_t; /* parser.frames at fps_t */
    int fps_n;
    uint64_t decode_errors, seen_bad_v, seen_bad_len, seen_resyncs;
    float decode_ms;
    int warned_size;
    int64_t warn_t, last_frame_ms; /* last idle / no-frames warning; newest decoded frame */
    int warned, stalled;
    uint32_t last_seq;
    char only_node[9];   /* guarded by mu; "" takes every node */
    char other_logged[4][9]; /* other nodes already logged as skipped */
    int gaps_stale, gaps_lost; /* frame gaps over VIDEO_STALE_MS / VIDEO_HOLD_MS since the last
                                * stats line */
    int64_t gap_max;
};

static void set_nonblock(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl >= 0)
        fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

static int listen_v4(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons((unsigned short)port);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0 || listen(fd, 4) < 0) {
        close(fd);
        return -1;
    }
    set_nonblock(fd);
    return fd;
}

static int listen_v6(int port)
{
    int fd = socket(AF_INET6, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    /* Same reason as seeker_udp.c: io-sock defaults to 1, Linux to 0, and with 0 Linux refuses
     * the bind because the IPv4 socket already holds the port. */
    if (setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &one, sizeof one) < 0) {
        close(fd);
        return -1;
    }
    struct sockaddr_in6 a;
    memset(&a, 0, sizeof a);
    a.sin6_family = AF_INET6;
    a.sin6_addr = in6addr_any;
    a.sin6_port = htons((unsigned short)port);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0 || listen(fd, 4) < 0) {
        close(fd);
        return -1;
    }
    set_nonblock(fd);
    return fd;
}

static void set_connected(video_rx *rx, int c)
{
    int64_t now = seeker_now_ms();
    rx->warn_t = now;
    rx->warned = 0;
    rx->stalled = 0;
    rx->last_frame_ms = 0;
    pthread_mutex_lock(&rx->mu);
    rx->stats.connected = c;
    rx->stats.conn = rx->conn;
    rx->stats.conn_ms = c ? now : 0;
    rx->stats.conn_data_ms = 0;
    rx->stats.conn_bytes = rx->stats.conn_frames = rx->stats.conn_decode_errors = 0;
    rx->stats.conn_other_frames = 0;
    rx->stats.last_decode_failed = 0;
    if (!c)
        rx->stats.fps = 0;
    pthread_mutex_unlock(&rx->mu);
}

static void drop_client(video_rx *rx, const char *why)
{
    seeker_log("video: relay %s disconnected (%s)", rx->peer, why);
    close(rx->cfd);
    rx->cfd = -1;
    skv2_parser_reset(&rx->parser);
    set_connected(rx, 0);
}

static void accept_client(video_rx *rx, int lfd)
{
    struct sockaddr_storage a;
    socklen_t al = sizeof a;
    int fd = accept(lfd, (struct sockaddr *)&a, &al);
    if (fd < 0)
        return;
    set_nonblock(fd);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    char peer[SEEKER_UDP_ADDRSTRLEN];
    seeker_udp_ntop(&a, peer, sizeof peer);
    if (rx->cfd >= 0) {
        seeker_log("video: relay %s replaced by %s (newest connection wins)", rx->peer, peer);
        close(rx->cfd);
    } else {
        seeker_log("video: relay connected from %s", peer);
    }
    rx->cfd = fd;
    snprintf(rx->peer, sizeof rx->peer, "%s", peer);
    skv2_parser_reset(&rx->parser);
    rx->first_frame = 1;
    rx->conn++;
    set_connected(rx, 1);
}

static void log_parser_news(video_rx *rx)
{
    skv2_parser *p = &rx->parser;
    if (p->bad_version != rx->seen_bad_v) {
        if (rx->seen_bad_v == 0 || p->bad_version / 100 != rx->seen_bad_v / 100)
            seeker_log("video: WARNING unknown SKV2 v %d (this build reads v %d), resyncing "
                       "(%llu such headers so far)",
                       p->last_bad_v, SKV2_VERSION, (unsigned long long)p->bad_version);
        rx->seen_bad_v = p->bad_version;
    }
    if (p->bad_len != rx->seen_bad_len) {
        if (rx->seen_bad_len == 0 || p->bad_len / 100 != rx->seen_bad_len / 100)
            seeker_log("video: payload_len outside 1..%u, resyncing (%llu so far)",
                       SKV2_MAX_PAYLOAD, (unsigned long long)p->bad_len);
        rx->seen_bad_len = p->bad_len;
    }
    if (p->resyncs != rx->seen_resyncs) {
        if (rx->seen_resyncs == 0 || p->resyncs / 100 != rx->seen_resyncs / 100)
            seeker_log("video: lost sync, scanning for SKV2 magic (%llu resyncs so far)",
                       (unsigned long long)p->resyncs);
        rx->seen_resyncs = p->resyncs;
    }
}

static void read_client(video_rx *rx, uint8_t *chunk)
{
    uint64_t got = 0;
    for (int i = 0; i < DRAIN_MAX_CHUNKS && rx->cfd >= 0; i++) {
        ssize_t n = recv(rx->cfd, chunk, CHUNK, 0);
        if (n > 0) {
            got += (uint64_t)n;
            int done = skv2_parser_feed(&rx->parser, chunk, (size_t)n);
            if (done < 0)
                seeker_log("video: out of memory for a payload, frame skipped");
            else if (done > 0)
                rx->arrive_ms = seeker_now_ms();
            continue;
        }
        if (n == 0) {
            drop_client(rx, "closed");
        } else if (errno == EINTR) {
            continue;
        } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
            drop_client(rx, strerror(errno));
        }
        break;
    }
    if (got && rx->cfd >= 0) {
        pthread_mutex_lock(&rx->mu);
        if (!rx->stats.conn_bytes)
            rx->stats.conn_data_ms = seeker_now_ms();
        rx->stats.conn_bytes += got;
        pthread_mutex_unlock(&rx->mu);
    }
    log_parser_news(rx);
}

static void publish(video_rx *rx)
{
    skv2_header h;
    const uint8_t *jpg;
    size_t len;
    if (!skv2_parser_take(&rx->parser, &h, &jpg, &len))
        return;
    pthread_mutex_lock(&rx->mu);
    int other = rx->only_node[0] && strncmp(h.node_id, rx->only_node, sizeof rx->only_node) != 0;
    if (other) {
        rx->stats.other_frames++;
        rx->stats.conn_other_frames++;
    }
    pthread_mutex_unlock(&rx->mu);
    if (other) {
        int known = 0, free_slot = -1;
        for (int i = 0; i < 4; i++) {
            if (!strcmp(rx->other_logged[i], h.node_id))
                known = 1;
            else if (!rx->other_logged[i][0] && free_slot < 0)
                free_slot = i;
        }
        if (!known) {
            seeker_log("video: frames from %s skipped: this HUD shows only %s's video (the "
                       "wearer); check which phone the bridge forwards (--view-node)",
                       h.node_id, rx->only_node);
            if (free_slot >= 0)
                snprintf(rx->other_logged[free_slot], sizeof rx->other_logged[0], "%s",
                         h.node_id);
        }
        return;
    }
    int64_t t0 = seeker_now_ms(), u0 = seeker_now_us();
    int w = 0, hh = 0;
    uint8_t *rgb = video_jpeg_decode(jpg, len, &w, &hh);
    int64_t u1 = seeker_now_us();
    if (!rgb) {
        rx->decode_errors++;
        if (rx->decode_errors == 1 || rx->decode_errors % 30 == 0)
            seeker_log("video: JPEG decode failed (%s seq %u, %zu bytes; %llu failures so far)",
                       h.node_id, h.seq, len, (unsigned long long)rx->decode_errors);
        pthread_mutex_lock(&rx->mu);
        rx->stats.decode_errors = rx->decode_errors;
        rx->stats.conn_decode_errors++;
        rx->stats.last_decode_failed = 1;
        pthread_mutex_unlock(&rx->mu);
        return;
    }
    video_frame *f = &rx->slot[rx->back];
    video_fit_rect(w, hh, rx->out_w, rx->out_h, f->rect);
    video_blit_rgb(rgb, w, hh, rx->pix[rx->back], rx->out_w, rx->out_h, rx->out_w, f->rect);
    video_jpeg_free(rgb);
    f->dec_ms = (float)(u1 - u0) / 1000.0f;
    f->scale_ms = (float)(seeker_now_us() - u1) / 1000.0f;
    f->px = rx->pix[rx->back];
    f->w = rx->out_w;
    f->h = rx->out_h;
    f->stride = rx->out_w;
    f->src_w = w;
    f->src_h = hh;
    snprintf(f->node_id, sizeof f->node_id, "%s", h.node_id);
    f->seq = h.seq;
    f->conn = rx->conn;
    f->ts_ms = h.ts_ms;
    f->age_ms = h.flags & SKV2_FLAG_AGE ? (int)h.reserved : -1;
    f->t_arrive_ms = rx->arrive_ms ? rx->arrive_ms : t0;
    int64_t now = seeker_now_ms();
    f->t_rx_ms = now;
    rx->decode_ms = (float)(now - t0);
    if (!rx->warned_size && (w != h.width || hh != h.height)) {
        seeker_log("video: header says %ux%u but the JPEG is %dx%d; drawing the JPEG size",
                   h.width, h.height, w, hh);
        rx->warned_size = 1;
    }
    if (rx->stalled)
        seeker_log("video: frames from %s resumed after %lld ms (seq %u)", rx->peer,
                   (long long)(now - rx->last_frame_ms), h.seq);
    if (rx->last_frame_ms > 0) {
        int64_t gap = now - rx->last_frame_ms;
        rx->gaps_stale += gap > VIDEO_STALE_MS;
        rx->gaps_lost += gap > VIDEO_HOLD_MS;
        if (gap > rx->gap_max)
            rx->gap_max = gap;
    }
    rx->stalled = 0;
    rx->last_frame_ms = now;
    rx->last_seq = h.seq;
    if (rx->first_frame) {
        seeker_log("video: first frame from %s: %s %dx%d seq %u, %zu bytes, decode+scale %.0f ms",
                   rx->peer, h.node_id, w, hh, h.seq, len, rx->decode_ms);
        rx->first_frame = 0;
    }
    rx->fps_n++;

    pthread_mutex_lock(&rx->mu);
    int t = rx->back;
    rx->back = rx->ready;
    rx->ready = t;
    rx->fresh = 1;
    pthread_cond_signal(&rx->cv);
    rx->stats.frames++;
    rx->stats.conn_frames++;
    rx->stats.last_decode_failed = 0;
    rx->stats.dropped = rx->parser.dropped;
    rx->stats.decode_ms = rx->decode_ms;
    pthread_mutex_unlock(&rx->mu);
}

/* Says why nothing is arriving: first after 10 s with no bridge or 5 s with a bridge but no
 * frame, then every 30 s; and once when frames stop after they had been flowing. */
static void watch(video_rx *rx)
{
    int64_t now = seeker_now_ms();
    int64_t wait = rx->warned ? 30000 : rx->cfd < 0 ? 10000 : 5000;
    if (rx->cfd < 0) {
        if (now - rx->warn_t >= wait) {
            seeker_log("video: nothing connected to tcp/%d yet. Is the bridge running? Daniel's "
                       "ws_ingest.py connects only after the phone's first video frame (helmet "
                       "page needs &video=1); scripts/seeker_bridge.py connects at once",
                       rx->port);
            rx->warn_t = now;
            rx->warned = 1;
        }
    } else if (!rx->last_frame_ms) {
        if (now - rx->warn_t >= wait) {
            pthread_mutex_lock(&rx->mu);
            uint64_t bytes = rx->stats.conn_bytes, bad = rx->stats.conn_decode_errors;
            pthread_mutex_unlock(&rx->mu);
            if (bad)
                seeker_log("video: bridge %s sends frames but none decodes (%llu JPEG errors)",
                           rx->peer, (unsigned long long)bad);
            else if (bytes)
                seeker_log("video: bridge %s sent %llu bytes but no complete SKV2 frame",
                           rx->peer, (unsigned long long)bytes);
            else
                seeker_log("video: bridge %s connected, no frames yet: the phone is not sending "
                           "video (helmet page needs &video=1)", rx->peer);
            rx->warn_t = now;
            rx->warned = 1;
        }
    } else if (!rx->stalled && now - rx->last_frame_ms > VIDEO_HOLD_MS) {
        seeker_log("video: no frame from %s for %lld ms (last seq %u), bridge still connected",
                   rx->peer, (long long)(now - rx->last_frame_ms), rx->last_seq);
        rx->stalled = 1;
    }
}

static void tick_stats(video_rx *rx)
{
    int64_t now = seeker_now_ms();
    if (now - rx->fps_t < 1000)
        return;
    float fps = (float)rx->fps_n * 1000.0f / (float)(now - rx->fps_t);
    rx->fps_n = 0;
    float rx_fps = (float)(rx->parser.frames - rx->rx_frames_t) * 1000.0f / (float)(now - rx->fps_t);
    rx->rx_frames_t = rx->parser.frames;
    rx->fps_t = now;
    pthread_mutex_lock(&rx->mu);
    rx->stats.fps = fps;
    rx->stats.rx_fps = rx_fps;
    rx->stats.resyncs = rx->parser.resyncs;
    rx->stats.dropped = rx->parser.dropped;
    video_stats s = rx->stats;
    pthread_mutex_unlock(&rx->mu);
    if (s.connected && now - rx->log_t >= 5000) {
        rx->log_t = now;
        char other[48] = "";
        if (s.other_frames)
            snprintf(other, sizeof other, ", %llu from other nodes skipped",
                     (unsigned long long)s.other_frames);
        seeker_log("video: %.1f fps (%.1f received), decode+scale %.0f ms, %llu frames, %llu dropped, "
                   "%llu decode errors, %llu resyncs%s; gaps over %d ms: %d (stale), over %d "
                   "ms: %d (lost), longest %lld ms",
                   s.fps, s.rx_fps, s.decode_ms, (unsigned long long)s.frames,
                   (unsigned long long)s.dropped, (unsigned long long)s.decode_errors,
                   (unsigned long long)s.resyncs, other, VIDEO_STALE_MS, rx->gaps_stale,
                   VIDEO_HOLD_MS, rx->gaps_lost, (long long)rx->gap_max);
        rx->gaps_stale = rx->gaps_lost = 0;
        rx->gap_max = 0;
    }
}

static void *rx_main(void *arg)
{
    video_rx *rx = arg;
    uint8_t *chunk = malloc(CHUNK);
    if (!chunk)
        return NULL;
    rx->fps_t = rx->log_t = rx->warn_t = seeker_now_ms();
    while (!atomic_load(&rx->stop)) {
        struct pollfd p[3];
        int np = 0;
        for (int i = 0; i < rx->nl; i++) {
            p[np].fd = rx->lfd[i];
            p[np].events = POLLIN;
            p[np++].revents = 0;
        }
        int ci = -1;
        if (rx->cfd >= 0) {
            ci = np;
            p[np].fd = rx->cfd;
            p[np].events = POLLIN;
            p[np++].revents = 0;
        }
        int r = poll(p, (nfds_t)np, 100);
        if (r < 0 && errno != EINTR) {
            seeker_log("video: poll failed: %s", strerror(errno));
            seeker_sleep_ms(100);
        }
        /* Read before accepting: a replaced client's fd number can be reused by accept. */
        if (r > 0 && ci >= 0 && (p[ci].revents & (POLLIN | POLLHUP | POLLERR)))
            read_client(rx, chunk);
        for (int i = 0; r > 0 && i < rx->nl; i++)
            if (p[i].revents & POLLIN)
                accept_client(rx, rx->lfd[i]);
        publish(rx);
        tick_stats(rx);
        watch(rx);
    }
    free(chunk);
    return NULL;
}

video_rx *video_rx_start(int port, int out_w, int out_h, int prio)
{
    video_rx *rx = calloc(1, sizeof *rx);
    if (!rx)
        return NULL;
    rx->cfd = -1;
    rx->back = 0;
    rx->ready = 1;
    rx->front = 2;
    rx->out_w = out_w;
    rx->out_h = out_h;
    skv2_parser_init(&rx->parser);
    for (int i = 0; i < 3; i++) {
        rx->pix[i] = calloc((size_t)out_w * (size_t)out_h, 4);
        if (!rx->pix[i])
            goto fail;
    }
    rx->lfd[0] = listen_v4(port);
    if (rx->lfd[0] < 0) {
        seeker_log("video: cannot listen on tcp/%d: %s", port, strerror(errno));
        goto fail;
    }
    rx->nl = 1;
    struct sockaddr_in got;
    socklen_t gl = sizeof got;
    rx->port = port;
    if (getsockname(rx->lfd[0], (struct sockaddr *)&got, &gl) == 0)
        rx->port = ntohs(got.sin_port);
    int fd6 = listen_v6(rx->port);
    if (fd6 >= 0) {
        rx->lfd[rx->nl++] = fd6;
        rx->ipv6 = 1;
    } else {
        seeker_log("video: tcp/%d: no IPv6 listener (%s), IPv4 only", rx->port, strerror(errno));
    }
    rx->stats.ipv6 = rx->ipv6;
    rx->cache = rx->stats;
    pthread_mutex_init(&rx->mu, NULL);
    pthread_condattr_t ca;
    pthread_condattr_init(&ca);
    pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
    pthread_cond_init(&rx->cv, &ca);
    pthread_condattr_destroy(&ca);

    int rc = -1;
    if (prio > 0) {
        pthread_attr_t at;
        struct sched_param sp;
        memset(&sp, 0, sizeof sp);
        sp.sched_priority = prio;
        pthread_attr_init(&at);
        if (pthread_attr_setinheritsched(&at, PTHREAD_EXPLICIT_SCHED) == 0 &&
            pthread_attr_setschedpolicy(&at, SCHED_RR) == 0 &&
            pthread_attr_setschedparam(&at, &sp) == 0)
            rc = pthread_create(&rx->th, &at, rx_main, rx);
        pthread_attr_destroy(&at);
        if (rc != 0)
            seeker_log("video: receiver priority %d rr refused (%s), inheriting", prio,
                       strerror(rc));
    }
    if (rc != 0)
        rc = pthread_create(&rx->th, NULL, rx_main, rx);
    if (rc != 0) {
        seeker_log("video: cannot start receiver thread: %s", strerror(rc));
        pthread_cond_destroy(&rx->cv);
        pthread_mutex_destroy(&rx->mu);
        goto fail;
    }
    seeker_log("video: listening on tcp/%d (%s), frames scaled to %dx%d, receiver %s %d; "
               "waiting for the bridge",
               rx->port, rx->ipv6 ? "IPv4 + IPv6" : "IPv4 only", out_w, out_h,
               prio > 0 ? "rr" : "inherited priority", prio);
    return rx;

fail:
    for (int i = 0; i < rx->nl; i++)
        close(rx->lfd[i]);
    for (int i = 0; i < 3; i++)
        free(rx->pix[i]);
    free(rx);
    return NULL;
}

int video_rx_port(const video_rx *rx)
{
    return rx ? rx->port : -1;
}

const video_frame *video_rx_latest(video_rx *rx, video_stats *st)
{
    if (pthread_mutex_trylock(&rx->mu) == 0) {
        if (rx->fresh) {
            int t = rx->front;
            rx->front = rx->ready;
            rx->ready = t;
            rx->fresh = 0;
        }
        rx->cache = rx->stats;
        pthread_mutex_unlock(&rx->mu);
    }
    if (st)
        *st = rx->cache;
    return &rx->slot[rx->front];
}

int video_rx_wait(video_rx *rx, int timeout_ms)
{
    if (timeout_ms < 0)
        timeout_ms = 0;
    struct timespec until;
    clock_gettime(CLOCK_MONOTONIC, &until);
    until.tv_sec += timeout_ms / 1000;
    until.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (until.tv_nsec >= 1000000000L) {
        until.tv_sec++;
        until.tv_nsec -= 1000000000L;
    }
    pthread_mutex_lock(&rx->mu);
    while (!rx->fresh)
        if (pthread_cond_timedwait(&rx->cv, &rx->mu, &until) != 0)
            break;
    int fresh = rx->fresh;
    pthread_mutex_unlock(&rx->mu);
    return fresh;
}

int video_rx_state(const video_frame *frame, const video_stats *st, int64_t now)
{
    if (!st)
        return VIDEO_OFF;
    if (!st->connected)
        return VIDEO_NONE;
    if (!frame || !frame->px || frame->conn != st->conn)
        return VIDEO_LOST;
    int64_t age = now - frame->t_rx_ms;
    return age <= VIDEO_STALE_MS ? VIDEO_OK : age <= VIDEO_HOLD_MS ? VIDEO_STALE : VIDEO_LOST;
}

int video_rx_frozen(const video_frame *frame, const video_stats *st, int64_t now, int max_ms)
{
    return st && st->connected && frame && frame->px && frame->conn == st->conn &&
           now - frame->t_rx_ms <= max_ms;
}

void video_rx_only_node(video_rx *rx, const char *node)
{
    pthread_mutex_lock(&rx->mu);
    snprintf(rx->only_node, sizeof rx->only_node, "%s", node ? node : "");
    pthread_mutex_unlock(&rx->mu);
}

int video_rx_why(const video_frame *frame, const video_stats *st, int64_t now)
{
    switch (video_rx_state(frame, st, now)) {
    case VIDEO_OFF: return VIDEO_WHY_OFF;
    case VIDEO_NONE: return VIDEO_WHY_NO_LINK;
    case VIDEO_OK:
    case VIDEO_STALE: return VIDEO_WHY_OK;
    default: break;
    }
    if (st->last_decode_failed)
        return VIDEO_WHY_DECODE;
    if (st->conn_frames > 0)
        return VIDEO_WHY_STALLED;
    if (st->conn_other_frames > 0)
        return VIDEO_WHY_OTHER_NODE;
    if (st->conn_bytes > 0 && now - st->conn_data_ms > VIDEO_BAD_DATA_MS)
        return VIDEO_WHY_BAD_DATA;
    return VIDEO_WHY_NO_FRAMES;
}

const char *video_why_text(int why)
{
    switch (why) {
    case VIDEO_WHY_OK: return "";
    case VIDEO_WHY_OFF: return "VIDEO OFF";
    case VIDEO_WHY_NO_LINK: return "NO VIDEO: BRIDGE NOT CONNECTED";
    case VIDEO_WHY_NO_FRAMES: return "NO VIDEO: PHONE NOT SENDING";
    case VIDEO_WHY_BAD_DATA: return "VIDEO ERROR: NOT SKV2 FRAMES";
    case VIDEO_WHY_DECODE: return "VIDEO ERROR: JPEG DECODE FAILED";
    case VIDEO_WHY_STALLED: return "VIDEO LOST";
    case VIDEO_WHY_OTHER_NODE: return "NO VIDEO: ONLY ANOTHER PHONE'S VIDEO";
    default: return "NO VIDEO";
    }
}

const char *video_why_name(int why)
{
    static const char *const names[] = { "ok", "off", "no-link", "no-frames", "bad-data", "decode",
                                         "stalled", "other-node" };
    return why >= 0 && why <= VIDEO_WHY_OTHER_NODE ? names[why] : "?";
}

const char *video_state_name(int state)
{
    switch (state) {
    case VIDEO_OFF: return "off";
    case VIDEO_NONE: return "none";
    case VIDEO_LOST: return "lost";
    case VIDEO_STALE: return "stale";
    case VIDEO_OK: return "ok";
    default: return "?";
    }
}

void video_rx_stop(video_rx *rx)
{
    if (!rx)
        return;
    atomic_store(&rx->stop, 1);
    pthread_join(rx->th, NULL);
    if (rx->cfd >= 0)
        close(rx->cfd);
    for (int i = 0; i < rx->nl; i++)
        close(rx->lfd[i]);
    for (int i = 0; i < 3; i++)
        free(rx->pix[i]);
    skv2_parser_free(&rx->parser);
    pthread_cond_destroy(&rx->cv);
    pthread_mutex_destroy(&rx->mu);
    free(rx);
}
