#ifndef SEEKER_VIDEO_RX_H
#define SEEKER_VIDEO_RX_H

/*
 * IF-4 / IF-5.6 receiver for hud_display: one thread owns TCP 47106 (IPv4 0.0.0.0 plus IPv6 [::]
 * with IPV6_V6ONLY=1, IPv4 only if the stack refuses IPv6), reads SKV2 frames from one relay at a
 * time (a new connection replaces the old one), decodes only the newest complete JPEG, scales it
 * into a canvas-sized buffer and publishes it through a triple buffer. The draw loop calls
 * video_rx_latest, which only ever try-locks and swaps pointers, so it never waits on the
 * network, the decoder or the scaler.
 */

#include <stdint.h>

/* Phone video over Wi-Fi is bursty (gaps of 0.5 to 1.5 s while the phone runs pose and tag
 * detection), so a 500 ms cut-off flapped the HUD between the picture and the radar fallback.
 * After VIDEO_STALE_MS without a frame the last picture stays up, dimmed and tagged VIDEO
 * STALE; only after VIDEO_HOLD_MS is the video LOST (banner, radar fallback). */
#define VIDEO_STALE_MS 500
#define VIDEO_HOLD_MS 2000
/* EagleEye wearer view: after VIDEO_HOLD_MS (VIDEO LOST) the last picture of a still-connected
 * relay stays up, dark, under the VIDEO LOST banner, and only after VIDEO_FREEZE_MS does the
 * full-screen radar take over, so a phone that hiccups for a few seconds never flips the
 * monitor's whole layout. */
#define VIDEO_FREEZE_MS 6000

enum {
    VIDEO_OFF,   /* receiver disabled (--no-video) */
    VIDEO_NONE,  /* no relay connected */
    VIDEO_LOST,  /* connected, but no decodable frame for VIDEO_HOLD_MS */
    VIDEO_STALE, /* last frame VIDEO_STALE_MS to VIDEO_HOLD_MS old: still drawn, dimmed */
    VIDEO_OK
};

/* 1 if the picture is drawn in this state (VIDEO_OK or VIDEO_STALE). */
#define VIDEO_SHOWN(state) ((state) == VIDEO_OK || (state) == VIDEO_STALE)

typedef struct {
    const uint32_t *px; /* 0x00RRGGBB, w x h, NULL until the first frame */
    int w, h, stride;
    int rect[4];        /* where the picture sits inside px (letterboxed), x y w h */
    int src_w, src_h;   /* decoded JPEG size */
    char node_id[9];
    uint32_t seq;
    int64_t t_rx_ms;    /* seeker_now_ms() when published (Pi receive clock) */
    uint32_t conn;      /* video_stats.conn of the relay that sent it */
    int64_t t_arrive_ms; /* seeker_now_ms() when its last byte arrived (before decode) */
    uint64_t ts_ms;      /* SKV2 ts_ms (the sender's clock; latency.h) */
    int age_ms;          /* phone capture-to-send ms (SKV2_FLAG_AGE), -1 when not sent */
    float dec_ms;        /* JPEG decode of this frame (receiver thread) */
    float scale_ms;      /* scaling it into the canvas-sized buffer (CPU, nearest neighbour) */
} video_frame;

typedef struct {
    int connected;         /* a relay is connected right now */
    uint32_t conn;         /* bumped on every accepted relay connection */
    float fps;             /* frames published per second, last full second */
    float rx_fps;          /* complete SKV2 frames received per second, before newest-only
                            * skipping (the phone's real send rate as it reaches the Pi) */
    float decode_ms;       /* decode + scale of the last frame */
    uint64_t frames;       /* published */
    uint64_t dropped;      /* complete frames skipped because a newer one was already there */
    uint64_t decode_errors;
    uint64_t resyncs;
    int ipv6;              /* 1 if the IPv6 listener is up */
    /* The current relay connection (all 0 while none is connected). */
    int64_t conn_ms;             /* seeker_now_ms() when it connected */
    int64_t conn_data_ms;        /* when its first byte arrived */
    uint64_t conn_bytes;         /* bytes received from it */
    uint64_t conn_frames;        /* frames from it that decoded */
    uint64_t conn_decode_errors; /* frames from it whose JPEG did not decode */
    uint64_t conn_other_frames;  /* frames from it skipped: not from video_rx_only_node */
    int last_decode_failed;      /* the newest complete frame from it did not decode */
    uint64_t other_frames;       /* all frames skipped as another node's */
} video_stats;

/* Why the picture is (not) on the HUD, for the banner and the logs. */
enum {
    VIDEO_WHY_OK,
    VIDEO_WHY_OFF,       /* --no-video */
    VIDEO_WHY_NO_LINK,   /* nothing connected to the video port (bridge down, or it has not sent) */
    VIDEO_WHY_NO_FRAMES, /* bridge connected, no frame yet: the phone is not sending video */
    VIDEO_WHY_BAD_DATA,  /* bytes arrive but no SKV2 frame completes within a second */
    VIDEO_WHY_DECODE,    /* frames arrive but the newest JPEG did not decode */
    VIDEO_WHY_STALLED,   /* frames came from this bridge, then stopped for VIDEO_HOLD_MS */
    VIDEO_WHY_OTHER_NODE, /* frames come, but only from nodes other than video_rx_only_node */
};

#define VIDEO_BAD_DATA_MS 1000 /* bytes with no complete frame for this long: BAD_DATA */

typedef struct video_rx video_rx;

/* Binds port (0 picks a free one; see video_rx_port) and starts the thread. out_w x out_h is the
 * canvas size frames are scaled to. prio > 0 runs the thread SCHED_RR at that priority (falls
 * back to the inherited policy if refused). Returns NULL if the IPv4 listener cannot bind. */
video_rx *video_rx_start(int port, int out_w, int out_h, int prio);
int video_rx_port(const video_rx *rx);
/* Only frames whose SKV2 node_id is node are decoded and shown; others are counted, logged once
 * per node and skipped before decoding, so they never replace or refresh the picture. NULL or ""
 * takes every node (the default). The wearer view pins its wearer node. */
void video_rx_only_node(video_rx *rx, const char *node);
/* Never blocks. Swaps in the newest published frame when the lock is free and returns the
 * current one (px NULL if none yet; the pointer stays valid until the next call). Fills st
 * if not NULL (the last snapshot if the lock was busy). */
const video_frame *video_rx_latest(video_rx *rx, video_stats *st);
/* Waits up to timeout_ms for a frame newer than the one video_rx_latest last handed out, so
 * the draw loop can draw it the moment it is decoded instead of on its next tick. Returns 1 if
 * one is ready, 0 on timeout. */
int video_rx_wait(video_rx *rx, int timeout_ms);
void video_rx_stop(video_rx *rx);

/* VIDEO_* for one snapshot. st NULL means the receiver is off; now is seeker_now_ms(). A frame
 * from an earlier connection is never shown, however recent: a new relay is VIDEO_LOST until
 * its own first frame decodes. */
int video_rx_state(const video_frame *frame, const video_stats *st, int64_t now);
/* 1 when frame is this relay's last picture and at most max_ms old (relay still connected). */
int video_rx_frozen(const video_frame *frame, const video_stats *st, int64_t now, int max_ms);
/* "off", "none", "lost", "stale" or "ok" (logs and the PPM snapshot note). */
const char *video_state_name(int state);
/* VIDEO_WHY_* for one snapshot (same arguments as video_rx_state). */
int video_rx_why(const video_frame *frame, const video_stats *st, int64_t now);
/* HUD banner text ("" for VIDEO_WHY_OK), e.g. "NO VIDEO: PHONE NOT SENDING". */
const char *video_why_text(int why);
/* Short name for logs: "ok", "off", "no-link", "no-frames", "bad-data", "decode", "stalled",
 * "other-node". */
const char *video_why_name(int why);

#endif
