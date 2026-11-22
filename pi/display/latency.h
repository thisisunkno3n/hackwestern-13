#ifndef SEEKER_LATENCY_H
#define SEEKER_LATENCY_H

/*
 * Camera to HUD latency of the wearer video, per frame drawn, from what is already on the wire:
 * the SKV2 ts_ms. Daniel's phone.html builds the SKV2 frame itself and stamps ts_ms with the
 * phone's Date.now() (epoch ms) just before ws.send; ws_ingest.py passes such frames through
 * untouched (it restamps Pi monotonic ms only on raw JPEG payloads). The two clocks are never
 * compared directly (IF-1): the phone's clock is mapped onto the Pi's by the smallest
 * (arrival - ts_ms) over the last LAT_WINDOW_S seconds, the frame that crossed the network
 * fastest, so "net" is the delay above the best case (queueing in the phone's WebSocket, the
 * Wi-Fi and the relay), not the fixed part (capture, JPEG encode, base Wi-Fi hop, monitor). A
 * ts_ms on the Pi's own monotonic clock (a relay stamp) is used as is.
 *
 * Optional, add-only (INTERFACES "reserved for add-only extensions"): SKV2 flags bit
 * SKV2_FLAG_AGE says the 16-bit reserved field holds the phone's capture-to-send ms (camera
 * frame grabbed to ws.send: canvas draw plus JPEG encode); it is added when present.
 *
 *   total = phone age (if sent) + net + pi + base
 *   pi    = drawn on screen (after the post) - last byte received on tcp/47106
 *   base  = SEEKER_LAT_BASE_MS, the fixed part measured once with the stopwatch photo
 *           (docs/LIVE-RUN.md "Latency"), 0 by default
 *
 * Pure (no I/O, clock passed in) so test_core drives it.
 */

#include <stddef.h>
#include <stdint.h>

#define SKV2_FLAG_AGE 0x10
#define LAT_WINDOW_S 30
#define LAT_MAX_SAMPLES 256

typedef struct {
    uint64_t ts_ms;     /* SKV2 ts_ms */
    int age_ms;         /* phone capture-to-send ms, -1 when the frame does not carry it */
    int64_t arrive_ms;  /* Pi monotonic: last byte of the frame received */
    int64_t publish_ms; /* Pi monotonic: decoded and scaled, ready to draw */
    int64_t present_ms; /* Pi monotonic: posted to the screen inside a HUD frame */
    /* Where the Pi's time goes, ms (0 when not measured): JPEG decode and the CPU scale into
     * the canvas-sized buffer (receiver thread), copying the picture into the canvas, drawing
     * everything else (overlay), copying the canvas to the screen buffer, buffer wait + post. */
    float dec_ms, scale_ms, copy_ms, overlay_ms, blit_ms, post_ms;
} lat_sample;

enum { LAT_DEC, LAT_SCALE, LAT_COPY, LAT_OVERLAY, LAT_BLIT, LAT_POST, LAT_N_STAGES };

typedef struct {
    int n;                         /* frames drawn in the last second */
    float fps;                     /* the same, per second */
    float total_p50, total_p95;    /* ms */
    float net_p50, net_p95;        /* ms above the window's best frame (same-clock: absolute) */
    float pi_p50, pi_p95;          /* receive to on screen */
    float decode_p50, wait_p50;    /* receive to decoded; decoded to on screen */
    float age_p50;                 /* phone capture-to-send, -1 when not sent */
    int same_clock;                /* ts_ms is Pi monotonic (relay stamp) */
    int skipped;                   /* frames received but never drawn (a newer one won) */
    float stage_p50[LAT_N_STAGES]; /* lat_sample stages, p50 ms */
    float rx_fps;                  /* set by the caller: frames reaching the Pi per second */
} lat_summary;

typedef struct {
    float base_ms;
    int have_clock, same_clock;
    int64_t bucket_min[LAT_WINDOW_S]; /* min (arrive - ts) per second, ring */
    int64_t bucket_sec[LAT_WINDOW_S];
    int64_t sec_start;
    int n;
    float total[LAT_MAX_SAMPLES], net[LAT_MAX_SAMPLES], pi[LAT_MAX_SAMPLES];
    float decode[LAT_MAX_SAMPLES], wait[LAT_MAX_SAMPLES], age[LAT_MAX_SAMPLES];
    float stage[LAT_N_STAGES][LAT_MAX_SAMPLES];
    int n_age;
    uint32_t last_seq;
    int have_seq, skipped;
    lat_summary last; /* the previous full second */
    int have_last;
} lat_meter;

void lat_init(lat_meter *m, float base_ms);
/* A wearer frame drawn for the first time (seq is its SKV2 seq: a gap counts as skipped). */
void lat_frame(lat_meter *m, uint32_t seq, const lat_sample *s);
/* Call every HUD frame. Returns 1 (and fills out) once per second when a second has ended. */
int lat_tick(lat_meter *m, int64_t now_ms, lat_summary *out);
/* "LAT 120MS P95 180MS FPS 28" for the HUD, "LAT --" with no frame in the last second. */
void lat_text(const lat_meter *m, char *out, size_t n);
/* The per-second log line. */
void lat_log_line(const lat_summary *s, float base_ms, char *out, size_t n);

#endif
