#ifndef SEEKER_VIDEO_PROTO_H
#define SEEKER_VIDEO_PROTO_H

/*
 * IF-5.6 video_frame framing on TCP 47106 (INTERFACES.md is the contract; VIDEO.md has notes).
 * Every frame is a 36-byte big-endian header followed by payload_len bytes of JPEG:
 *
 *   off size field        value
 *     0    4 magic        "SKV2"
 *     4    1 v            2
 *     5    1 flags        0 (baseline JPEG)
 *     6    2 reserved     0
 *     8    8 node_id      NUL-padded ASCII, e.g. "R1"
 *    16    8 ts_ms        sender's monotonic ms (informational, never compared, IF-1)
 *    24    2 width        px
 *    26    2 height       px
 *    28    4 seq          per node, monotonic
 *    32    4 payload_len  1 .. 2097152
 *    36    n jpeg         FF D8 ... FF D9
 *
 * The parser is pure (no I/O, no logging) so test_core can drive it byte by byte. Bad magic,
 * an unknown v, or a payload_len outside 1 .. SKV2_MAX_PAYLOAD makes it scan forward for the
 * next "SKV2" (resync); the counters say what happened and the caller logs.
 */

#include <stddef.h>
#include <stdint.h>

#define SEEKER_PORT_VIDEO 47106
#define SKV2_HEADER_LEN 36
#define SKV2_VERSION 2
#define SKV2_MAX_PAYLOAD 2097152u
#define SKV2_NODE_LEN 8

typedef struct {
    uint8_t v, flags;
    uint16_t reserved;
    char node_id[SKV2_NODE_LEN + 1]; /* always NUL terminated */
    uint64_t ts_ms;
    uint16_t width, height;
    uint32_t seq, payload_len;
} skv2_header;

enum { SKV2_OK = 0, SKV2_ERR_MAGIC = -1, SKV2_ERR_VERSION = -2, SKV2_ERR_LEN = -3 };

int skv2_header_decode(const uint8_t b[SKV2_HEADER_LEN], skv2_header *h);
/* Writes magic plus every field of h (node_id is truncated to 8 bytes and NUL padded). */
void skv2_header_encode(const skv2_header *h, uint8_t out[SKV2_HEADER_LEN]);

typedef struct {
    /* Counters since init. */
    uint64_t frames;      /* complete frames */
    uint64_t dropped;     /* complete frames replaced by a newer one before skv2_parser_take */
    uint64_t resyncs;     /* times the stream lost sync (each scan for "SKV2" counts once) */
    uint64_t bad_version; /* headers with the right magic but an unknown v */
    uint64_t bad_len;     /* headers with payload_len 0 or above SKV2_MAX_PAYLOAD */
    uint64_t bytes;
    int last_bad_v;

    /* Internal state. */
    uint8_t hdr[SKV2_HEADER_LEN];
    size_t hdr_have;
    int in_payload, scanning;
    skv2_header cur;
    uint8_t *buf;
    size_t cap, have;
    skv2_header ready_hdr;
    uint8_t *ready;
    size_t ready_cap, ready_len;
    int has_ready;
} skv2_parser;

void skv2_parser_init(skv2_parser *p);
/* Forgets any partial frame and the unread ready frame (new connection); keeps buffers and
 * counters. */
void skv2_parser_reset(skv2_parser *p);
void skv2_parser_free(skv2_parser *p);
/* Consumes n bytes. Returns the number of frames completed by this call (only the newest is
 * kept), or -1 if a payload buffer could not be allocated (the parser resyncs). */
int skv2_parser_feed(skv2_parser *p, const uint8_t *data, size_t n);
/* Hands out the newest complete frame once. Returns 1 and fills h, jpeg, len, or 0 if none.
 * jpeg stays valid until the next feed, reset or free. */
int skv2_parser_take(skv2_parser *p, skv2_header *h, const uint8_t **jpeg, size_t *len);

#endif
