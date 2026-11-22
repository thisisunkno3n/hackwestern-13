#include "video_proto.h"

#include <stdlib.h>
#include <string.h>

static const uint8_t MAGIC[4] = { 'S', 'K', 'V', '2' };

static uint16_t be16(const uint8_t *b)
{
    return (uint16_t)((b[0] << 8) | b[1]);
}

static uint32_t be32(const uint8_t *b)
{
    return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}

static uint64_t be64(const uint8_t *b)
{
    return ((uint64_t)be32(b) << 32) | be32(b + 4);
}

static void put16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)(v >> 8);
    b[1] = (uint8_t)v;
}

static void put32(uint8_t *b, uint32_t v)
{
    put16(b, (uint16_t)(v >> 16));
    put16(b + 2, (uint16_t)v);
}

int skv2_header_decode(const uint8_t b[SKV2_HEADER_LEN], skv2_header *h)
{
    if (memcmp(b, MAGIC, 4) != 0)
        return SKV2_ERR_MAGIC;
    memset(h, 0, sizeof *h);
    h->v = b[4];
    h->flags = b[5];
    h->reserved = be16(b + 6);
    memcpy(h->node_id, b + 8, SKV2_NODE_LEN);
    h->node_id[SKV2_NODE_LEN] = '\0';
    h->ts_ms = be64(b + 16);
    h->width = be16(b + 24);
    h->height = be16(b + 26);
    h->seq = be32(b + 28);
    h->payload_len = be32(b + 32);
    if (h->v != SKV2_VERSION)
        return SKV2_ERR_VERSION;
    if (h->payload_len == 0 || h->payload_len > SKV2_MAX_PAYLOAD)
        return SKV2_ERR_LEN;
    return SKV2_OK;
}

void skv2_header_encode(const skv2_header *h, uint8_t out[SKV2_HEADER_LEN])
{
    memcpy(out, MAGIC, 4);
    out[4] = h->v;
    out[5] = h->flags;
    put16(out + 6, h->reserved);
    memset(out + 8, 0, SKV2_NODE_LEN);
    memcpy(out + 8, h->node_id, strnlen(h->node_id, SKV2_NODE_LEN));
    put32(out + 16, (uint32_t)(h->ts_ms >> 32));
    put32(out + 20, (uint32_t)h->ts_ms);
    put16(out + 24, h->width);
    put16(out + 26, h->height);
    put32(out + 28, h->seq);
    put32(out + 32, h->payload_len);
}

void skv2_parser_init(skv2_parser *p)
{
    memset(p, 0, sizeof *p);
}

void skv2_parser_reset(skv2_parser *p)
{
    p->hdr_have = 0;
    p->in_payload = 0;
    p->scanning = 0;
    p->have = 0;
    p->has_ready = 0;
}

void skv2_parser_free(skv2_parser *p)
{
    free(p->buf);
    free(p->ready);
    p->buf = p->ready = NULL;
    p->cap = p->ready_cap = 0;
    skv2_parser_reset(p);
}

/* 1 if the first min(n, 4) bytes of b could start a header. */
static int prefix_ok(const uint8_t *b, size_t n)
{
    return memcmp(b, MAGIC, n < 4 ? n : 4) == 0;
}

/* Drops at least one byte from the header buffer, up to the next place a header could start. */
static void resync(skv2_parser *p)
{
    if (!p->scanning) {
        p->resyncs++;
        p->scanning = 1;
    }
    size_t k = 1;
    while (k < p->hdr_have && !prefix_ok(p->hdr + k, p->hdr_have - k))
        k++;
    memmove(p->hdr, p->hdr + k, p->hdr_have - k);
    p->hdr_have -= k;
}

int skv2_parser_feed(skv2_parser *p, const uint8_t *data, size_t n)
{
    int done = 0, oom = 0;
    p->bytes += n;
    size_t i = 0;
    while (i < n) {
        if (!p->in_payload) {
            size_t take = SKV2_HEADER_LEN - p->hdr_have;
            if (take > n - i)
                take = n - i;
            memcpy(p->hdr + p->hdr_have, data + i, take);
            p->hdr_have += take;
            i += take;
            if (p->hdr_have && !prefix_ok(p->hdr, p->hdr_have))
                resync(p);
            if (p->hdr_have < SKV2_HEADER_LEN)
                continue;
            skv2_header h;
            int rc = skv2_header_decode(p->hdr, &h);
            if (rc == SKV2_ERR_VERSION) {
                p->bad_version++;
                p->last_bad_v = h.v;
            } else if (rc == SKV2_ERR_LEN) {
                p->bad_len++;
            }
            if (rc == SKV2_OK && h.payload_len > p->cap) {
                uint8_t *nb = realloc(p->buf, h.payload_len);
                if (nb) {
                    p->buf = nb;
                    p->cap = h.payload_len;
                } else {
                    oom = 1;
                    rc = SKV2_ERR_LEN;
                }
            }
            if (rc != SKV2_OK) {
                resync(p);
                continue;
            }
            p->cur = h;
            p->have = 0;
            p->hdr_have = 0;
            p->scanning = 0;
            p->in_payload = 1;
        } else {
            size_t take = p->cur.payload_len - p->have;
            if (take > n - i)
                take = n - i;
            memcpy(p->buf + p->have, data + i, take);
            p->have += take;
            i += take;
            if (p->have < p->cur.payload_len)
                continue;
            uint8_t *t = p->ready;
            size_t tc = p->ready_cap;
            p->ready = p->buf;
            p->ready_cap = p->cap;
            p->buf = t;
            p->cap = tc;
            p->ready_hdr = p->cur;
            p->ready_len = p->cur.payload_len;
            if (p->has_ready)
                p->dropped++;
            p->has_ready = 1;
            p->frames++;
            p->in_payload = 0;
            done++;
        }
    }
    return oom ? -1 : done;
}

int skv2_parser_take(skv2_parser *p, skv2_header *h, const uint8_t **jpeg, size_t *len)
{
    if (!p->has_ready)
        return 0;
    p->has_ready = 0;
    *h = p->ready_hdr;
    *jpeg = p->ready;
    *len = p->ready_len;
    return 1;
}
