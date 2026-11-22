#include "sink.h"
#include "seeker_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void seeker_sink_blit(const seeker_sink *s, const canvas *c, uint32_t *dst, int dst_stride_px)
{
    int w = c->w < s->w ? c->w : s->w;
    int h = c->h < s->h ? c->h : s->h;
    for (int y = 0; y < h; y++) {
        int sy = s->mirror_v ? h - 1 - y : y;
        const uint32_t *src = c->px + sy * c->stride;
        uint32_t *d = dst + y * dst_stride_px;
        if (!s->mirror_h && !s->swap_rb) {
            memcpy(d, src, (size_t)w * 4);
            continue;
        }
        for (int x = 0; x < w; x++) {
            uint32_t p = src[s->mirror_h ? w - 1 - x : x];
            if (s->swap_rb)
                p = (p & 0xff00ff00u) | ((p >> 16) & 0xffu) | ((p & 0xffu) << 16);
            d[x] = p;
        }
    }
}

int seeker_sink_write_ppm(const char *path, const canvas *c)
{
    return seeker_sink_write_ppm_note(path, c, NULL);
}

int seeker_sink_write_ppm_note(const char *path, const canvas *c, const char *note)
{
    char tmp[300];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE *f = fopen(tmp, "wb");
    if (!f)
        return -1;
    fputs("P6\n", f);
    if (note && note[0]) {
        fputs("# ", f);
        for (const char *p = note; *p; p++)
            fputc(*p >= 0x20 && *p < 0x7f ? *p : '?', f);
        fputc('\n', f);
    }
    fprintf(f, "%d %d\n255\n", c->w, c->h);
    unsigned char *row = malloc((size_t)c->w * 3);
    for (int y = 0; row && y < c->h; y++) {
        for (int x = 0; x < c->w; x++) {
            uint32_t p = c->px[y * c->stride + x];
            row[x * 3] = (unsigned char)(p >> 16);
            row[x * 3 + 1] = (unsigned char)(p >> 8);
            row[x * 3 + 2] = (unsigned char)p;
        }
        fwrite(row, 3, (size_t)c->w, f);
    }
    free(row);
    fclose(f);
    return rename(tmp, path);
}

/* The part of a hud_frame_note that is state, not a counter: " video=.. drawn=.. node=..". */
static void note_state(const char *note, char *out, size_t n)
{
    const char *a = strstr(note, " video=");
    if (!a)
        a = note;
    const char *e = strstr(a, " seq=");
    size_t len = e ? (size_t)(e - a) : strlen(a);
    if (len >= n)
        len = n - 1;
    memcpy(out, a, len);
    out[len] = '\0';
}

/* Every ppm_every-th frame, plus the first frame of every new video state: a state that lasts
 * only a few frames (one video frame, fresh for 500 ms) always lands in a snapshot even when
 * slow PPM writes stretch the frame time on the Pi. */
static int ppm_present(seeker_sink *s, const canvas *c)
{
    char state[sizeof s->ppm_state];
    note_state(s->ppm_note, state, sizeof state);
    s->frames++;
    int due = s->ppm_every > 0 && s->frames % s->ppm_every == 1 % s->ppm_every;
    if (due || strcmp(state, s->ppm_state) != 0) {
        seeker_sink_write_ppm_note(s->ppm_path, c, s->ppm_note);
        memcpy(s->ppm_state, state, sizeof state);
    }
    return 0;
}

static int none_present(seeker_sink *s, const canvas *c)
{
    (void)c;
    s->frames++;
    return 0;
}

static void plain_close(seeker_sink *s)
{
    (void)s;
}

int seeker_sink_open(seeker_sink *s, const char *kind, const seeker_config *cfg)
{
    s->w = cfg->width;
    s->h = cfg->height;
    s->mirror_h = !strcmp(cfg->mirror, "h");
    s->mirror_v = !strcmp(cfg->mirror, "v");
    s->swap_rb = cfg->swap_rb;
    s->close = plain_close;
    if (!strcmp(kind, "screen"))
        return seeker_sink_screen_open(s, cfg);
    if (!strcmp(kind, "ppm")) {
        if (!s->ppm_path[0])
            snprintf(s->ppm_path, sizeof s->ppm_path, "logs/hud.ppm");
        if (s->ppm_every <= 0)
            s->ppm_every = 30;
        s->present = ppm_present;
        return 0;
    }
    if (!strcmp(kind, "none")) {
        s->present = none_present;
        return 0;
    }
    seeker_log("unknown sink %s (use screen, ppm or none)", kind);
    return -1;
}
