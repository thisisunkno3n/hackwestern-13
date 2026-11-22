/*
 * QNX Screen sink (IF-1 backend, the only QNX-only file in display/). Fullscreen window on the
 * display at cfg->display_index (0 is assumed to be HDMI0), two CPU-writable buffers, one post per
 * frame. Built against the real <screen/screen.h> for SEEKER_TARGET=qnx when that header is on
 * the sysroot; otherwise against third_party/qnx-screen/screen/screen.h (documented fallback, not
 * the SDP header). Host compile checks use host-stub/screen/screen.h (NOT QNX, and not this
 * fallback).
 *
 * Checked against the QNX 8.0 Screen docs (qnx.com/developers/docs/8.0, com.qnx.doc.screen):
 * screen_create_context, screen_create_window, screen_create_window_buffers (USAGE must include
 * SCREEN_USAGE_WRITE before creating buffers to read SCREEN_PROPERTY_POINTER),
 * screen_set_window_property_iv (FORMAT, USAGE, BUFFER_SIZE, SIZE, SWAP_INTERVAL, ZORDER),
 * screen_get_buffer_property_pv (POINTER), screen_post_window (flags 0, first RENDER_BUFFERS entry).
 * NOT verified on the Pi: that display index 0 is HDMI0, that SCREEN_FORMAT_RGBX8888 stores a
 * uint32 as 0x00RRGGBB (set display.swap_rb if red and blue come out swapped), and whether a
 * window from a non-root SSH session shows above the image's default desktop (ZORDER below).
 */

#include "sink.h"
#include "seeker_log.h"
#include "seeker_time.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <screen/screen.h>

typedef struct {
    screen_context_t ctx;
    screen_window_t win;
    screen_event_t ev;
} screen_priv;

static void drain_events(screen_priv *p)
{
    /* Nothing is handled, but the context's event queue is emptied so it cannot grow. */
    for (int guard = 0; guard < 64; guard++) {
        if (screen_get_event(p->ctx, p->ev, 0) != 0)
            return;
        int type = SCREEN_EVENT_NONE;
        screen_get_event_property_iv(p->ev, SCREEN_PROPERTY_TYPE, &type);
        if (type == SCREEN_EVENT_NONE)
            return;
    }
}

static int screen_present(seeker_sink *s, const canvas *c)
{
    screen_priv *p = s->priv;
    int64_t u0 = seeker_now_us();
    screen_buffer_t bufs[2] = { 0, 0 };
    if (screen_get_window_property_pv(p->win, SCREEN_PROPERTY_RENDER_BUFFERS, (void **)bufs) != 0)
        return -1;
    void *ptr = NULL;
    int stride = 0;
    if (screen_get_buffer_property_pv(bufs[0], SCREEN_PROPERTY_POINTER, &ptr) != 0 || !ptr ||
        screen_get_buffer_property_iv(bufs[0], SCREEN_PROPERTY_STRIDE, &stride) != 0)
        return -1;
    int64_t u1 = seeker_now_us();
    seeker_sink_blit(s, c, (uint32_t *)ptr, stride / 4);
    int64_t u2 = seeker_now_us();
    int rect[4] = { 0, 0, s->w, s->h };
    int rc = screen_post_window(p->win, bufs[0], 1, rect, 0);
    int64_t u3 = seeker_now_us();
    s->blit_ms = (float)(u2 - u1) / 1000.0f;
    s->post_ms = (float)((u1 - u0) + (u3 - u2)) / 1000.0f; /* buffer wait + post (vsync) */
    drain_events(p);
    s->frames++;
    return rc;
}

static void screen_close(seeker_sink *s)
{
    screen_priv *p = s->priv;
    if (!p)
        return;
    if (p->ev)
        screen_destroy_event(p->ev);
    if (p->win)
        screen_destroy_window(p->win);
    if (p->ctx)
        screen_destroy_context(p->ctx);
    free(p);
    s->priv = NULL;
}

#define TRY(call)                                                                                  \
    do {                                                                                           \
        if ((call) != 0) {                                                                         \
            seeker_log("screen: %s failed: %s", #call, strerror(errno));                         \
            screen_close(s);                                                                       \
            return -1;                                                                             \
        }                                                                                          \
    } while (0)

int seeker_sink_screen_open(seeker_sink *s, const seeker_config *cfg)
{
    screen_priv *p = calloc(1, sizeof *p);
    if (!p)
        return -1;
    s->priv = p;
    s->present = screen_present;
    s->close = screen_close;

    TRY(screen_create_context(&p->ctx, SCREEN_APPLICATION_CONTEXT));
    TRY(screen_create_event(&p->ev));

    int ndisp = 0;
    TRY(screen_get_context_property_iv(p->ctx, SCREEN_PROPERTY_DISPLAY_COUNT, &ndisp));
    screen_display_t disp = NULL;
    if (ndisp > 0) {
        screen_display_t *list = calloc((size_t)ndisp, sizeof *list);
        if (list &&
            screen_get_context_property_pv(p->ctx, SCREEN_PROPERTY_DISPLAYS, (void **)list) == 0) {
            int idx = cfg->display_index < ndisp ? cfg->display_index : 0;
            disp = list[idx];
        }
        free(list);
    }
    int size[2] = { cfg->width, cfg->height };
    if (disp)
        screen_get_display_property_iv(disp, SCREEN_PROPERTY_SIZE, size);
    seeker_log("screen: %d display(s), using index %d at %dx%d", ndisp, cfg->display_index,
               size[0], size[1]);

    TRY(screen_create_window(&p->win, p->ctx));
    if (disp)
        TRY(screen_set_window_property_pv(p->win, SCREEN_PROPERTY_DISPLAY, (void **)&disp));
    int format = SCREEN_FORMAT_RGBX8888;
    TRY(screen_set_window_property_iv(p->win, SCREEN_PROPERTY_FORMAT, &format));
    int usage = SCREEN_USAGE_WRITE | SCREEN_USAGE_NATIVE;
    TRY(screen_set_window_property_iv(p->win, SCREEN_PROPERTY_USAGE, &usage));
    int buf[2] = { size[0], size[1] };
    const char *scaled = "no";
    if (s->buf_w > 0 && s->buf_h > 0 && (s->buf_w != size[0] || s->buf_h != size[1])) {
        buf[0] = s->buf_w;
        buf[1] = s->buf_h;
        scaled = "yes";
    } else if (s->buf_w > 0 && s->buf_h > 0) {
        scaled = "no (SCREEN_BUFFER is the display size)";
    }
    TRY(screen_set_window_property_iv(p->win, SCREEN_PROPERTY_BUFFER_SIZE, buf));
    TRY(screen_set_window_property_iv(p->win, SCREEN_PROPERTY_SIZE, size));
#ifdef SCREEN_NO_SOURCE_SIZE /* a screen.h without SCREEN_PROPERTY_SOURCE_SIZE: no downscale */
    if (buf[0] != size[0] || buf[1] != size[1]) {
        seeker_log("screen: --screen-buffer (SCREEN_BUFFER) unsupported by this screen.h; drawing "
                   "at the display's %dx%d", size[0], size[1]);
        scaled = "no (unsupported)";
        buf[0] = size[0];
        buf[1] = size[1];
        TRY(screen_set_window_property_iv(p->win, SCREEN_PROPERTY_BUFFER_SIZE, buf));
    }
#else
    if (buf[0] != size[0] || buf[1] != size[1]) {
        int pos[2] = { 0, 0 };
        if (screen_set_window_property_iv(p->win, SCREEN_PROPERTY_SOURCE_SIZE, buf) != 0 ||
            screen_set_window_property_iv(p->win, SCREEN_PROPERTY_SOURCE_POSITION, pos) != 0) {
            seeker_log("screen: Screen refused the %dx%d source viewport (%s); --screen-buffer "
                       "off, drawing at the display's %dx%d", buf[0], buf[1], strerror(errno),
                       size[0], size[1]);
            scaled = "no (refused)";
            buf[0] = size[0];
            buf[1] = size[1];
            TRY(screen_set_window_property_iv(p->win, SCREEN_PROPERTY_BUFFER_SIZE, buf));
            screen_set_window_property_iv(p->win, SCREEN_PROPERTY_SOURCE_SIZE, buf);
        }
    }
#endif
    /* SWAP_INTERVAL: "the minimum number of vsync periods between posts" (QNX 8.0 docs); 0 is
     * --vsync off: posts do not wait for vsync (may tear). */
    int interval = s->no_vsync ? 0 : 1;
    int iv_rc = screen_set_window_property_iv(p->win, SCREEN_PROPERTY_SWAP_INTERVAL, &interval);
    int zorder = 100; /* unverified: high enough to sit above the image's desktop window */
    screen_set_window_property_iv(p->win, SCREEN_PROPERTY_ZORDER, &zorder);
    TRY(screen_create_window_buffers(p->win, 2));

    s->w = buf[0];
    s->h = buf[1];
    seeker_log("screen: window %dx%d buffer %dx%d scaled %s, swap interval %d%s", size[0], size[1],
               buf[0], buf[1], scaled, interval, iv_rc != 0 ? " (refused)" : "");
    return 0;
}
