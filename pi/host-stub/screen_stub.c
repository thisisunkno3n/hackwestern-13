/* HOST STUB, NOT QNX. Fake Screen for compile and smoke tests on a host; see screen/screen.h. */

#include <screen/screen.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STUB_W 1280
#define STUB_H 720

struct stub_screen_context { int unused; };
struct stub_screen_display { int size[2]; };
struct stub_screen_event { int type; };
struct stub_screen_buffer { uint32_t *px; int w, h, stride; };
struct stub_screen_window {
    int size[2];
    int buf_size[2]; /* BUFFER_SIZE, when set apart from SIZE (scaled window) */
    int usage, nbuf, front;
    long posts;
    struct stub_screen_buffer buf[2];
};

static struct stub_screen_display g_disp = { { STUB_W, STUB_H } };

int screen_create_context(screen_context_t *pctx, int flags)
{
    (void)flags;
    *pctx = calloc(1, sizeof **pctx);
    fprintf(stderr, "[screen-stub] HOST STUB, not QNX Screen\n");
    return *pctx ? 0 : -1;
}

int screen_destroy_context(screen_context_t ctx)
{
    free(ctx);
    return 0;
}

int screen_get_context_property_iv(screen_context_t ctx, int pname, int *param)
{
    (void)ctx;
    if (pname != SCREEN_PROPERTY_DISPLAY_COUNT)
        return errno = EINVAL, -1;
    *param = 1;
    return 0;
}

int screen_get_context_property_pv(screen_context_t ctx, int pname, void **param)
{
    (void)ctx;
    if (pname != SCREEN_PROPERTY_DISPLAYS)
        return errno = EINVAL, -1;
    param[0] = &g_disp;
    return 0;
}

int screen_get_display_property_iv(screen_display_t disp, int pname, int *param)
{
    if (pname != SCREEN_PROPERTY_SIZE)
        return errno = EINVAL, -1;
    param[0] = disp->size[0];
    param[1] = disp->size[1];
    return 0;
}

int screen_create_window(screen_window_t *pwin, screen_context_t ctx)
{
    (void)ctx;
    *pwin = calloc(1, sizeof **pwin);
    return *pwin ? 0 : -1;
}

int screen_destroy_window(screen_window_t win)
{
    for (int i = 0; i < 2; i++)
        free(win->buf[i].px);
    free(win);
    return 0;
}

int screen_set_window_property_iv(screen_window_t win, int pname, const int *param)
{
    /* SEEKER_STUB_NO_SOURCE_SIZE=1: act like a Screen that refuses a source viewport, to try
     * the --screen-buffer fallback on a laptop. */
    if (pname == SCREEN_PROPERTY_SOURCE_SIZE && getenv("SEEKER_STUB_NO_SOURCE_SIZE"))
        return errno = ENOTSUP, -1;
    if (pname == SCREEN_PROPERTY_BUFFER_SIZE) {
        win->buf_size[0] = param[0];
        win->buf_size[1] = param[1];
    } else if (pname == SCREEN_PROPERTY_SIZE) {
        win->size[0] = param[0];
        win->size[1] = param[1];
    } else if (pname == SCREEN_PROPERTY_USAGE) {
        win->usage = param[0];
    }
    return 0;
}

int screen_set_window_property_pv(screen_window_t win, int pname, void **param)
{
    (void)win;
    (void)pname;
    (void)param;
    return 0;
}

int screen_create_window_buffers(screen_window_t win, int count)
{
    if (count < 1 || count > 2 || !(win->usage & SCREEN_USAGE_WRITE))
        return errno = EINVAL, -1;
    for (int i = 0; i < count; i++) {
        struct stub_screen_buffer *b = &win->buf[i];
        b->w = win->buf_size[0] ? win->buf_size[0] : win->size[0];
        b->h = win->buf_size[0] ? win->buf_size[1] : win->size[1];
        b->stride = b->w * 4;
        b->px = calloc((size_t)b->w * (size_t)b->h, 4);
        if (!b->px)
            return -1;
    }
    win->nbuf = count;
    return 0;
}

int screen_get_window_property_pv(screen_window_t win, int pname, void **param)
{
    if (pname != SCREEN_PROPERTY_RENDER_BUFFERS || win->nbuf == 0)
        return errno = EINVAL, -1;
    for (int i = 0; i < win->nbuf; i++)
        param[i] = &win->buf[(win->front + 1 + i) % win->nbuf];
    return 0;
}

static void write_ppm(const char *path, const struct stub_screen_buffer *b)
{
    FILE *f = fopen(path, "wb");
    if (!f)
        return;
    fprintf(f, "P6\n%d %d\n255\n", b->w, b->h);
    for (int i = 0; i < b->w * b->h; i++) {
        uint32_t p = b->px[i];
        unsigned char rgb[3] = { (unsigned char)(p >> 16), (unsigned char)(p >> 8),
                                 (unsigned char)p };
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}

int screen_post_window(screen_window_t win, screen_buffer_t buf, int count,
                       const int *dirty_rects, int flags)
{
    (void)count;
    (void)dirty_rects;
    (void)flags;
    win->front = (int)(buf - win->buf);
    const char *path = getenv("SEEKER_STUB_PPM");
    if (path && win->posts % 30 == 0)
        write_ppm(path, buf);
    win->posts++;
    return 0;
}

int screen_get_buffer_property_pv(screen_buffer_t buf, int pname, void **param)
{
    if (pname != SCREEN_PROPERTY_POINTER)
        return errno = EINVAL, -1;
    *param = buf->px;
    return 0;
}

int screen_get_buffer_property_iv(screen_buffer_t buf, int pname, int *param)
{
    if (pname != SCREEN_PROPERTY_STRIDE)
        return errno = EINVAL, -1;
    *param = buf->stride;
    return 0;
}

int screen_create_event(screen_event_t *pev)
{
    *pev = calloc(1, sizeof **pev);
    return *pev ? 0 : -1;
}

int screen_destroy_event(screen_event_t ev)
{
    free(ev);
    return 0;
}

int screen_get_event(screen_context_t ctx, screen_event_t ev, uint64_t timeout)
{
    (void)ctx;
    (void)timeout;
    ev->type = SCREEN_EVENT_NONE;
    return 0;
}

int screen_get_event_property_iv(screen_event_t ev, int pname, int *param)
{
    if (pname != SCREEN_PROPERTY_TYPE)
        return errno = EINVAL, -1;
    *param = ev->type;
    return 0;
}
