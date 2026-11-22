/*
 * HOST STUB, NOT QNX. This is not the QNX Screen header and none of these constant values are
 * QNX's. It declares only the subset that display/sink_screen.c calls, with the signatures from the
 * QNX 8.0 Screen docs, so the sink compiles and runs on a Linux or macOS host. The implementation
 * (host-stub/screen_stub.c) mallocs buffers and, if SEEKER_STUB_PPM is set, writes every 30th
 * posted frame to that path. Never put this directory on the include path of a qnx build.
 */
#ifndef SEEKER_HOST_STUB_SCREEN_H
#define SEEKER_HOST_STUB_SCREEN_H

#include <stdint.h>

typedef struct stub_screen_context *screen_context_t;
typedef struct stub_screen_window *screen_window_t;
typedef struct stub_screen_buffer *screen_buffer_t;
typedef struct stub_screen_display *screen_display_t;
typedef struct stub_screen_event *screen_event_t;

enum {
    SCREEN_APPLICATION_CONTEXT = 0,
    SCREEN_EVENT_NONE = 0,
    SCREEN_FORMAT_RGBX8888 = 8,
    SCREEN_USAGE_WRITE = 1 << 1,
    SCREEN_USAGE_NATIVE = 1 << 3,
};

enum {
    SCREEN_PROPERTY_TYPE = 1,
    SCREEN_PROPERTY_DISPLAY_COUNT,
    SCREEN_PROPERTY_DISPLAYS,
    SCREEN_PROPERTY_SIZE,
    SCREEN_PROPERTY_DISPLAY,
    SCREEN_PROPERTY_FORMAT,
    SCREEN_PROPERTY_USAGE,
    SCREEN_PROPERTY_BUFFER_SIZE,
    SCREEN_PROPERTY_SWAP_INTERVAL,
    SCREEN_PROPERTY_ZORDER,
    SCREEN_PROPERTY_RENDER_BUFFERS,
    SCREEN_PROPERTY_POINTER,
    SCREEN_PROPERTY_STRIDE,
    SCREEN_PROPERTY_SOURCE_SIZE,
    SCREEN_PROPERTY_SOURCE_POSITION,
};

int screen_create_context(screen_context_t *pctx, int flags);
int screen_destroy_context(screen_context_t ctx);
int screen_get_context_property_iv(screen_context_t ctx, int pname, int *param);
int screen_get_context_property_pv(screen_context_t ctx, int pname, void **param);
int screen_get_display_property_iv(screen_display_t disp, int pname, int *param);

int screen_create_window(screen_window_t *pwin, screen_context_t ctx);
int screen_destroy_window(screen_window_t win);
int screen_set_window_property_iv(screen_window_t win, int pname, const int *param);
int screen_set_window_property_pv(screen_window_t win, int pname, void **param);
int screen_get_window_property_pv(screen_window_t win, int pname, void **param);
int screen_create_window_buffers(screen_window_t win, int count);
int screen_post_window(screen_window_t win, screen_buffer_t buf, int count,
                       const int *dirty_rects, int flags);

int screen_get_buffer_property_pv(screen_buffer_t buf, int pname, void **param);
int screen_get_buffer_property_iv(screen_buffer_t buf, int pname, int *param);

int screen_create_event(screen_event_t *pev);
int screen_destroy_event(screen_event_t ev);
int screen_get_event(screen_context_t ctx, screen_event_t ev, uint64_t timeout);
int screen_get_event_property_iv(screen_event_t ev, int pname, int *param);

#endif
