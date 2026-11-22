/*
 * FALLBACK Screen API declarations for building on a QNX 8.0 target that
 * ships libscreen but not <screen/screen.h>. The official header is part of
 * the QNX Software Development Platform (SDP) and is not published as source.
 *
 * This is NOT the official QNX header and is NOT a QNX product. It declares
 * ONLY the types, constants, and functions used by display/sink_screen.c.
 * Every constant value and prototype below is cited from the public QNX SDP
 * 8.0 Screen documentation:
 *   https://www.qnx.com/developers/docs/8.0/
 *
 * Build wiring (CMakeLists.txt and scripts/build_pi.sh) adds
 * -I third_party/qnx-screen only when the real header is absent
 * ($QNX_TARGET/usr/include/screen/screen.h, /usr/include/screen/screen.h,
 * or /system/include/screen/screen.h). Keep host-stub/ off the include
 * path of any QNX build; that stub's constant values are intentionally
 * not QNX's and exist only for laptop compile checks.
 *
 * These declarations are original to this project, based on the publicly
 * documented API. The official screen.h remains proprietary to QNX Software
 * Systems / BlackBerry QNX.
 *
 * A host compile is not a QNX test pass. Nothing counts as tested until it
 * runs on the QNX Pi, built there with gcc and linked to the on-device
 * libscreen.
 */
#ifndef SEEKER_FALLBACK_QNX_SCREEN_H
#define SEEKER_FALLBACK_QNX_SCREEN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Opaque handle types. Official typedefs:
 *   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_context_t.html
 *   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_window_t.html
 *   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_buffer_t.html
 *   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_display_t.html
 *   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_event_t.html
 */
typedef struct _screen_context *screen_context_t;
typedef struct _screen_window *screen_window_t;
typedef struct _screen_buffer *screen_buffer_t;
typedef struct _screen_display *screen_display_t;
typedef struct _screen_event *screen_event_t;

/*
 * Screen context types (only SCREEN_APPLICATION_CONTEXT is used here).
 * https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/group__screen__contexts_1Screen_Context_Types.html
 */
enum {
    SCREEN_APPLICATION_CONTEXT = 0
};

/*
 * Screen event types (only SCREEN_EVENT_NONE is used here).
 * https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/group__screen__events_1Screen_Event_Types.html
 */
enum {
    SCREEN_EVENT_NONE = 0
};

/*
 * Screen pixel format types (only SCREEN_FORMAT_RGBX8888 is used here).
 * https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_8h_1Screen_Pixel_Format_Types.html
 *
 * Official listing (QNX SDP 8.0, page updated June 11, 2026):
 *   SCREEN_FORMAT_RGBA8888 = 8,
 *   SCREEN_FORMAT_RGBX8888 = 9,
 * Do not confuse these two. The host-stub uses different placeholder values.
 */
enum {
    SCREEN_FORMAT_RGBX8888 = 9
};

/*
 * Screen usage flag types (only WRITE and NATIVE are used here).
 * https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_8h_1Screen_Usage_Flag_Types.html
 */
enum {
    SCREEN_USAGE_WRITE = (1 << 2),
    SCREEN_USAGE_NATIVE = (1 << 3)
};

/*
 * Screen property types (only the properties sink_screen.c uses).
 * https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_8h_1Screen_Property_Types.html
 * Rechecked against that page (updated November 2026) for the downscale option
 * (hud_display --screen-buffer): SOURCE_POSITION = 41 and SOURCE_SIZE = 42,
 * "the width and height of a rectangular region within the window buffer
 * representing the source viewport of the window", which Screen scales to the
 * window's SCREEN_PROPERTY_SIZE.
 */
enum {
    SCREEN_PROPERTY_BUFFER_SIZE = 5,
    SCREEN_PROPERTY_DISPLAY = 11,
    SCREEN_PROPERTY_FORMAT = 14,
    SCREEN_PROPERTY_POINTER = 34,
    SCREEN_PROPERTY_RENDER_BUFFERS = 37,
    SCREEN_PROPERTY_SIZE = 40,
    SCREEN_PROPERTY_SOURCE_POSITION = 41,
    SCREEN_PROPERTY_SOURCE_SIZE = 42,
    SCREEN_PROPERTY_STRIDE = 44,
    SCREEN_PROPERTY_SWAP_INTERVAL = 45,
    SCREEN_PROPERTY_TYPE = 47,
    SCREEN_PROPERTY_USAGE = 48,
    SCREEN_PROPERTY_ZORDER = 54,
    SCREEN_PROPERTY_DISPLAY_COUNT = 59,
    SCREEN_PROPERTY_DISPLAYS = 60
};

/* Context */
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_create_context.html */
int screen_create_context(screen_context_t *pctx, int flags);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_destroy_context.html */
int screen_destroy_context(screen_context_t ctx);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_get_context_property_iv.html */
int screen_get_context_property_iv(screen_context_t ctx, int pname, int *param);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_get_context_property_pv.html */
int screen_get_context_property_pv(screen_context_t ctx, int pname, void **param);

/* Display */
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_get_display_property_iv.html */
int screen_get_display_property_iv(screen_display_t disp, int pname, int *param);

/* Window */
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_create_window.html */
int screen_create_window(screen_window_t *pwin, screen_context_t ctx);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_destroy_window.html */
int screen_destroy_window(screen_window_t win);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_set_window_property_iv.html */
int screen_set_window_property_iv(screen_window_t win, int pname, const int *param);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_set_window_property_pv.html */
int screen_set_window_property_pv(screen_window_t win, int pname, void **param);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_get_window_property_pv.html */
int screen_get_window_property_pv(screen_window_t win, int pname, void **param);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_create_window_buffers.html */
int screen_create_window_buffers(screen_window_t win, int count);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_post_window.html */
int screen_post_window(screen_window_t win, screen_buffer_t buf, int count,
                       const int *dirty_rects, int flags);

/* Buffer */
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_get_buffer_property_pv.html */
int screen_get_buffer_property_pv(screen_buffer_t buf, int pname, void **param);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_get_buffer_property_iv.html */
int screen_get_buffer_property_iv(screen_buffer_t buf, int pname, int *param);

/* Event */
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_create_event.html */
int screen_create_event(screen_event_t *pev);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_destroy_event.html */
int screen_destroy_event(screen_event_t ev);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_get_event.html */
int screen_get_event(screen_context_t ctx, screen_event_t ev, uint64_t timeout);
/* https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_get_event_property_iv.html */
int screen_get_event_property_iv(screen_event_t ev, int pname, int *param);

#ifdef __cplusplus
}
#endif

#endif
