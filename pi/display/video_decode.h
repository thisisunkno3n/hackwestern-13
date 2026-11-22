#ifndef SEEKER_VIDEO_DECODE_H
#define SEEKER_VIDEO_DECODE_H

/*
 * JPEG decode for IF-5.6 frames with the vendored stb_image (third_party/stb, public domain or
 * MIT, no system libraries), plus the letterbox scale into the HUD canvas.
 */

#include <stddef.h>
#include <stdint.h>

#define VIDEO_MAX_DIM 4096 /* larger JPEGs are rejected before any pixel memory is allocated */

/* Decodes baseline or progressive JPEG to packed RGB (3 bytes per pixel, rows top to bottom).
 * Returns a buffer to release with video_jpeg_free, or NULL (not a JPEG, corrupt, too large). */
uint8_t *video_jpeg_decode(const uint8_t *jpg, size_t len, int *w, int *h);
void video_jpeg_free(uint8_t *rgb);

/* Largest rectangle with the source aspect ratio centred inside dw x dh: rect = {x, y, w, h}. */
void video_fit_rect(int sw, int sh, int dw, int dh, int rect[4]);

/* Nearest-neighbour scale of RGB (sw x sh) into rect of a 0x00RRGGBB destination (dw x dh,
 * stride in pixels); everything outside rect is painted black. */
void video_blit_rgb(const uint8_t *rgb, int sw, int sh, uint32_t *dst, int dw, int dh, int stride,
                    const int rect[4]);

#endif
