#ifndef SEEKER_HUD_FONTS_H
#define SEEKER_HUD_FONTS_H

/*
 * The HUD's font set (plan theme): Atkinson Hyperlegible Next in three weights for words and
 * Atkinson Hyperlegible Mono in two for numbers, as TTF bytes compiled in (hud_fonts.c) so the
 * Pi needs no font file. Made by tools/gen_hud_fonts.py; another family keeps these names.
 */

extern const char hud_font_family[]; /* for the logs */
extern const unsigned char hud_font_sans_regular[];
extern const unsigned int hud_font_sans_regular_len;
extern const unsigned char hud_font_sans_medium[];
extern const unsigned int hud_font_sans_medium_len;
extern const unsigned char hud_font_sans_semibold[];
extern const unsigned int hud_font_sans_semibold_len;
extern const unsigned char hud_font_mono_regular[];
extern const unsigned int hud_font_mono_regular_len;
extern const unsigned char hud_font_mono[]; /* medium */
extern const unsigned int hud_font_mono_len;

#endif
