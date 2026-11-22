#ifndef SEEKER_ANIM_H
#define SEEKER_ANIM_H

/*
 * The plan theme's animations as one switch (hud_display --anim on|off, display.anim): the alert
 * animations (anim_alert.h), the person animations (anim_person.h) and the plan map animations
 * (anim_map.h). All three run on one frame time, hud_state.now_ms, which hud_render hands to each
 * module (and to the wearer view as wv_view.clock_ms). Off draws exactly the still plan theme.
 * The classic theme never animates.
 */

#include "anim_alert.h"
#include "anim_map.h"
#include "anim_person.h"

static inline void anim_set_enabled(int on)
{
    anim_alert_set_enabled(on);
    anim_person_set_enabled(on);
    anim_map_set_enabled(on);
}

static inline int anim_enabled(void)
{
    return anim_alert_enabled() && anim_person_enabled() && anim_map_enabled();
}

#endif
