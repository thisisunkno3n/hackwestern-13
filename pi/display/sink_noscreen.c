#include "sink.h"
#include "seeker_log.h"

int seeker_sink_screen_open(seeker_sink *s, const seeker_config *cfg)
{
    (void)s;
    (void)cfg;
    seeker_log("this build has no Screen sink; rebuild with SEEKER_TARGET=qnx or "
               "SEEKER_SCREEN_STUB=ON, or use --sink ppm|none");
    return -1;
}
