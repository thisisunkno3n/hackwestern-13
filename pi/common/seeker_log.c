#include "seeker_log.h"
#include "seeker_time.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static char g_name[32] = "seeker";

void seeker_log_init(const char *process_name)
{
    snprintf(g_name, sizeof g_name, "%s", process_name);
    setvbuf(stderr, NULL, _IOLBF, 0);
}

const char *seeker_log_name(void)
{
    return g_name;
}

void seeker_log(const char *fmt, ...)
{
    char line[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    fprintf(stderr, "[%s] %s\n", g_name, line);
}

void seeker_event(const char *json_fields)
{
    FILE *f = fopen("logs/events.jsonl", "a");
    if (!f)
        return;
    fprintf(f, "{\"t_mono_ms\":%lld,\"src\":\"%s\",%s}\n", (long long)seeker_now_ms(), g_name,
            json_fields);
    fclose(f);
}
