#include "seeker_sv.h"

#include <ctype.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <sys/wait.h>

void sv_backoff_init(sv_backoff *b, int min_ms, int max_ms, int stable_ms)
{
    b->min_ms = min_ms > 0 ? min_ms : 1;
    b->max_ms = max_ms > b->min_ms ? max_ms : b->min_ms;
    b->stable_ms = stable_ms;
    b->failures = 0;
}

int sv_backoff_next(sv_backoff *b, int64_t uptime_ms)
{
    if (uptime_ms >= b->stable_ms)
        b->failures = 0;
    b->failures++;
    int64_t d = b->min_ms;
    for (int i = 1; i < b->failures && d < b->max_ms; i++)
        d *= 2;
    return d > b->max_ms ? b->max_ms : (int)d;
}

int sv_parse_prio(const char *spec, int *prio, int *policy)
{
    *prio = 0;
    *policy = SCHED_RR;
    if (!spec || !*spec)
        return 0;
    const char *p = spec;
    long v = 0;
    if (!isdigit((unsigned char)*p))
        return -1;
    while (isdigit((unsigned char)*p)) {
        v = v * 10 + (*p++ - '0');
        if (v > 255)
            return -1;
    }
    if (*p == 'f')
        *policy = SCHED_FIFO;
    else if (*p == 'r')
        *policy = SCHED_RR;
    else if (*p == 'o')
        *policy = SCHED_OTHER;
    else if (*p != '\0')
        return -1;
    if (*p && p[1] != '\0')
        return -1;
    *prio = (int)v;
    return 0;
}

const char *sv_prio_str(int prio, int policy, char *buf, size_t n)
{
    if (prio <= 0) {
        snprintf(buf, n, "inherit");
        return buf;
    }
    char c = policy == SCHED_FIFO ? 'f' : policy == SCHED_OTHER ? 'o' : 'r';
    snprintf(buf, n, "%d%c", prio, c);
    return buf;
}

const char *sv_signal_name(int sig, char *buf, size_t n)
{
    static const struct {
        int sig;
        const char *name;
    } names[] = {
        { SIGHUP, "SIGHUP" },   { SIGINT, "SIGINT" },   { SIGQUIT, "SIGQUIT" },
        { SIGILL, "SIGILL" },   { SIGTRAP, "SIGTRAP" }, { SIGABRT, "SIGABRT" },
        { SIGFPE, "SIGFPE" },   { SIGKILL, "SIGKILL" }, { SIGBUS, "SIGBUS" },
        { SIGSEGV, "SIGSEGV" }, { SIGPIPE, "SIGPIPE" }, { SIGALRM, "SIGALRM" },
        { SIGTERM, "SIGTERM" }, { SIGUSR1, "SIGUSR1" }, { SIGUSR2, "SIGUSR2" },
    };
    for (size_t i = 0; i < sizeof names / sizeof names[0]; i++)
        if (names[i].sig == sig) {
            snprintf(buf, n, "%s", names[i].name);
            return buf;
        }
    snprintf(buf, n, "signal %d", sig);
    return buf;
}

int sv_parse_port(const char *s)
{
    if (!s || !*s)
        return -1;
    long v = 0;
    for (const char *p = s; *p; p++) {
        if (!isdigit((unsigned char)*p))
            return -1;
        v = v * 10 + (*p - '0');
        if (v > 65535)
            return -1;
    }
    return v >= 1 ? (int)v : -1;
}

int sv_valid_node_id(const char *s, int max_len)
{
    if (!s || !*s)
        return 0;
    int n = 0;
    for (const char *p = s; *p; p++, n++) {
        if (!isalnum((unsigned char)*p) && *p != '_' && *p != '-')
            return 0;
    }
    return n <= max_len;
}

const char *sv_describe_status(int status, char *buf, size_t n)
{
    char sig[24];
    if (WIFEXITED(status))
        snprintf(buf, n, "exited with status %d", WEXITSTATUS(status));
    else if (WIFSIGNALED(status))
        snprintf(buf, n, "killed by %s (%d)", sv_signal_name(WTERMSIG(status), sig, sizeof sig),
                 WTERMSIG(status));
    else
        snprintf(buf, n, "ended with wait status 0x%x", (unsigned)status);
    return buf;
}
