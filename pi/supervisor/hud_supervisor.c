/*
 * hud_supervisor: spawns the children listed in config supervisor.children with their IF-3
 * priorities, watches exits and /seeker_health heartbeats at 20 Hz, and restarts a dead child
 * after health.backoff_ms (at most once per second, IF-6). `slay hud_fusion` on the Pi sends
 * SIGTERM; the child exits, is reaped here and comes back.
 *
 *   hud_supervisor [--config config/seeker.json] [--only a,b] [--run-for s]
 *
 * Children get priorities through posix_spawnattr_setschedpolicy/setschedparam with
 * POSIX_SPAWN_SETSCHEDULER (POSIX, documented in the QNX 8.0 libc reference). Priorities above the
 * unprivileged limit need PROCMGR_AID_PRIORITY on QNX (run under `sudo`, or `on -p`), and root on
 * Linux; without it the child is spawned at the inherited priority and a warning is logged.
 * Each child's stdout and stderr go to logs/<name>.log.
 */

#include "seeker_config.h"
#include "seeker_health.h"
#include "seeker_log.h"
#include "seeker_time.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

#define START_GRACE_MS 2000
#define MIN_RESTART_GAP_MS 1000

typedef struct {
    const seeker_child_cfg *cfg;
    pid_t pid;
    int slot;
    int restarts;
    int64_t started_ms;
    int64_t restart_at_ms; /* 0 = no restart pending */
    int warned_prio;
} child;

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static int policy_of(const char *p)
{
    if (!strcmp(p, "fifo"))
        return SCHED_FIFO;
    if (!strcmp(p, "rr"))
        return SCHED_RR;
    return SCHED_OTHER;
}

static pid_t spawn_child(child *c)
{
    char *argv[SEEKER_MAX_ARGS + 1];
    for (int i = 0; i < c->cfg->argc; i++)
        argv[i] = (char *)c->cfg->argv[i];
    argv[c->cfg->argc] = NULL;

    char logpath[96];
    snprintf(logpath, sizeof logpath, "logs/%s.log", c->cfg->name);
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, STDOUT_FILENO, logpath, O_WRONLY | O_CREAT | O_APPEND,
                                     0644);
    posix_spawn_file_actions_adddup2(&fa, STDOUT_FILENO, STDERR_FILENO);

    pid_t pid = -1;
    int rc = -1;
    if (c->cfg->prio > 0) {
        posix_spawnattr_t at;
        posix_spawnattr_init(&at);
        struct sched_param sp;
        memset(&sp, 0, sizeof sp);
        sp.sched_priority = c->cfg->prio;
        posix_spawnattr_setschedpolicy(&at, policy_of(c->cfg->policy));
        posix_spawnattr_setschedparam(&at, &sp);
        posix_spawnattr_setflags(&at, POSIX_SPAWN_SETSCHEDULER);
        rc = strchr(argv[0], '/') ? posix_spawn(&pid, argv[0], &fa, &at, argv, environ)
                                  : posix_spawnp(&pid, argv[0], &fa, &at, argv, environ);
        posix_spawnattr_destroy(&at);
        if (rc != 0 && !c->warned_prio) {
            seeker_log("%s: spawn at prio %d %s failed (%s); spawning at inherited priority",
                       c->cfg->name, c->cfg->prio, c->cfg->policy, strerror(rc));
            c->warned_prio = 1;
        }
    }
    if (rc != 0)
        rc = strchr(argv[0], '/') ? posix_spawn(&pid, argv[0], &fa, NULL, argv, environ)
                                  : posix_spawnp(&pid, argv[0], &fa, NULL, argv, environ);
    posix_spawn_file_actions_destroy(&fa);
    if (rc != 0) {
        seeker_log("%s: spawn %s failed: %s", c->cfg->name, argv[0], strerror(rc));
        return -1;
    }
    return pid;
}

static void start(child *c, seeker_health_table *h, int64_t now)
{
    if (h && c->slot >= 0) {
        h->slot[c->slot].last_beat_ms = 0;
        h->slot[c->slot].state = SH_STARTING;
        h->slot[c->slot].restarts = c->restarts;
    }
    c->pid = spawn_child(c);
    c->started_ms = now;
    c->restart_at_ms = 0;
    if (c->pid > 0)
        seeker_log("started %s pid %d (restarts %d)", c->cfg->name, (int)c->pid, c->restarts);
    else
        c->restart_at_ms = now + MIN_RESTART_GAP_MS;
}

static void mark_dead(child *c, seeker_health_table *h, int64_t now, const char *why,
                      int backoff_ms)
{
    seeker_log("%s pid %d dead: %s; restart in %d ms", c->cfg->name, (int)c->pid, why,
               backoff_ms);
    char ev[160];
    snprintf(ev, sizeof ev, "\"event\":\"restart\",\"process\":\"%s\",\"reason\":\"%s\"",
             c->cfg->name, why);
    seeker_event(ev);
    if (h && c->slot >= 0)
        h->slot[c->slot].state = SH_DEAD;
    c->pid = 0;
    int64_t at = now + backoff_ms;
    if (at < c->started_ms + MIN_RESTART_GAP_MS)
        at = c->started_ms + MIN_RESTART_GAP_MS;
    c->restart_at_ms = at;
    c->restarts++;
}

static int wanted(const char *only, const char *name)
{
    if (!only)
        return 1;
    char list[256];
    snprintf(list, sizeof list, "%s", only);
    for (char *tok = strtok(list, ","); tok; tok = strtok(NULL, ","))
        if (!strcmp(tok, name) || (!strncmp(name, "hud_", 4) && !strcmp(tok, name + 4)))
            return 1;
    return 0;
}

int main(int argc, char **argv)
{
    const char *cfg_path = "config/seeker.json", *only = NULL;
    double run_for = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--config") && i + 1 < argc)
            cfg_path = argv[++i];
        else if (!strcmp(argv[i], "--only") && i + 1 < argc)
            only = argv[++i];
        else if (!strcmp(argv[i], "--run-for") && i + 1 < argc)
            run_for = atof(argv[++i]);
        else {
            fprintf(stderr, "usage: %s [--config f] [--only a,b] [--run-for s]\n", argv[0]);
            return 2;
        }
    }
    seeker_log_init("hud_supervisor");
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    mkdir("logs", 0755);

    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path);
    if (cfg.n_children == 0) {
        seeker_log("no supervisor.children in %s", cfg_path);
        return 1;
    }

    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = cfg.supervisor_prio;
    int rc = pthread_setschedparam(pthread_self(), policy_of(cfg.supervisor_policy), &sp);
    if (rc != 0)
        seeker_log("own priority %d %s not set (%s)", cfg.supervisor_prio, cfg.supervisor_policy,
                   strerror(rc));

    seeker_health_table *h = seeker_health_open(1);
    if (!h)
        seeker_log("shm_open %s failed (%s): exit-based restarts only", SEEKER_HEALTH_NAME,
                   strerror(errno));
    int self_slot = seeker_health_slot_for(h, "hud_supervisor");

    child kids[SEEKER_MAX_CHILDREN];
    int nk = 0;
    int64_t now = seeker_now_ms();
    for (int i = 0; i < cfg.n_children; i++) {
        if (!wanted(only, cfg.children[i].name))
            continue;
        child *c = &kids[nk++];
        memset(c, 0, sizeof *c);
        c->cfg = &cfg.children[i];
        c->slot = seeker_health_slot_for(h, c->cfg->name);
        start(c, h, now);
    }

    const int64_t t_start = now;
    int64_t next_status = now + 1000;
    while (!g_stop) {
        now = seeker_now_ms();
        if (run_for > 0 && now - t_start >= (int64_t)(run_for * 1000))
            break;
        int status;
        pid_t p;
        while ((p = waitpid(-1, &status, WNOHANG)) > 0) {
            for (int i = 0; i < nk; i++) {
                if (kids[i].pid != p)
                    continue;
                char why[48];
                if (WIFSIGNALED(status))
                    snprintf(why, sizeof why, "killed by signal %d", WTERMSIG(status));
                else
                    snprintf(why, sizeof why, "exited with %d", WEXITSTATUS(status));
                mark_dead(&kids[i], h, now, why, cfg.backoff_ms);
            }
        }
        for (int i = 0; i < nk; i++) {
            child *c = &kids[i];
            if (c->pid > 0 && h && c->slot >= 0) {
                int64_t beat = h->slot[c->slot].last_beat_ms;
                int64_t since = beat > 0 ? now - beat : now - c->started_ms;
                int limit = beat > 0 ? cfg.dead_ms : START_GRACE_MS;
                if (since > limit) {
                    kill(c->pid, SIGKILL); /* reaped by waitpid on a later pass */
                    seeker_log("%s pid %d: no heartbeat for %lld ms, killed", c->cfg->name,
                               (int)c->pid, (long long)since);
                    c->started_ms = now; /* avoid repeating the kill before the reap */
                }
            }
            if (c->pid == 0 && c->restart_at_ms && now >= c->restart_at_ms)
                start(c, h, now);
        }
        seeker_health_beat(h, self_slot, 0, SH_OK);
        if (now >= next_status) {
            char line[400];
            int o = 0;
            for (int i = 0; i < nk && o < (int)sizeof line - 64; i++) {
                const char *st = kids[i].pid > 0 && h && kids[i].slot >= 0
                                     ? seeker_state_name(h->slot[kids[i].slot].state)
                                     : kids[i].pid > 0 ? "running" : "dead";
                o += snprintf(line + o, sizeof line - (size_t)o, "%s%s %s pid %d r%d",
                              i ? " | " : "", kids[i].cfg->name, st, (int)kids[i].pid,
                              kids[i].restarts);
            }
            seeker_log("%s", line);
            next_status = now + 1000;
        }
        seeker_sleep_ms(50);
    }

    seeker_log("stopping children");
    for (int i = 0; i < nk; i++)
        if (kids[i].pid > 0)
            kill(kids[i].pid, SIGTERM);
    int64_t deadline = seeker_now_ms() + 1000;
    for (;;) {
        int alive = 0;
        for (int i = 0; i < nk; i++) {
            if (kids[i].pid > 0 && waitpid(kids[i].pid, NULL, WNOHANG) == kids[i].pid)
                kids[i].pid = 0;
            alive += kids[i].pid > 0;
        }
        if (!alive)
            break;
        if (seeker_now_ms() > deadline) {
            for (int i = 0; i < nk; i++)
                if (kids[i].pid > 0) {
                    kill(kids[i].pid, SIGKILL);
                    waitpid(kids[i].pid, NULL, 0);
                    kids[i].pid = 0;
                }
            break;
        }
        seeker_sleep_ms(20);
    }
    return 0;
}
