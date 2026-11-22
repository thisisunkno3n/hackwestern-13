/*
 * seeker_supervisor: starts hud_fusion and hud_display, restarts either one whenever it exits or
 * crashes (exponential backoff, reset after a stable run), logs every restart, and on SIGINT
 * (Ctrl+C), SIGTERM (`slay seeker_supervisor`) or SIGHUP stops both children and exits 0.
 * A second signal while stopping skips the grace period and sends SIGKILL.
 *
 *   seeker_supervisor [--bin-dir build] [--config f] [--log-dir logs|-] [--only fusion|display]
 *                     [--fusion path] [--display path] [--fusion-arg a]... [--display-arg a]...
 *                     [--pose ID=x_m,y_m,yaw_deg]... [--poses FILE]
 *                     [--background radar|wearer] [--view-node ID] [--video-port N]
 *                     [--fusion-prio 30r] [--display-prio 50f] [--self-prio 45f]
 *                     [--backoff-min-ms 500] [--backoff-max-ms 8000] [--stable-ms 5000]
 *                     [--hang-ms 0] [--stop-timeout-ms 2000] [--status-s 10]
 *                     [--ignore-hup] [--no-health] [--run-for s] [--check]
 *
 * --check validates the arguments and pose file, prints each child's command line and the pinned
 * poses on stdout, and exits 0 without starting anything.
 *
 * --background, --view-node and --video-port are passed to hud_display unchanged (IF-7 wearer
 * view with phone video from TCP 47106, IF-5.6), after the same up-front validation.
 *
 * --pose and --poses are passed to hud_fusion unchanged (pinned phone poses, see
 * fusion/seeker_pose.h). They are parsed here first, so a typo exits 2 at startup instead of
 * crash-looping hud_fusion, and the startup log lists the poses in effect.
 *
 * Children are spawned with posix_spawnp(). Priorities use POSIX_SPAWN_SETSCHEDULER with the
 * IF-3 values by default (display 50 FIFO, fusion 30 RR). On QNX 8.0 priorities 1..63 need no
 * privilege (procnto -P can change that); on Linux SCHED_FIFO/RR needs root. If the spawn is
 * refused, the child is started at the inherited priority and a warning is logged once. QNX may
 * also report a late scheduler failure as the child exiting with status 127; that is handled the
 * same way.
 *
 * With a log dir (default logs/): each child's stdout and stderr are appended to
 * <dir>/<name>.log, its pid is in <dir>/<name>.pid, this process logs to stderr and to
 * <dir>/seeker_supervisor.log, and restarts are appended to logs/events.jsonl (IF-9).
 *
 * The IF-6 /seeker_health table is recreated at start. Slots for hud_fusion and hud_display are
 * claimed here, and this process beats the "hud_supervisor" slot so the HUD's SUP light works.
 * --hang-ms N (0 = off) also SIGKILLs a child whose heartbeat is older than N ms.
 */

#include "seeker_health.h"
#include "seeker_log.h"
#include "seeker_pose.h"
#include "seeker_sv.h"
#include "seeker_time.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <spawn.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

extern char **environ;

#define MAX_EXTRA_ARGS 24
#define POLL_MS 50
#define HANG_GRACE_MS 3000
#define EXEC_FAIL_WINDOW_MS 1000

typedef struct {
    const char *name; /* process name, health slot, log and pid file name */
    const char *path;
    const char *extra[MAX_EXTRA_ARGS];
    int n_extra;
    int prio, policy;
    int use_prio; /* cleared once the system refuses the priority */
    int spawned_with_prio;
    int enabled;
    pid_t pid;
    int64_t started_ms, exited_ms, restart_at_ms;
    int starts, restarts;
    int hang_killed;
    sv_backoff bo;
    int slot;
} child;

static volatile sig_atomic_t g_stop;
static int g_stopping;

static void on_signal(int sig)
{
    (void)sig;
    if (g_stop < 2)
        g_stop++;
}

static FILE *g_logf;

static void slog(const char *fmt, ...)
{
    char msg[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    time_t sec = ts.tv_sec;
    localtime_r(&sec, &tm);
    char stamp[32];
    snprintf(stamp, sizeof stamp, "%02d:%02d:%02d.%03ld", tm.tm_hour, tm.tm_min, tm.tm_sec,
             ts.tv_nsec / 1000000L);
    fprintf(stderr, "[seeker_supervisor %s] %s\n", stamp, msg);
    if (g_logf) {
        fprintf(g_logf, "[seeker_supervisor %s] %s\n", stamp, msg);
        fflush(g_logf);
    }
}

static const char *g_log_dir = "logs";

static int log_path(char *buf, size_t n, const char *name, const char *ext)
{
    if (!g_log_dir)
        return -1;
    snprintf(buf, n, "%s/%s.%s", g_log_dir, name, ext);
    return 0;
}

static void write_pidfile(const char *name, pid_t pid)
{
    char p[256];
    if (log_path(p, sizeof p, name, "pid") != 0)
        return;
    FILE *f = fopen(p, "w");
    if (!f)
        return;
    fprintf(f, "%d\n", (int)pid);
    fclose(f);
}

static void remove_pidfile(const char *name)
{
    char p[256];
    if (log_path(p, sizeof p, name, "pid") == 0)
        unlink(p);
}

static void child_log_banner(const child *c, int start_no)
{
    char p[256];
    if (log_path(p, sizeof p, c->name, "log") != 0)
        return;
    FILE *f = fopen(p, "a");
    if (!f)
        return;
    fprintf(f, "==== seeker_supervisor: start #%d of %s ====\n", start_no, c->name);
    fclose(f);
}

static int g_ignore_hup;

static int spawn_child(child *c, const char *config, pid_t *out, int with_prio)
{
    char *argv[MAX_EXTRA_ARGS + 4];
    int a = 0;
    argv[a++] = (char *)c->path;
    for (int i = 0; i < c->n_extra; i++)
        argv[a++] = (char *)c->extra[i];
    if (config) {
        argv[a++] = "--config";
        argv[a++] = (char *)config;
    }
    argv[a] = NULL;

    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    char lp[256];
    if (log_path(lp, sizeof lp, c->name, "log") == 0) {
        posix_spawn_file_actions_addopen(&fa, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
        posix_spawn_file_actions_addopen(&fa, STDOUT_FILENO, lp, O_WRONLY | O_CREAT | O_APPEND,
                                         0644);
        posix_spawn_file_actions_adddup2(&fa, STDOUT_FILENO, STDERR_FILENO);
    }

    posix_spawnattr_t at;
    posix_spawnattr_init(&at);
    short flags = POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF;
    sigset_t none, dflt;
    sigemptyset(&none);
    posix_spawnattr_setsigmask(&at, &none);
    /* Our handlers become SIG_DFL across exec anyway; this also undoes SIG_IGN inherited from a
     * non-interactive shell's `&`, so `slay` and Ctrl+C reach the children. */
    sigemptyset(&dflt);
    sigaddset(&dflt, SIGINT);
    sigaddset(&dflt, SIGTERM);
    sigaddset(&dflt, SIGQUIT);
    sigaddset(&dflt, SIGPIPE);
    if (!g_ignore_hup)
        sigaddset(&dflt, SIGHUP);
    posix_spawnattr_setsigdefault(&at, &dflt);
    if (with_prio) {
        struct sched_param sp;
        memset(&sp, 0, sizeof sp);
        sp.sched_priority = c->prio;
        posix_spawnattr_setschedpolicy(&at, c->policy);
        posix_spawnattr_setschedparam(&at, &sp);
        flags |= POSIX_SPAWN_SETSCHEDULER;
    }
    posix_spawnattr_setflags(&at, flags);

    int rc = posix_spawnp(out, c->path, &fa, &at, argv, environ);
    posix_spawnattr_destroy(&at);
    posix_spawn_file_actions_destroy(&fa);
    return rc;
}

static void schedule_restart(child *c, int64_t now, int64_t uptime_ms)
{
    int delay = sv_backoff_next(&c->bo, uptime_ms);
    c->restarts++;
    c->restart_at_ms = now + delay;
}

static void start_child(child *c, const char *config, seeker_health_table *h, int64_t now)
{
    char ps[16];
    pid_t pid = -1;
    int with_prio = c->use_prio && c->prio > 0;
    child_log_banner(c, c->starts + 1);
    int rc = spawn_child(c, config, &pid, with_prio);
    if (rc != 0 && with_prio && (rc == EPERM || rc == EINVAL || rc == ENOTSUP)) {
        slog("%s: spawn at priority %s refused (%s); using the inherited priority from now on",
             c->name, sv_prio_str(c->prio, c->policy, ps, sizeof ps), strerror(rc));
        c->use_prio = 0;
        with_prio = 0;
        rc = spawn_child(c, config, &pid, 0);
    }
    if (rc != 0) {
        schedule_restart(c, now, 0);
        slog("%s: spawn %s failed: %s; retry #%d in %lld ms", c->name, c->path, strerror(rc),
             c->restarts, (long long)(c->restart_at_ms - now));
        return;
    }
    c->pid = pid;
    c->started_ms = now;
    c->restart_at_ms = 0;
    c->hang_killed = 0;
    c->spawned_with_prio = with_prio;
    c->starts++;
    write_pidfile(c->name, pid);
    if (h && c->slot >= 0) {
        h->slot[c->slot].pid = (int32_t)pid;
        h->slot[c->slot].last_beat_ms = 0;
        h->slot[c->slot].state = SH_STARTING;
        h->slot[c->slot].restarts = c->restarts;
    }
    char down[48] = "";
    if (c->exited_ms > 0)
        snprintf(down, sizeof down, ", down %lld ms", (long long)(now - c->exited_ms));
    slog("started %s pid %d prio %s (start #%d, restarts %d%s)", c->name, (int)pid,
         sv_prio_str(with_prio ? c->prio : 0, c->policy, ps, sizeof ps), c->starts, c->restarts,
         down);
}

static void child_exited(child *c, int status, seeker_health_table *h, int64_t now)
{
    char why[64], ps[16];
    sv_describe_status(status, why, sizeof why);
    int64_t up = now - c->started_ms;
    if (WIFEXITED(status) && WEXITSTATUS(status) == 127 && c->spawned_with_prio &&
        up < EXEC_FAIL_WINDOW_MS) {
        slog("%s: exit status 127 right after a spawn at priority %s; using the inherited "
             "priority from now on", c->name, sv_prio_str(c->prio, c->policy, ps, sizeof ps));
        c->use_prio = 0;
    }
    remove_pidfile(c->name);
    if (h && c->slot >= 0)
        h->slot[c->slot].state = SH_DEAD;
    pid_t old = c->pid;
    c->pid = 0;
    c->exited_ms = now;
    if (g_stop || g_stopping) {
        slog("%s pid %d %s", c->name, (int)old, why);
        return;
    }
    schedule_restart(c, now, up);
    int64_t delay = c->restart_at_ms - now;
    slog("%s pid %d %s after %.1f s; restart #%d in %lld ms", c->name, (int)old, why,
         (double)up / 1000.0, c->restarts, (long long)delay);
    char ev[256];
    snprintf(ev, sizeof ev,
             "\"event\":\"restart\",\"process\":\"%s\",\"pid\":%d,\"reason\":\"%s\","
             "\"uptime_ms\":%lld,\"delay_ms\":%lld,\"restarts\":%d",
             c->name, (int)old, why, (long long)up, (long long)delay, c->restarts);
    seeker_event(ev);
    if (h && c->slot >= 0)
        h->slot[c->slot].restarts = c->restarts;
}

static void check_hang(child *c, const seeker_health_table *h, int hang_ms, int64_t now)
{
    if (!h || c->slot < 0 || hang_ms <= 0 || c->pid <= 0 || c->hang_killed)
        return;
    int64_t beat = h->slot[c->slot].last_beat_ms;
    int64_t since = beat > 0 ? now - beat : now - c->started_ms;
    int limit = beat > 0 ? hang_ms : (hang_ms > HANG_GRACE_MS ? hang_ms : HANG_GRACE_MS);
    if (since <= limit)
        return;
    slog("%s pid %d: no heartbeat for %lld ms; sending SIGKILL", c->name, (int)c->pid,
         (long long)since);
    kill(c->pid, SIGKILL);
    c->hang_killed = 1;
}

static void stop_children(child *kids, int nk, int timeout_ms, seeker_health_table *h)
{
    int alive = 0;
    for (int i = 0; i < nk; i++)
        if (kids[i].pid > 0) {
            kill(kids[i].pid, SIGTERM);
            alive++;
        }
    if (alive)
        slog("stopping: SIGTERM sent to %d child%s", alive, alive == 1 ? "" : "ren");
    int64_t deadline = seeker_now_ms() + timeout_ms;
    for (;;) {
        alive = 0;
        for (int i = 0; i < nk; i++) {
            child *c = &kids[i];
            if (c->pid <= 0)
                continue;
            int st = 0;
            pid_t r = waitpid(c->pid, &st, WNOHANG);
            if (r == c->pid)
                child_exited(c, st, h, seeker_now_ms());
            else if (r < 0 && errno == ECHILD) {
                remove_pidfile(c->name);
                c->pid = 0;
            } else
                alive++;
        }
        if (!alive)
            return;
        if (g_stop >= 2 || seeker_now_ms() > deadline)
            break;
        struct timespec ts = { 0, 20 * 1000000L };
        nanosleep(&ts, NULL);
    }
    for (int i = 0; i < nk; i++) {
        child *c = &kids[i];
        if (c->pid <= 0)
            continue;
        slog("%s pid %d still running; sending SIGKILL", c->name, (int)c->pid);
        kill(c->pid, SIGKILL);
        int st = 0;
        while (waitpid(c->pid, &st, 0) < 0 && errno == EINTR) {
        }
        child_exited(c, st, h, seeker_now_ms());
    }
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "usage: %s [--bin-dir build] [--config f] [--log-dir logs|-] [--only fusion|display]\n"
            "       [--fusion path] [--display path] [--fusion-arg a]... [--display-arg a]...\n"
            "       [--pose ID=x_m,y_m,yaw_deg]... [--poses FILE]\n"
            "       [--background radar|wearer] [--view-node ID] [--video-port N]\n"
            "       [--fusion-prio 30r] [--display-prio 50f] [--self-prio 45f]\n"
            "       [--backoff-min-ms 500] [--backoff-max-ms 8000] [--stable-ms 5000]\n"
            "       [--hang-ms 0] [--stop-timeout-ms 2000] [--status-s 10]\n"
            "       [--ignore-hup] [--no-health] [--run-for s] [--check]\n"
            "priority spec: <prio>[f|r|o] as for QNX `on -p`; 0 = inherit\n",
            prog);
}

int main(int argc, char **argv)
{
    const char *bin_dir = "build", *config = NULL, *only = NULL;
    const char *fusion_path = NULL, *display_path = NULL;
    const char *fusion_prio = "30r", *display_prio = "50f", *self_prio = "45f";
    int bo_min = 500, bo_max = 8000, stable_ms = 5000, hang_ms = 0, stop_timeout = 2000;
    int status_s = 10, use_health = 1, check_only = 0;
    double run_for = 0;
    const char *pose_file = NULL, *pose_args[SEEKER_MAX_NODES];
    const char *background = NULL, *view_node = NULL, *video_port = NULL;
    int n_pose_args = 0;
    static child kids[2];
    memset(kids, 0, sizeof kids);
    kids[0].name = "hud_fusion";
    kids[1].name = "hud_display";

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        const char *v = i + 1 < argc ? argv[i + 1] : NULL;
#define TAKE(dst) (v ? ((dst) = v, i++, 1) : 0)
        if (!strcmp(a, "--bin-dir") && TAKE(bin_dir)) {
        } else if (!strcmp(a, "--config") && TAKE(config)) {
        } else if (!strcmp(a, "--log-dir") && TAKE(g_log_dir)) {
        } else if (!strcmp(a, "--only") && TAKE(only)) {
        } else if (!strcmp(a, "--fusion") && TAKE(fusion_path)) {
        } else if (!strcmp(a, "--display") && TAKE(display_path)) {
        } else if (!strcmp(a, "--fusion-prio") && TAKE(fusion_prio)) {
        } else if (!strcmp(a, "--display-prio") && TAKE(display_prio)) {
        } else if (!strcmp(a, "--self-prio") && TAKE(self_prio)) {
        } else if (!strcmp(a, "--poses") && TAKE(pose_file)) {
        } else if (!strcmp(a, "--background") && TAKE(background)) {
        } else if (!strcmp(a, "--view-node") && TAKE(view_node)) {
        } else if (!strcmp(a, "--video-port") && TAKE(video_port)) {
        } else if (!strcmp(a, "--pose") && v) {
            if (n_pose_args >= SEEKER_MAX_NODES) {
                fprintf(stderr, "too many --pose values (max %d)\n", SEEKER_MAX_NODES);
                return 2;
            }
            pose_args[n_pose_args++] = v;
            i++;
        } else if ((!strcmp(a, "--fusion-arg") || !strcmp(a, "--display-arg")) && v) {
            child *c = &kids[a[2] == 'f' ? 0 : 1];
            if (c->n_extra >= MAX_EXTRA_ARGS) {
                fprintf(stderr, "too many %s values (max %d)\n", a, MAX_EXTRA_ARGS);
                return 2;
            }
            c->extra[c->n_extra++] = v;
            i++;
        } else if (!strcmp(a, "--backoff-min-ms") && v) {
            bo_min = atoi(argv[++i]);
        } else if (!strcmp(a, "--backoff-max-ms") && v) {
            bo_max = atoi(argv[++i]);
        } else if (!strcmp(a, "--stable-ms") && v) {
            stable_ms = atoi(argv[++i]);
        } else if (!strcmp(a, "--hang-ms") && v) {
            hang_ms = atoi(argv[++i]);
        } else if (!strcmp(a, "--stop-timeout-ms") && v) {
            stop_timeout = atoi(argv[++i]);
        } else if (!strcmp(a, "--status-s") && v) {
            status_s = atoi(argv[++i]);
        } else if (!strcmp(a, "--run-for") && v) {
            run_for = atof(argv[++i]);
        } else if (!strcmp(a, "--check")) {
            check_only = 1;
        } else if (!strcmp(a, "--ignore-hup")) {
            g_ignore_hup = 1;
        } else if (!strcmp(a, "--no-health")) {
            use_health = 0;
        } else {
            usage(argv[0]);
            return 2;
        }
#undef TAKE
    }
    if (g_log_dir && !strcmp(g_log_dir, "-"))
        g_log_dir = NULL;
    if (bo_min <= 0 || bo_max < bo_min || stable_ms < 0 || stop_timeout < 0) {
        fprintf(stderr, "bad backoff or timeout values\n");
        return 2;
    }

    static seeker_pose_table poses;
    char perr[160];
    if (pose_file && seeker_pose_load_file(&poses, pose_file, perr, sizeof perr) < 0) {
        fprintf(stderr, "--poses: %s\n", perr);
        return 2;
    }
    for (int i = 0; i < n_pose_args; i++) {
        char id[SEEKER_NODE_LEN];
        seeker_pose p;
        if (seeker_pose_parse(pose_args[i], id, &p, perr, sizeof perr) != 0 ||
            seeker_pose_table_set(&poses, id, &p, "cli") != 0) {
            fprintf(stderr, "--pose %s: %s\n", pose_args[i], perr);
            return 2;
        }
    }
    if (background && strcmp(background, "radar") && strcmp(background, "wearer")) {
        fprintf(stderr, "--background %s: want radar or wearer\n", background);
        return 2;
    }
    if (view_node && !sv_valid_node_id(view_node, SEEKER_NODE_LEN - 1)) {
        fprintf(stderr, "--view-node %s: want 1 to %d characters of A-Z a-z 0-9 _ -\n", view_node,
                SEEKER_NODE_LEN - 1);
        return 2;
    }
    if (video_port && sv_parse_port(video_port) < 0) {
        fprintf(stderr, "--video-port %s: want 1..65535\n", video_port);
        return 2;
    }
    const char *video_flags[3][2] = { { "--background", background },
                                      { "--view-node", view_node },
                                      { "--video-port", video_port } };
    for (int i = 0; i < 3; i++) {
        if (!video_flags[i][1])
            continue;
        if (kids[1].n_extra + 2 > MAX_EXTRA_ARGS) {
            fprintf(stderr, "too many hud_display arguments (max %d)\n", MAX_EXTRA_ARGS);
            return 2;
        }
        kids[1].extra[kids[1].n_extra++] = video_flags[i][0];
        kids[1].extra[kids[1].n_extra++] = video_flags[i][1];
    }
    if (kids[0].n_extra + 2 + 2 * n_pose_args > MAX_EXTRA_ARGS) {
        fprintf(stderr, "too many hud_fusion arguments (max %d)\n", MAX_EXTRA_ARGS);
        return 2;
    }
    if (pose_file) {
        kids[0].extra[kids[0].n_extra++] = "--poses";
        kids[0].extra[kids[0].n_extra++] = pose_file;
    }
    for (int i = 0; i < n_pose_args; i++) {
        kids[0].extra[kids[0].n_extra++] = "--pose";
        kids[0].extra[kids[0].n_extra++] = pose_args[i];
    }

    static char fpath[256], dpath[256];
    snprintf(fpath, sizeof fpath, "%s/hud_fusion", bin_dir);
    snprintf(dpath, sizeof dpath, "%s/hud_display", bin_dir);
    kids[0].path = fusion_path ? fusion_path : fpath;
    kids[1].path = display_path ? display_path : dpath;
    const char *prio_spec[2] = { fusion_prio, display_prio };
    for (int i = 0; i < 2; i++) {
        child *c = &kids[i];
        if (sv_parse_prio(prio_spec[i], &c->prio, &c->policy) != 0) {
            fprintf(stderr, "bad priority '%s' for %s (want e.g. 50f, 30r)\n", prio_spec[i],
                    c->name);
            return 2;
        }
        c->use_prio = 1;
        c->slot = -1;
        sv_backoff_init(&c->bo, bo_min, bo_max, stable_ms);
        const char *shortname = c->name + 4;
        c->enabled = !only || !strcmp(only, c->name) || !strcmp(only, shortname);
        if (c->enabled && strchr(c->path, '/') && access(c->path, X_OK) != 0) {
            fprintf(stderr, "%s: %s is not executable (%s); build first or pass --bin-dir\n",
                    c->name, c->path, strerror(errno));
            return 2;
        }
    }
    if (!kids[0].enabled && !kids[1].enabled) {
        fprintf(stderr, "--only %s matches neither fusion nor display\n", only);
        return 2;
    }
    if (check_only) {
        for (int i = 0; i < 2; i++) {
            const child *c = &kids[i];
            if (!c->enabled)
                continue;
            printf("%s:", c->name);
            printf(" %s", c->path);
            for (int k = 0; k < c->n_extra; k++)
                printf(" %s", c->extra[k]);
            if (config)
                printf(" --config %s", config);
            printf("\n");
        }
        for (int i = 0; i < poses.n; i++)
            printf("pose %s %.2f,%.2f,%.1f (%s)\n", poses.e[i].id, poses.e[i].pose.x_m,
                   poses.e[i].pose.y_m, poses.e[i].pose.yaw_deg,
                   strcmp(poses.e[i].src, "file") ? "--pose" : pose_file);
        return 0;
    }

    seeker_log_init("seeker_supervisor");
    if (g_log_dir) {
        mkdir(g_log_dir, 0755);
        char p[256];
        log_path(p, sizeof p, "seeker_supervisor", "log");
        g_logf = fopen(p, "a");
        write_pidfile("seeker_supervisor", getpid());
    }
    mkdir("logs", 0755); /* seeker_event() writes logs/events.jsonl */

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    struct sigaction old_hup;
    sigaction(SIGHUP, NULL, &old_hup);
    if (old_hup.sa_handler == SIG_IGN) /* started under nohup: stay that way */
        g_ignore_hup = 1;
    if (g_ignore_hup) {
        struct sigaction ign;
        memset(&ign, 0, sizeof ign);
        ign.sa_handler = SIG_IGN;
        sigaction(SIGHUP, &ign, NULL);
    } else {
        sigaction(SIGHUP, &sa, NULL);
    }
    struct sigaction ign_pipe;
    memset(&ign_pipe, 0, sizeof ign_pipe);
    ign_pipe.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &ign_pipe, NULL);

    char ps[16];
    int sprio = 0, spol = SCHED_RR;
    if (sv_parse_prio(self_prio, &sprio, &spol) != 0) {
        fprintf(stderr, "bad --self-prio '%s'\n", self_prio);
        return 2;
    }
    if (sprio > 0) {
        struct sched_param sp;
        memset(&sp, 0, sizeof sp);
        sp.sched_priority = sprio;
        int rc = pthread_setschedparam(pthread_self(), spol, &sp);
        if (rc != 0)
            slog("own priority %s not set (%s); running at the inherited priority",
                 sv_prio_str(sprio, spol, ps, sizeof ps), strerror(rc));
    }

    seeker_health_table *h = use_health ? seeker_health_open(1) : NULL;
    if (use_health && !h)
        slog("shm_open %s failed (%s); no SUP light, --hang-ms disabled", SEEKER_HEALTH_NAME,
             strerror(errno));
    int self_slot = seeker_health_slot_for(h, "hud_supervisor");
    for (int i = 0; i < 2; i++)
        if (kids[i].enabled)
            kids[i].slot = seeker_health_slot_for(h, kids[i].name);

    slog("pid %d: backoff %d..%d ms, stable after %d ms, hang watchdog %s, log dir %s",
         (int)getpid(), bo_min, bo_max, stable_ms, hang_ms > 0 ? "on" : "off",
         g_log_dir ? g_log_dir : "(none)");
    if (kids[1].enabled)
        slog("hud_display background %s, view node %s, video tcp/%s", background ? background : "(config)",
             view_node ? view_node : "(config)", video_port ? video_port : "47106 (default)");
    if (kids[0].enabled) {
        if (poses.n == 0)
            slog("hud_fusion poses: none pinned (message poses, then remote.nodes)");
        for (int i = 0; i < poses.n; i++) {
            const seeker_pose_entry *e = &poses.e[i];
            slog("hud_fusion pose %s %.2f,%.2f,%.1f pinned from %s", e->id, e->pose.x_m,
                 e->pose.y_m, e->pose.yaw_deg, strcmp(e->src, "file") ? "--pose" : pose_file);
        }
    }
    int64_t now = seeker_now_ms();
    for (int i = 0; i < 2; i++)
        if (kids[i].enabled)
            start_child(&kids[i], config, h, now);

    const int64_t t_start = now;
    int64_t next_status = now + (int64_t)status_s * 1000;
    while (!g_stop) {
        now = seeker_now_ms();
        if (run_for > 0 && now - t_start >= (int64_t)(run_for * 1000))
            break;
        int st;
        pid_t p;
        while ((p = waitpid(-1, &st, WNOHANG)) > 0) {
            for (int i = 0; i < 2; i++)
                if (kids[i].pid == p)
                    child_exited(&kids[i], st, h, now);
        }
        for (int i = 0; i < 2 && !g_stop; i++) {
            child *c = &kids[i];
            if (!c->enabled)
                continue;
            check_hang(c, h, hang_ms, now);
            if (c->pid == 0 && c->restart_at_ms && now >= c->restart_at_ms)
                start_child(c, config, h, now);
        }
        seeker_health_beat(h, self_slot, 0, SH_OK);
        if (status_s > 0 && now >= next_status) {
            char line[256];
            int o = 0;
            for (int i = 0; i < 2; i++) {
                const child *c = &kids[i];
                if (!c->enabled)
                    continue;
                if (c->pid > 0)
                    o += snprintf(line + o, sizeof line - (size_t)o, "%s%s pid %d up %lld s r%d",
                                  o ? " | " : "", c->name, (int)c->pid,
                                  (long long)((now - c->started_ms) / 1000), c->restarts);
                else
                    o += snprintf(line + o, sizeof line - (size_t)o, "%s%s down r%d",
                                  o ? " | " : "", c->name, c->restarts);
            }
            slog("status: %s", line);
            next_status = now + (int64_t)status_s * 1000;
        }
        struct timespec ts = { 0, POLL_MS * 1000000L };
        nanosleep(&ts, NULL); /* EINTR is fine: the loop re-checks g_stop */
    }

    slog("%s; stopping children", g_stop ? "signal received" : "--run-for elapsed");
    g_stopping = 1;
    stop_children(kids, 2, stop_timeout, h);
    if (h && self_slot >= 0)
        h->slot[self_slot].state = SH_DEAD;
    slog("stopped cleanly (restarts: hud_fusion %d, hud_display %d)", kids[0].restarts,
         kids[1].restarts);
    if (g_log_dir)
        remove_pidfile("seeker_supervisor");
    if (g_logf)
        fclose(g_logf);
    return 0;
}
