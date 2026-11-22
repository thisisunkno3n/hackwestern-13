/* Host unit tests for seeker_supervisor's backoff, priority specs and exit-status text. */

#include "seeker_sv.h"

#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

static int g_fail, g_pass;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (cond) {                                                                                \
            g_pass++;                                                                              \
        } else {                                                                                   \
            g_fail++;                                                                              \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                        \
        }                                                                                          \
    } while (0)

static void test_backoff(void)
{
    sv_backoff b;
    sv_backoff_init(&b, 500, 8000, 5000);
    /* crash loop: 500, 1000, 2000, 4000, 8000, then capped */
    CHECK(sv_backoff_next(&b, 10) == 500);
    CHECK(sv_backoff_next(&b, 10) == 1000);
    CHECK(sv_backoff_next(&b, 10) == 2000);
    CHECK(sv_backoff_next(&b, 10) == 4000);
    CHECK(sv_backoff_next(&b, 10) == 8000);
    CHECK(sv_backoff_next(&b, 10) == 8000);
    for (int i = 0; i < 100; i++)
        sv_backoff_next(&b, 0); /* no overflow after many failures */
    CHECK(sv_backoff_next(&b, 0) == 8000);
    /* a stable run resets to the minimum */
    CHECK(sv_backoff_next(&b, 5000) == 500);
    CHECK(sv_backoff_next(&b, 4999) == 1000);
    CHECK(sv_backoff_next(&b, 60000) == 500);

    /* max below min is clamped up to min; min <= 0 becomes 1 */
    sv_backoff_init(&b, 300, 100, 1000);
    CHECK(sv_backoff_next(&b, 0) == 300);
    CHECK(sv_backoff_next(&b, 0) == 300);
    sv_backoff_init(&b, 0, 4, 1000);
    CHECK(sv_backoff_next(&b, 0) == 1);
    CHECK(sv_backoff_next(&b, 0) == 2);
    CHECK(sv_backoff_next(&b, 0) == 4);
    CHECK(sv_backoff_next(&b, 0) == 4);

    /* non-power-of-two cap */
    sv_backoff_init(&b, 300, 1000, 1000);
    CHECK(sv_backoff_next(&b, 0) == 300);
    CHECK(sv_backoff_next(&b, 0) == 600);
    CHECK(sv_backoff_next(&b, 0) == 1000);
}

static void test_prio(void)
{
    int p, pol;
    char buf[16];
    CHECK(sv_parse_prio("50f", &p, &pol) == 0 && p == 50 && pol == SCHED_FIFO);
    CHECK(sv_parse_prio("30r", &p, &pol) == 0 && p == 30 && pol == SCHED_RR);
    CHECK(sv_parse_prio("10o", &p, &pol) == 0 && p == 10 && pol == SCHED_OTHER);
    CHECK(sv_parse_prio("45", &p, &pol) == 0 && p == 45 && pol == SCHED_RR);
    CHECK(sv_parse_prio("0", &p, &pol) == 0 && p == 0);
    CHECK(sv_parse_prio("", &p, &pol) == 0 && p == 0);
    CHECK(sv_parse_prio(NULL, &p, &pol) == 0 && p == 0);
    CHECK(sv_parse_prio("255f", &p, &pol) == 0 && p == 255);
    CHECK(sv_parse_prio("256", &p, &pol) < 0);
    CHECK(sv_parse_prio("f50", &p, &pol) < 0);
    CHECK(sv_parse_prio("50x", &p, &pol) < 0);
    CHECK(sv_parse_prio("50ff", &p, &pol) < 0);
    CHECK(sv_parse_prio("-5", &p, &pol) < 0);
    CHECK(sv_parse_prio("99999999999", &p, &pol) < 0);
    CHECK(!strcmp(sv_prio_str(50, SCHED_FIFO, buf, sizeof buf), "50f"));
    CHECK(!strcmp(sv_prio_str(30, SCHED_RR, buf, sizeof buf), "30r"));
    CHECK(!strcmp(sv_prio_str(10, SCHED_OTHER, buf, sizeof buf), "10o"));
    CHECK(!strcmp(sv_prio_str(0, SCHED_FIFO, buf, sizeof buf), "inherit"));
}

/* Real wait statuses from real children, so the macros are exercised as on the target. */
static int status_of(int how)
{
    pid_t pid = fork();
    if (pid == 0) {
        if (how >= 0)
            _exit(how);
        signal(-how, SIG_DFL);
        kill(getpid(), -how);
        _exit(99);
    }
    int st = 0;
    waitpid(pid, &st, 0);
    return st;
}

static void test_status(void)
{
    char buf[64];
    CHECK(!strcmp(sv_describe_status(status_of(0), buf, sizeof buf), "exited with status 0"));
    CHECK(!strcmp(sv_describe_status(status_of(3), buf, sizeof buf), "exited with status 3"));
    CHECK(!strcmp(sv_describe_status(status_of(-SIGTERM), buf, sizeof buf),
                  "killed by SIGTERM (15)"));
    CHECK(!strcmp(sv_describe_status(status_of(-SIGKILL), buf, sizeof buf),
                  "killed by SIGKILL (9)"));
    char want[64];
    /* SIGUSR1, not SIGSEGV: no core file from QNX dumper. Its number differs between systems. */
    snprintf(want, sizeof want, "killed by SIGUSR1 (%d)", SIGUSR1);
    CHECK(!strcmp(sv_describe_status(status_of(-SIGUSR1), buf, sizeof buf), want));
    CHECK(!strcmp(sv_signal_name(SIGINT, buf, sizeof buf), "SIGINT"));
    CHECK(!strcmp(sv_signal_name(SIGHUP, buf, sizeof buf), "SIGHUP"));
    CHECK(!strcmp(sv_signal_name(1000, buf, sizeof buf), "signal 1000"));
}

static void test_video_args(void)
{
    CHECK(sv_parse_port("47106") == 47106);
    CHECK(sv_parse_port("1") == 1);
    CHECK(sv_parse_port("65535") == 65535);
    CHECK(sv_parse_port("0") < 0);
    CHECK(sv_parse_port("65536") < 0);
    CHECK(sv_parse_port("99999999999") < 0);
    CHECK(sv_parse_port("") < 0);
    CHECK(sv_parse_port(NULL) < 0);
    CHECK(sv_parse_port("-1") < 0);
    CHECK(sv_parse_port("47106x") < 0);
    CHECK(sv_valid_node_id("R1", 7));
    CHECK(sv_valid_node_id("node_2-a", 8));
    CHECK(sv_valid_node_id("ABCDEFG", 7));
    CHECK(!sv_valid_node_id("ABCDEFGH", 7));
    CHECK(!sv_valid_node_id("", 7));
    CHECK(!sv_valid_node_id(NULL, 7));
    CHECK(!sv_valid_node_id("R 1", 7));
    CHECK(!sv_valid_node_id("R1;", 7));
}

int main(void)
{
    test_backoff();
    test_prio();
    test_status();
    test_video_args();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
