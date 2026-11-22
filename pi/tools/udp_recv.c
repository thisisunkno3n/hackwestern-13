/*
 * udp_recv: prints one line per UDP datagram (lanes task 4, the C fallback for tools/recv.py).
 *
 *   udp_recv [port] [--raw] [--run-for <s>]
 *
 * remote_contacts and tracks get a summary line in tools/recv.py's format; anything else is
 * printed raw. Also prints "NODE Rn resumed" after 1.5 s of silence from a node, like recv.py.
 */

#include "seeker_msgs.h"
#include "seeker_time.h"
#include "seeker_udp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    char id[SEEKER_NODE_LEN];
    int64_t last_ms;
} seen;

int main(int argc, char **argv)
{
    int port = SEEKER_PORT_REMOTE, raw = 0;
    double run_for = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--raw"))
            raw = 1;
        else if (!strcmp(argv[i], "--run-for") && i + 1 < argc)
            run_for = atof(argv[++i]);
        else
            port = atoi(argv[i]);
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    seeker_udp_rx rx;
    if (seeker_udp_listen(&rx, port) != 0) {
        fprintf(stderr, "[udp_recv] cannot bind udp/%d\n", port);
        return 1;
    }
    printf("[recv] listening on udp/%d (%s)\n", port, rx.n == 2 ? "IPv4 + IPv6" : "IPv4 only");

    static char buf[65536];
    static seeker_remote_contacts rc;
    static seeker_tracks tr;
    seen nodes[SEEKER_MAX_NODES];
    int n_nodes = 0;
    int64_t t0 = seeker_now_ms();
    for (;;) {
        if (run_for > 0 && seeker_now_ms() - t0 >= (int64_t)(run_for * 1000))
            break;
        struct sockaddr_storage from;
        int n = seeker_udp_recv(&rx, buf, sizeof buf, 200, &from);
        if (n < 0)
            return 1;
        if (n == 0)
            continue;
        if (raw) {
            printf("[recv] %s\n", buf);
            continue;
        }
        char err[64];
        if (seeker_decode_remote_contacts(buf, n, &rc, err, sizeof err) == 0) {
            int64_t now = seeker_now_ms();
            seen *s = NULL;
            for (int i = 0; i < n_nodes; i++)
                if (!strcmp(nodes[i].id, rc.node_id))
                    s = &nodes[i];
            if (!s && n_nodes < SEEKER_MAX_NODES) {
                s = &nodes[n_nodes++];
                snprintf(s->id, sizeof s->id, "%s", rc.node_id);
                s->last_ms = now;
            }
            if (s && now - s->last_ms > 1500)
                printf("[recv] NODE %s resumed\n", rc.node_id);
            if (s)
                s->last_ms = now;
            printf("[recv] remote_contacts %-3s %d contact%s", rc.node_id, rc.n,
                   rc.n == 1 ? "" : "s");
            if (rc.n > 0) {
                if (rc.c[0].has_range)
                    printf("  bearing %.1f  range %.1f m", rc.c[0].bearing_deg, rc.c[0].range_m);
                else
                    printf("  bearing %.1f  range n/a", rc.c[0].bearing_deg);
            }
            printf("\n");
        } else if (seeker_decode_tracks(buf, n, &tr, err, sizeof err) == 0) {
            printf("[recv] tracks seq %lld  %d track%s", (long long)tr.basis_frame_seq, tr.n,
                   tr.n == 1 ? "" : "s");
            for (int i = 0; i < tr.n_nodes; i++)
                printf("  %s:%s", tr.nodes[i].node_id, tr.nodes[i].state);
            if (tr.n > 0) {
                const seeker_track *k = &tr.tracks[0];
                printf("  [%s %s", k->label, k->colour);
                if (k->has_range)
                    printf(" %.1fm brg %.0f]", k->range_m, k->bearing_deg);
                else
                    printf(" node brg %.0f]", k->node_bearing_deg);
            }
            printf("\n");
        } else {
            char ip[SEEKER_UDP_ADDRSTRLEN];
            seeker_udp_ntop(&from, ip, sizeof ip);
            printf("[recv] %d bytes from %s (%s): %.200s\n", n, ip, err, buf);
        }
    }
    return 0;
}
