#ifndef SEEKER_UDP_H
#define SEEKER_UDP_H

/* Plain BSD sockets. On QNX 8.0 these live in libsocket (link with -lsocket).
 * Include every header we use directly: glibc pulls sys/socket.h in through
 * netinet/in.h and arpa/inet.h, QNX does not. */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stddef.h>
#include <sys/socket.h>
#include <sys/types.h>

/* Inbound UDP on one port: an IPv4 socket on 0.0.0.0 and, when the stack allows it, an IPv6
 * socket on [::] with IPV6_V6ONLY=1. Two sockets instead of one dual-stack socket because
 * io-sock is FreeBSD's stack, which defaults v6only to 1 and recommends two sockets
 * (FreeBSD inet6(4), "Interaction between IPv4/v6 sockets"). */
typedef struct {
    int fd[2];
    int n;
} seeker_udp_rx;

/* Binds port with SO_REUSEADDR. Fails (-1) only if IPv4 cannot bind; a missing IPv6 socket
 * is logged and the listener runs IPv4 only. */
int seeker_udp_listen(seeker_udp_rx *rx, int port);
void seeker_udp_close(seeker_udp_rx *rx);
/* Waits up to timeout_ms for a datagram on either socket; returns its length (NUL terminated
 * in buf), 0 on timeout, -1 on error. `from` may be NULL. */
int seeker_udp_recv(seeker_udp_rx *rx, char *buf, int bufsz, int timeout_ms,
                    struct sockaddr_storage *from);
/* Formats an IPv4 or IPv6 sender as "1.2.3.4:5" or "[fe80::1%en0]:5" into out (size
 * SEEKER_UDP_ADDRSTRLEN is always enough). Returns out. */
#define SEEKER_UDP_ADDRSTRLEN 96
const char *seeker_udp_ntop(const struct sockaddr_storage *a, char *out, size_t n);

/* Resolves an IPv4 host:port for sendto (tracks to 127.0.0.1). Returns 0 or -1. */
int seeker_udp_addr(const char *host, int port, struct sockaddr_in *out);

#endif
