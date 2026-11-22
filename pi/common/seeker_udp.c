#include "seeker_udp.h"
#include "seeker_log.h"

#include <errno.h>
#include <netdb.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static int bind_v4(int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons((unsigned short)port);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int bind_v6(int port)
{
    int fd = socket(AF_INET6, SOCK_DGRAM, 0);
    if (fd < 0)
        return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    /* Set explicitly: FreeBSD and io-sock default to 1, Linux to 0, and with 0 Linux would
     * refuse this bind because the IPv4 socket already holds the port. */
    if (setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &one, sizeof one) < 0) {
        close(fd);
        return -1;
    }
    struct sockaddr_in6 a;
    memset(&a, 0, sizeof a);
    a.sin6_family = AF_INET6;
    a.sin6_addr = in6addr_any;
    a.sin6_port = htons((unsigned short)port);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

int seeker_udp_listen(seeker_udp_rx *rx, int port)
{
    rx->n = 0;
    int fd = bind_v4(port);
    if (fd < 0)
        return -1;
    rx->fd[rx->n++] = fd;
    fd = bind_v6(port);
    if (fd >= 0)
        rx->fd[rx->n++] = fd;
    else
        seeker_log("udp/%d: no IPv6 socket (%s), IPv4 only", port, strerror(errno));
    return 0;
}

void seeker_udp_close(seeker_udp_rx *rx)
{
    for (int i = 0; i < rx->n; i++)
        close(rx->fd[i]);
    rx->n = 0;
}

int seeker_udp_recv(seeker_udp_rx *rx, char *buf, int bufsz, int timeout_ms,
                    struct sockaddr_storage *from)
{
    struct pollfd p[2];
    for (int i = 0; i < rx->n; i++) {
        p[i].fd = rx->fd[i];
        p[i].events = POLLIN;
        p[i].revents = 0;
    }
    int r = poll(p, (nfds_t)rx->n, timeout_ms < 0 ? 0 : timeout_ms);
    if (r < 0)
        return errno == EINTR ? 0 : -1;
    for (int i = 0; i < rx->n; i++) {
        if (!(p[i].revents & POLLIN))
            continue;
        socklen_t fl = sizeof(struct sockaddr_storage);
        ssize_t n = recvfrom(p[i].fd, buf, (size_t)(bufsz - 1), 0, (struct sockaddr *)from,
                             from ? &fl : NULL);
        if (n < 0)
            return errno == EINTR || errno == EAGAIN ? 0 : -1;
        buf[n] = '\0';
        return (int)n;
    }
    return 0;
}

const char *seeker_udp_ntop(const struct sockaddr_storage *a, char *out, size_t n)
{
    char host[SEEKER_UDP_ADDRSTRLEN], serv[16];
    socklen_t len = a->ss_family == AF_INET6 ? sizeof(struct sockaddr_in6)
                                              : sizeof(struct sockaddr_in);
    if (getnameinfo((const struct sockaddr *)a, len, host, sizeof host, serv, sizeof serv,
                    NI_NUMERICHOST | NI_NUMERICSERV) != 0)
        snprintf(out, n, "?");
    else if (a->ss_family == AF_INET6)
        snprintf(out, n, "[%s]:%s", host, serv);
    else
        snprintf(out, n, "%s:%s", host, serv);
    return out;
}

int seeker_udp_addr(const char *host, int port, struct sockaddr_in *out)
{
    memset(out, 0, sizeof *out);
    out->sin_family = AF_INET;
    out->sin_port = htons((unsigned short)port);
    if (inet_pton(AF_INET, host, &out->sin_addr) == 1)
        return 0;
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(host, NULL, &hints, &res) != 0 || !res)
        return -1;
    out->sin_addr = ((struct sockaddr_in *)res->ai_addr)->sin_addr;
    freeaddrinfo(res);
    return 0;
}
