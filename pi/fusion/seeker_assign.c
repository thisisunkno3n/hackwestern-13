#include "seeker_assign.h"

#define BIG 1e6     /* bonus per allowed pair: the most pairs always wins over a lower cost */
#define COST_MAX 1e5

static double a[SEEKER_ASSIGN_MAX + 1][SEEKER_ASSIGN_MAX + 1];

int seeker_assign(const double *cost, const unsigned char *allowed, int n, int m, int *row_col)
{
    if (n < 0 || m < 0 || n > SEEKER_ASSIGN_MAX || m > SEEKER_ASSIGN_MAX)
        return -1;
    for (int i = 0; i < n; i++)
        row_col[i] = -1;
    if (n == 0 || m == 0)
        return 0;
    /* The solver needs rows <= columns: transpose when there are more rows. */
    int tr = n > m, R = tr ? m : n, C = tr ? n : m;
    for (int i = 1; i <= R; i++)
        for (int j = 1; j <= C; j++) {
            int k = tr ? (j - 1) * m + (i - 1) : (i - 1) * m + (j - 1);
            double c = cost[k] < 0 ? 0 : cost[k] > COST_MAX ? COST_MAX : cost[k];
            a[i][j] = allowed[k] ? c - BIG : 0;
        }

    static double u[SEEKER_ASSIGN_MAX + 1], v[SEEKER_ASSIGN_MAX + 1], minv[SEEKER_ASSIGN_MAX + 1];
    static int p[SEEKER_ASSIGN_MAX + 1], way[SEEKER_ASSIGN_MAX + 1];
    static unsigned char used[SEEKER_ASSIGN_MAX + 1];
    for (int j = 0; j <= C; j++)
        u[j] = v[j] = 0, p[j] = way[j] = 0;
    for (int i = 0; i <= R; i++)
        u[i] = 0;
    for (int i = 1; i <= R; i++) {
        p[0] = i;
        int j0 = 0;
        for (int j = 0; j <= C; j++)
            minv[j] = 1e300, used[j] = 0;
        do {
            used[j0] = 1;
            int i0 = p[j0], j1 = 0;
            double delta = 1e300;
            for (int j = 1; j <= C; j++)
                if (!used[j]) {
                    double cur = a[i0][j] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            for (int j = 0; j <= C; j++)
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    int pairs = 0;
    for (int j = 1; j <= C; j++) {
        if (!p[j])
            continue;
        int row = tr ? j - 1 : p[j] - 1, col = tr ? p[j] - 1 : j - 1;
        if (allowed[row * m + col]) {
            row_col[row] = col;
            pairs++;
        }
    }
    return pairs;
}
