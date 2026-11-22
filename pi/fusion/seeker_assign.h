#ifndef SEEKER_ASSIGN_H
#define SEEKER_ASSIGN_H

/*
 * Gated minimum-cost assignment (Hungarian / Kuhn-Munkres, O(n^2 m)) for small problems: rows
 * (existing clusters or tracks) against columns (new rays or fixes). Each row takes at most one
 * column and each column at most one row. Pairs not allowed (gate failed) are never assigned.
 * Among assignments with the most allowed pairs, the total cost is the lowest, so a cheap ghost
 * pair can never force its partners into worse pairs (greedy nearest can).
 */

#define SEEKER_ASSIGN_MAX 80 /* rows and columns each */

/* cost[i * m + j] is the cost of row i with column j; allowed[i * m + j] != 0 if the pair passes
 * its gate. Writes row_col[i] = assigned column or -1. Returns the number of assigned pairs, or -1
 * if n or m is over SEEKER_ASSIGN_MAX. Not reentrant (static work arrays). */
int seeker_assign(const double *cost, const unsigned char *allowed, int n, int m, int *row_col);

#endif
