#!/usr/bin/env python3
import math
import time
import numpy as np


class RRTStarPlanner:
    def __init__(self, obstacles=(), max_iter=300, d_limit=0.8, d_clear = 0.5,
                 time_budget=0.015, goal_bias=0.2, seed=None):
        self.set_obstacles(obstacles)
        self.max_iter = max_iter
        self.d_limit = d_limit
        self.time_budget = time_budget
        self.goal_bias = goal_bias
        self.rng = np.random.default_rng(seed)
        self.d_clear = d_clear

    def set_obstacles(self, obstacles):
        """obstacles: iterable of (s, d) points."""
        self.obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)

    # ------------------------------------------------------------------
    def _segment_free(self, p0, p1, s_clear):
        for i, obstacle in enumerate(self.obstacles):
            s, d = obstacle
            min_s = s - 0.5
            max_s = s + s_clear
            min_d = np.clip(d - self.d_clear, -1.0, 1.0)
            max_d = np.clip(d + self.d_clear, -1.0, 1.0)
            
            p0_s, p0_d = p0
            p1_s, p1_d = p1

            free_conditions = [
                p0_s < min_s and p1_s < min_s,
                p0_s > max_s and p1_s > max_s,
                p0_d < min_d and p1_d < min_d,
                p0_d > max_d and p1_d > max_d
            ]

            if (any(free_conditions)):
                continue

            if abs(p1_s - p0_s) < 1e-12 or abs(p1_d - p0_d) < 1e-12: return False

            # Slab check for box-segment intersection
            t_low = 0.0
            t_high = 1.0

            t_a = (min_s - p0_s)/(p1_s - p0_s)
            t_b = (max_s - p0_s)/(p1_s - p0_s)

            t_low = max(t_low, min(t_a, t_b))
            t_high = min(t_high, max(t_a, t_b))
            
            t_a = (min_d - p0_d)/(p1_d - p0_d)
            t_b = (max_d - p0_d)/(p1_d - p0_d)

            t_low = max(t_low, min(t_a, t_b))
            t_high = min(t_high, max(t_a, t_b))

            if t_low <= t_high:
                return False
            
        return True

    # ------------------------------------------------------------------
    def plan(self, start, goals, s_clear, step=1.0):
        """
        start: (s, d)
        goals: (G, 2) array of (s, d)
        returns: list of length G; each entry is an (N, 2) array from start to
                 goal, or None if that goal could not be connected.
        """
        start = np.asarray(start, dtype=float)
        goals = np.asarray(goals, dtype=float).reshape(-1, 2)

        cap = self.max_iter + 1
        S = np.empty((cap, 2))
        parent = np.full(cap, -1, dtype=int)
        cost = np.zeros(cap)
        S[0] = start
        n = 1

        s_hi = float(goals[:, 0].max())
        if s_hi <= start[0]:
            return [None] * len(goals)

        deadline = time.perf_counter() + self.time_budget

        for _ in range(self.max_iter):
            if time.perf_counter() > deadline:
                break

            # sample (with goal bias)
            if self.rng.random() < self.goal_bias:
                rnd = goals[self.rng.integers(len(goals))]
            else:
                rnd = np.array([self.rng.uniform(start[0], s_hi),
                                self.rng.uniform(-self.d_limit, self.d_limit)])

            # nearest
            i_near = int(np.argmin(np.sum((S[:n] - rnd) ** 2, axis=1)))
            vec = rnd - S[i_near]
            dist = math.hypot(vec[0], vec[1])
            if dist < 1e-6:
                continue

            # steer (forward only)
            new = S[i_near] + vec * min(1.0, step / dist)
            if new[0] <= S[i_near, 0] + 1e-3:
                continue
            if not self._segment_free(S[i_near], new, s_clear):
                continue

            # neighbours
            r = 1.5 * step * math.sqrt(math.log(n + 1) / (n + 1))
            d2 = np.sum((S[:n] - new) ** 2, axis=1)
            within = d2 <= r * r
            near_parent = np.nonzero(within & (S[:n, 0] < new[0]))[0]
            near_child = np.nonzero(within & (S[:n, 0] > new[0]))[0]

            # choose parent (cheapest collision-free)
            best = i_near
            best_cost = cost[i_near] + math.hypot(*(new - S[i_near]))
            if len(near_parent):
                c = cost[near_parent] + np.sqrt(d2[near_parent])
                for k in np.argsort(c):
                    if c[k] >= best_cost:
                        break
                    if self._segment_free(S[near_parent[k]], new, s_clear):
                        best, best_cost = int(near_parent[k]), float(c[k])
                        break

            S[n] = new
            parent[n] = best
            cost[n] = best_cost
            idx = n
            n += 1

            # rewire
            for j in near_child:
                c = best_cost + math.sqrt(d2[j])
                if c < cost[j] and self._segment_free(new, S[j], s_clear):
                    parent[j] = idx
                    cost[j] = c

        # connect every goal
        paths = []
        for g in goals:
            d2 = np.sum((S[:n] - g) ** 2, axis=1)
            cand = np.nonzero((d2 <= step * step) & (S[:n, 0] < g[0]))[0]
            best = None
            if len(cand):
                c = cost[cand] + np.sqrt(d2[cand])
                for k in np.argsort(c):
                    if self._segment_free(S[cand[k]], g, s_clear):
                        best = int(cand[k])
                        break
            if best is None:
                paths.append(None)
                continue
            chain = [g]
            i = best
            while i != -1:
                chain.append(S[i])
                i = parent[i]
            paths.append(np.array(chain[::-1]))
        return paths
