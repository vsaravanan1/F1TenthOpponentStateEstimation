#!/usr/bin/env python3
"""
RRT* planner in the Frenet (s, d) frame for generating candidate trajectories
around a set of box obstacles.

Design overview
---------------
* Forward-only growth. The tree only ever extends in the +s direction: a racing
  line never needs to travel backwards along the track, so every steer/rewire is
  constrained to increase s. This keeps the search 1-directional and cheap.

* One dedicated tree per goal. `plan()` grows a *separate* goal-biased RRT* tree
  for each goal it is given. This is the key difference from the previous design,
  which grew a single shared tree per start and chose the goal to bias toward at
  random: with one shared tree, goals that happened never to be sampled near were
  never connected, so parts of the goal set were silently dropped. Growing a tree
  per goal means every goal that is actually reachable (collision-free) gets its
  own fair attempt. The per-call time budget is divided evenly across the goals so
  the total wall-clock cost of a `plan()` call stays bounded.

* Obstacles are axis-aligned boxes in (s, d). Collision checking is a
  Liang-Barsky segment-vs-box overlap test (`_segment_free`).
"""

import math
import time
import numpy as np


class RRTStarPlanner:
    def __init__(self, obstacles=(), max_iter=300, d_limit=0.8, d_clear=0.5,
                 time_budget=0.015, goal_bias=0.2, seed=None,
                 step_target_hops=12, step_min=0.5, step_max=3.0):
        self.set_obstacles(obstacles)
        self.max_iter = max_iter        # cap on nodes grown per goal tree
        self.d_limit = d_limit          # |d| bound when sampling random points
        self.d_clear = d_clear          # lateral half-width of an obstacle box
        self.time_budget = time_budget  # TOTAL wall-clock budget for one plan() call
        self.goal_bias = goal_bias      # probability of sampling the goal directly
        self.rng = np.random.default_rng(seed)

        # Dynamic step sizing (used when plan() is called with step=None). The
        # per-goal extension length is dist(start, goal) / step_target_hops,
        # clipped to [step_min, step_max]. Farther goals therefore get larger
        # steps, so the number of forward extensions needed to span the gap
        # stays roughly constant and a distant goal remains reachable within
        # max_iter nodes.
        self.step_target_hops = step_target_hops
        self.step_min = step_min
        self.step_max = step_max

    def set_obstacles(self, obstacles):
        """obstacles: iterable of (s, d) points, each the centre of a box."""
        self.obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)

    # ------------------------------------------------------------------
    # Collision checking
    # ------------------------------------------------------------------
    def _segment_free(self, p0, p1, s_clear):
        """Return True iff segment p0->p1 misses every obstacle box.

        An obstacle centred at (s, d) is an axis-aligned box spanning
            s in [s - 0.5, s + s_clear]        (longer ahead than behind:
                                                 this reserves following distance)
            d in [d - d_clear, d + d_clear]     (clipped to the track [-1, 1])
        A segment is free of a box when it stays entirely outside it; it is free
        overall only when it is free of *all* boxes.
        """
        for s, d in self.obstacles:
            min_s, max_s = s - 0.5, s + s_clear
            min_d = np.clip(d - self.d_clear, -1.0, 1.0)
            max_d = np.clip(d + self.d_clear, -1.0, 1.0)

            p0_s, p0_d = p0
            p1_s, p1_d = p1

            # (1) Trivial accept for this box: both endpoints strictly on one
            #     side of it, so the whole segment is clear of it.
            if (p0_s < min_s and p1_s < min_s) or \
               (p0_s > max_s and p1_s > max_s) or \
               (p0_d < min_d and p1_d < min_d) or \
               (p0_d > max_d and p1_d > max_d):
                continue

            # (2) Degenerate axis-aligned segment that step (1) did not separate:
            #     the slab test below would divide by zero, so treat it as a
            #     collision (conservative).
            if abs(p1_s - p0_s) < 1e-12 or abs(p1_d - p0_d) < 1e-12:
                return False

            # (3) Liang-Barsky clip of the segment against the box. Track the
            #     parameter interval [t_low, t_high] (t in [0, 1]) that lies
            #     inside the box on both axes; if it is non-empty the segment
            #     enters the box and the segment is blocked.
            t_low, t_high = 0.0, 1.0

            t_a = (min_s - p0_s) / (p1_s - p0_s)
            t_b = (max_s - p0_s) / (p1_s - p0_s)
            t_low = max(t_low, min(t_a, t_b))
            t_high = min(t_high, max(t_a, t_b))

            t_a = (min_d - p0_d) / (p1_d - p0_d)
            t_b = (max_d - p0_d) / (p1_d - p0_d)
            t_low = max(t_low, min(t_a, t_b))
            t_high = min(t_high, max(t_a, t_b))

            if t_low <= t_high:
                return False

        return True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def plan(self, start, goals, s_clear, step=None):
        """Plan one trajectory per goal from a single start.

        A separate goal-biased RRT* tree is grown for each goal so that every
        feasible goal gets a dedicated attempt, instead of sharing one tree in
        which some goals may never be sampled near. The per-call time budget is
        divided evenly across the goals.

        start : (s, d)
        goals : (G, 2) array of (s, d)
        step  : max Euclidean length of a single tree extension. If None
                (default), a dynamic step is chosen per goal from the
                start->goal distance (farther goals -> larger steps) so distant
                goals stay reachable within max_iter nodes. Pass a float to
                force one fixed step for every goal.
        returns: list of length G; entry g is an (N, 2) path start->goal[g],
                 or None if that goal could not be connected.
        """
        start = np.asarray(start, dtype=float)
        goals = np.asarray(goals, dtype=float).reshape(-1, 2)
        n_goals = len(goals)
        if n_goals == 0:
            return []

        per_goal_budget = self.time_budget / n_goals

        paths = []
        for goal in goals:
            goal_step = step if step is not None else self._dynamic_step(start, goal)
            deadline = time.perf_counter() + per_goal_budget
            paths.append(self._plan_single_goal(start, goal, s_clear, goal_step, deadline))
        return paths

    def _dynamic_step(self, start, goal):
        """Extension length scaled to the start->goal distance, so the number
        of forward hops needed to span the gap stays ~step_target_hops no
        matter how far ahead the goal has been projected."""
        dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
        return float(np.clip(dist / self.step_target_hops, self.step_min, self.step_max))

    # ------------------------------------------------------------------
    # One goal, one tree
    # ------------------------------------------------------------------
    def _plan_single_goal(self, start, goal, s_clear, step, deadline):
        """Grow one goal-biased RRT* tree from `start` toward a single `goal`.

        Returns an (N, 2) path start->goal, or None if no collision-free
        connection was found before the deadline / iteration cap.
        """
        s_hi = float(goal[0])

        # Goal lies at or behind the start in +s. Forward-only growth can never
        # reach it, so there is nothing to search.
        if s_hi <= start[0]:
            return None

        # Tree storage. Node i has position S[i], parent parent[i] (-1 at the
        # root) and path cost-from-start cost[i].
        cap = self.max_iter + 1
        S = np.empty((cap, 2))
        parent = np.full(cap, -1, dtype=int)
        cost = np.zeros(cap)
        S[0] = start
        n = 1

        for _ in range(self.max_iter):
            if time.perf_counter() > deadline:
                break

            # --- 1. Sample, biased toward THIS goal ---
            if self.rng.random() < self.goal_bias:
                rnd = goal
            else:
                rnd = np.array([self.rng.uniform(start[0], s_hi),
                                self.rng.uniform(-self.d_limit, self.d_limit)])

            # --- 2. Nearest existing node to the sample ---
            i_near = int(np.argmin(np.sum((S[:n] - rnd) ** 2, axis=1)))
            vec = rnd - S[i_near]
            dist = math.hypot(vec[0], vec[1])
            if dist < 1e-6:
                continue

            # --- 3. Steer toward the sample, forward (+s) only ---
            new = S[i_near] + vec * min(1.0, step / dist)
            if new[0] <= S[i_near, 0] + 1e-3:          # reject non-forward steps
                continue
            if not self._segment_free(S[i_near], new, s_clear):
                continue

            # --- 4. Neighbourhood for choose-parent / rewire ---
            #     Ball radius shrinks as the tree grows (standard RRT* schedule).
            #     Split neighbours by whether they could precede `new` (smaller s,
            #     candidate parents) or follow it (larger s, candidate children).
            r = 1.5 * step * math.sqrt(math.log(n + 1) / (n + 1))
            d2 = np.sum((S[:n] - new) ** 2, axis=1)
            within = d2 <= r * r
            near_parent = np.nonzero(within & (S[:n, 0] < new[0]))[0]
            near_child = np.nonzero(within & (S[:n, 0] > new[0]))[0]

            # --- 5. Choose the cheapest collision-free parent for `new` ---
            #     Start from the nearest node, then see if any neighbour offers a
            #     cheaper collision-free connection (neighbours sorted by cost).
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

            # --- 6. Insert the new node ---
            S[n] = new
            parent[n] = best
            cost[n] = best_cost
            idx = n
            n += 1

            # --- 7. Rewire: let `new` become the parent of any neighbour it can
            #     reach more cheaply (collision-free) ---
            for j in near_child:
                c = best_cost + math.sqrt(d2[j])
                if c < cost[j] and self._segment_free(new, S[j], s_clear):
                    parent[j] = idx
                    cost[j] = c

        # --- 8. Connect the goal to the cheapest collision-free node within
        #     reach, then walk parent pointers back to the start ---
        d2 = np.sum((S[:n] - goal) ** 2, axis=1)
        cand = np.nonzero((d2 <= step * step) & (S[:n, 0] < goal[0]))[0]
        if not len(cand):
            return None

        best = None
        c = cost[cand] + np.sqrt(d2[cand])
        for k in np.argsort(c):
            if self._segment_free(S[cand[k]], goal, s_clear):
                best = int(cand[k])
                break
        if best is None:
            return None

        chain = [goal]
        i = best
        while i != -1:
            chain.append(S[i])
            i = parent[i]
        return np.array(chain[::-1])