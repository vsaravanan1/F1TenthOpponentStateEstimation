#!/usr/bin/env python3
"""
diverse_opponent_trajectories_node.py

Consumes ALL predicted opponent splines (the 5 /predicted_opponent_splines_i
MarkerArrays, each with up to 5 LINE_STRIP splines) and emits up to 3 edge-diverse
opponent trajectories on /diverse_opponent_trajectories.

How it works
------------
1. Collect every spline currently published (0..25 of them -- RRT* does not always
   return 5/5 per starting point, so the real count varies).
2. Fuse them into ONE Roadmap: each point is quantized to a grid cell, so splines
   that run through the same region share nodes and edges. That shared structure is
   what makes edge-overlap diversity meaningful (two near-identical splines share
   most edges -> one is rejected as non-diverse).
     * a virtual START node is wired (cost 0) to the first cell of every spline, so
       a single search can begin from any opponent start;
     * the GOAL SET is the set of (coarsely quantized) spline END cells -- it is
       determined entirely by the splines that arrived, so it has AT MOST 5 goals
       and fewer when some are missing.
3. diverse_roadmap_search (Yen's k-shortest-paths + an edge-overlap diversity
   filter) returns up to k=3 diverse START->(any goal) paths, which are mapped back
   to xy polylines and published.

The graph / dijkstra / diverse-search below mirror the project's `.graph` +
`diverse_search` API, but dijkstra takes a GOAL SET instead of a single goal. They
are kept in this file so the node is self-contained; move them into graph.py if you
prefer.
"""

import heapq
import itertools
from collections import defaultdict
from typing import Dict, List, Sequence, Set, Tuple, Optional

import numpy as np
from scipy.interpolate import splprep, splev

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ─────────────────────────────────────────────────────────────────────────────
# Roadmap  (self-contained; mirrors the project's Roadmap API)
# ─────────────────────────────────────────────────────────────────────────────
class Roadmap:
    """Directed weighted graph. Nodes are hashable ids:
         - START               : the virtual super-source (a string)
         - (ix, iy)            : a fused interior/path cell (length-2 tuple)
         - ("G", gx, gy)       : a goal cell (length-3 tuple)
    Only interior-cell edges count toward path diversity (START/goal link edges
    are zero-cost connectors and are excluded from path_edges)."""

    START = "__START__"

    def __init__(self) -> None:
        self.adj: Dict[object, List[Tuple[object, float]]] = defaultdict(list)
        self.edge_cost: Dict[Tuple[object, object], float] = {}
        self.points: Dict[object, np.ndarray] = {}      # node -> representative xy

    def add_point(self, node: object, xy) -> None:
        if node not in self.points:
            self.points[node] = np.asarray(xy, dtype=float)

    def add_edge(self, u: object, v: object, w: float) -> None:
        """Add directed edge u->v, keeping the cheapest if it already exists."""
        if u == v:
            return
        prev = self.edge_cost.get((u, v))
        if prev is None:
            self.adj[u].append((v, w))
            self.edge_cost[(u, v)] = w
        elif w < prev:
            self.edge_cost[(u, v)] = w

    def neighbors(self, u: object) -> List[Tuple[object, float]]:
        return self.adj.get(u, [])

    def path_cost(self, path: Sequence[object]) -> float:
        return sum(self.edge_cost[(path[i], path[i + 1])] for i in range(len(path) - 1))

    @staticmethod
    def _is_interior(node: object) -> bool:
        return isinstance(node, tuple) and len(node) == 2

    def path_edges(self, path: Sequence[object]) -> Set[Tuple[object, object]]:
        """Edges that count for diversity: interior-cell to interior-cell only."""
        edges: Set[Tuple[object, object]] = set()
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            if self._is_interior(u) and self._is_interior(v):
                edges.add((u, v))
        return edges


# ─────────────────────────────────────────────────────────────────────────────
# Goal-set Dijkstra  (modified: stops at the cheapest node in `goals`)
# ─────────────────────────────────────────────────────────────────────────────
def dijkstra(
    roadmap: Roadmap,
    start: object,
    goals: Set[object],
    removed_nodes: Set[object] = frozenset(),
    removed_edges: Set[Tuple[object, object]] = frozenset(),
) -> Optional[Tuple[List[object], float]]:
    """Shortest path from `start` to the CHEAPEST-to-reach node in `goals`.

    `goals` is a set of up to ~5 goal nodes (one per distinct spline endpoint).
    Returns (path, cost) ending at whichever goal is reached first, or None.
    """
    goals = set(goals)
    if not goals or start in removed_nodes:
        return None

    counter = itertools.count()                     # tiebreaker: never compares node ids
    dist: Dict[object, float] = {start: 0.0}
    prev: Dict[object, object] = {}
    visited: Set[object] = set()
    pq: List[Tuple[float, int, object]] = [(0.0, next(counter), start)]

    while pq:
        d, _, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)

        if u in goals:                              # reached ANY goal -> done
            path = [u]
            while path[-1] in prev:
                path.append(prev[path[-1]])
            path.reverse()
            return path, d

        for v, w in roadmap.neighbors(u):
            if v in removed_nodes or (u, v) in removed_edges:
                continue
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, next(counter), v))
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Diverse search  (Yen's k-shortest-paths + edge-overlap diversity, goal-set form)
# ─────────────────────────────────────────────────────────────────────────────
def diverse_roadmap_search(
    roadmap: Roadmap,
    start: object,
    goals: Set[object],
    k: int = 3,
    max_overlap: float = 0.5,
    candidate_pool: int = 25,
) -> List[Tuple[List[object], float]]:
    """Up to k diverse start->(any goal) paths. A candidate is accepted only if
    its interior-edge overlap with every already-accepted path is <= max_overlap."""
    first = dijkstra(roadmap, start, goals)
    if first is None:
        return []

    accepted: List[Tuple[List[object], float]] = [first]
    candidate_heap: List[Tuple[float, int, List[object]]] = []
    seen_paths = {tuple(first[0])}
    A = [first[0]]
    tie = itertools.count()                         # heap tiebreaker (paths never compared)

    def push_yen_candidates(base_path: List[object]) -> None:
        for i in range(len(base_path) - 1):
            spur_node = base_path[i]
            root_path = base_path[: i + 1]

            removed_edges = set()
            for p in A:
                if len(p) > i and p[: i + 1] == root_path:
                    removed_edges.add((p[i], p[i + 1]))
            removed_nodes = set(root_path[:-1])

            result = dijkstra(roadmap, spur_node, goals, removed_nodes, removed_edges)
            if result is None:
                continue
            spur_path, _ = result
            total_path = root_path[:-1] + spur_path
            if tuple(total_path) in seen_paths:
                continue
            total_cost = roadmap.path_cost(total_path)
            heapq.heappush(candidate_heap, (total_cost, next(tie), total_path))
            seen_paths.add(tuple(total_path))

    if len(accepted) >= k:
        return accepted

    push_yen_candidates(first[0])

    while candidate_heap and len(A) < candidate_pool:
        cost, _, path = heapq.heappop(candidate_heap)
        A.append(path)

        new_edges = roadmap.path_edges(path)
        is_diverse = True
        for acc_path, _ in accepted:
            acc_edges = roadmap.path_edges(acc_path)
            overlap = len(new_edges & acc_edges) / max(1, len(new_edges))
            if overlap > max_overlap:
                is_diverse = False
                break

        if is_diverse:
            accepted.append((path, cost))
            if len(accepted) >= k:
                break

        push_yen_candidates(path)

    return accepted


# ─────────────────────────────────────────────────────────────────────────────
# Build a Roadmap from a bag of opponent splines
# ─────────────────────────────────────────────────────────────────────────────
def build_roadmap_from_splines(
    splines: Sequence[np.ndarray],
    node_res: float,
    goal_res: float,
    min_spline_len: float,
) -> Tuple[Roadmap, Set[object]]:
    """Fuse xy splines into a Roadmap. Returns (roadmap, goal_set).

    node_res  : grid size for interior fusion (smaller -> fewer splines fuse).
    goal_res  : grid size for clustering endpoints into goals (larger -> fewer,
                caps the goal set near the number of distinct lateral targets).
    The goal set is whatever endpoints the input splines produced, so it is <= 5
    and smaller when splines are missing.
    """
    rm = Roadmap()
    goals: Set[object] = set()

    def cell(p) -> Tuple[int, int]:
        return (int(round(p[0] / node_res)), int(round(p[1] / node_res)))

    def goal_cell(p) -> Tuple[str, int, int]:
        return ("G", int(round(p[0] / goal_res)), int(round(p[1] / goal_res)))

    for pts in splines:
        pts = np.asarray(pts, dtype=float)
        if pts.shape[0] < 2:
            continue
        # drop splines that barely move (degenerate)
        if float(np.hypot(*(pts[-1] - pts[0]))) < min_spline_len:
            continue

        prev_c = cell(pts[0])
        rm.add_point(prev_c, pts[0])
        rm.add_edge(Roadmap.START, prev_c, 0.0)      # virtual source -> spline start

        acc = 0.0
        last_c = prev_c
        for j in range(1, pts.shape[0]):
            acc += float(np.hypot(*(pts[j] - pts[j - 1])))
            c = cell(pts[j])
            if c != prev_c:
                rm.add_point(c, pts[j])
                rm.add_edge(prev_c, c, acc)          # edge cost ~ true arc length
                prev_c, acc, last_c = c, 0.0, c

        g = goal_cell(pts[-1])
        rm.add_point(g, pts[-1])
        rm.add_edge(last_c, g, 0.0)                  # spline end -> its goal node
        goals.add(g)

    return rm, goals


def path_to_xy(roadmap: Roadmap, path: Sequence[object]) -> np.ndarray:
    """Interior-cell representatives in path order (drops START and goal nodes)."""
    return np.array([roadmap.points[n] for n in path if Roadmap._is_interior(n)])


def bspline_smooth(xy: np.ndarray, n_out: int = 60, smooth: float = 0.3) -> np.ndarray:
    """Fit a parametric cubic B-spline to a polyline and return n_out resampled
    points. Arc-length parameterized so output points are ~evenly spaced, and the
    recombination snaps at fused junctions get rounded out.

    smooth : splprep's `s` (sum of squared residuals budget); 0 interpolates the
             points exactly, larger pulls the curve smoother/away from them.
    Falls back to the input unchanged when there are too few distinct points.
    """
    xy = np.asarray(xy, dtype=float)
    # splprep cannot handle zero-length segments -> drop consecutive duplicates
    keep = np.concatenate(([True], np.hypot(*np.diff(xy, axis=0).T) > 1e-6))
    xy = xy[keep]
    n = xy.shape[0]
    if n < 2:
        return xy

    seg = np.hypot(*np.diff(xy, axis=0).T)
    u = np.concatenate(([0.0], np.cumsum(seg)))
    if u[-1] < 1e-9:
        return xy
    u = u / u[-1]                               # arc-length parameter in [0, 1]

    k = min(3, n - 1)                           # cubic when possible, lower for short paths
    try:
        tck, _ = splprep([xy[:, 0], xy[:, 1]], u=u, s=smooth, k=k)
    except Exception:
        return xy
    x, y = splev(np.linspace(0.0, 1.0, n_out), tck)
    return np.column_stack([x, y])


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 node
# ─────────────────────────────────────────────────────────────────────────────
NUM_START_POINTS = 5


class DiverseOpponentTrajectories(Node):
    def __init__(self):
        super().__init__("diverse_opponent_trajectories")

        # ---- parameters ----
        self.k = 3
        self.max_overlap = 0.5
        self.candidate_pool = 25
        self.node_res = 0.20          # m, interior fusion grid
        self.goal_res = 0.60          # m, endpoint clustering grid (caps goals ~5)
        self.min_spline_len = 0.30    # m, drop near-stationary splines
        self.stale_timeout = 0.5      # s, ignore a topic whose last msg is older
        self.plan_period = 0.1        # s, rebuild/search rate
        self.marker_lifetime = 0.3    # s
        self.smooth_points = 60       # resampled points per smoothed trajectory
        self.smooth_s = 0.3           # B-spline smoothing factor (splprep s; 0 = interpolate)

        # latest splines per input topic + receipt time (ns)
        self.latest: List[List[np.ndarray]] = [[] for _ in range(NUM_START_POINTS)]
        self.latest_t: List[int] = [0 for _ in range(NUM_START_POINTS)]

        io_group = MutuallyExclusiveCallbackGroup()
        for i in range(NUM_START_POINTS):
            self.create_subscription(
                MarkerArray, f"/predicted_opponent_splines_{i}",
                self._make_cb(i), 10, callback_group=io_group)

        self.pub = self.create_publisher(MarkerArray, "/diverse_opponent_trajectories", 10)
        self.create_timer(self.plan_period, self.plan_cb)

        self.get_logger().info("diverse_opponent_trajectories ready.")

    def _make_cb(self, idx: int):
        def _cb(msg: MarkerArray):
            splines = []
            for mk in msg.markers:
                if len(mk.points) >= 2:
                    splines.append(np.array([[p.x, p.y] for p in mk.points], dtype=float))
            self.latest[idx] = splines
            self.latest_t[idx] = self.get_clock().now().nanoseconds
        return _cb

    def _gather_fresh_splines(self) -> List[np.ndarray]:
        now = self.get_clock().now().nanoseconds
        horizon = self.stale_timeout * 1e9
        out: List[np.ndarray] = []
        for i in range(NUM_START_POINTS):
            if self.latest[i] and (now - self.latest_t[i]) <= horizon:
                out.extend(self.latest[i])
        return out

    def plan_cb(self):
        splines = self._gather_fresh_splines()
        if len(splines) == 0:
            return

        roadmap, goals = build_roadmap_from_splines(
            splines, self.node_res, self.goal_res, self.min_spline_len)
        if not goals or not roadmap.neighbors(Roadmap.START):
            return

        try:
            accepted = diverse_roadmap_search(
                roadmap, Roadmap.START, goals,
                k=self.k, max_overlap=self.max_overlap,
                candidate_pool=self.candidate_pool)
        except Exception as e:  # never let a graph edge-case kill the node
            self.get_logger().warn(f"diverse search failed: {e}")
            return

        polylines = []
        for path, _cost in accepted:
            xy = path_to_xy(roadmap, path)
            if xy.shape[0] >= 2:
                polylines.append(bspline_smooth(xy, self.smooth_points, self.smooth_s))

        self.get_logger().info(
            f"{len(splines)} splines, {len(goals)} goals -> {len(polylines)} diverse trajectories",
            throttle_duration_sec=1.0)
        self.publish_trajectories(polylines)

    def publish_trajectories(self, polylines: List[np.ndarray]):
        colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.3, 1.0)]
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        # clear any previously drawn trajectory first
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = stamp
        clear.ns = "diverse_opponent_traj"
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for idx, xy in enumerate(polylines):
            mk = Marker()
            mk.header.frame_id = "map"
            mk.header.stamp = stamp
            mk.ns = "diverse_opponent_traj"
            mk.id = idx
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.07
            r, g, b = colors[idx % len(colors)]
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = r, g, b, 0.9
            mk.lifetime.sec = int(self.marker_lifetime)
            mk.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            for p in xy:
                pt = Point()
                pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.0
                mk.points.append(pt)
            arr.markers.append(mk)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DiverseOpponentTrajectories()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()