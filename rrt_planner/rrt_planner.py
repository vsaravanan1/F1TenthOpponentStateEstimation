#!/usr/bin/env python3

import math
import random
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from scipy.interpolate import splprep, splev
from scipy.spatial import KDTree
from racetrack_utilities.racetrack_utilities import RacetrackUtilities

class NodeRRT:
    def __init__(self, s, d):
        self.s = s
        self.d = d
        self.parent = None
        self.cost = 0.0

class RRTStarPlanner:
    def __init__(self, obstacles : list, max_iter=600, path_resolution = 0.1, clearance = 0.5):
        self.obstacles = obstacles
        self.max_iter = max_iter
        self.path_resolution = path_resolution
        self.clearance = clearance

        map_csv_path = "/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv"
        self.rutil = RacetrackUtilities(map_csv_path)
        
    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.s - rnd_node.s)**2 + (node.d - rnd_node.d)**2 for node in node_list]
        return dlist.index(min(dlist))

    def get_random_node(self, start, goal):
        bounds = [
            (start.s, goal.s), (-0.8, 0.8)
        ]

        rng = np.random.default_rng()
        rand_s, rand_d = rng.uniform(low=bounds[0][0], high=bounds[0][1]), rng.uniform(low=bounds[1][0], high=bounds[1][1])

        rrt_node = NodeRRT(rand_s, rand_d)
        return rrt_node

    def steer(self, from_node, to_node, extend_s=float("inf")):
        new_node = NodeRRT(from_node.s, from_node.d)
        s_diff = to_node.s - from_node.s
        d_diff = to_node.d - from_node.d


        extend_s = min(s_diff, extend_s)
        scale = extend_s / s_diff
        extend_d = scale * d_diff

        new_node.s += extend_s
        new_node.d += extend_d
        new_node.parent = from_node
        new_node.cost = from_node.cost + extend_s + extend_d

        return new_node

    def get_clearance(self, s, d):
        dists = []
        for obs in self.obstacles:
            obs_s, obs_d = obs
            dist = math.sqrt((obs_s - s)**2 + (obs_d - d)**2)
            dists.append(dist)
        return min(dists)
    

    def check_collision_segment(self, nearest_node, new_node):
        num_steps = int((new_node.s - nearest_node.s)//self.path_resolution)
        for i in range(num_steps):
            candidate_s = nearest_node.s + i * (new_node.s - nearest_node.s)/num_steps
            candidate_d = nearest_node.d + i * (new_node.d - nearest_node.d)/num_steps
            clear = self.get_clearance(candidate_s, candidate_d) >= self.clearance
            if not clear:
                return False
        return True

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = 1.5 * self.step_size * math.sqrt(math.log(nnode) / nnode)
        dlist = [(node.s - new_node.s)**2 + (node.d - new_node.d)**2 for node in self.node_list]
        return [i for i, d in enumerate(dlist) if d <= r**2]

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            s_diff = abs(new_node.s - near_node.s)
            d_diff = abs(new_node.d - near_node.d)
            scost = new_node.cost + s_diff + d_diff
            if near_node.cost > scost and self.check_collision_segment(near_node, new_node):
                near_node.parent = new_node
                near_node.cost = scost
    
    def choose_parent(self, new_node, near_inds):
        if not near_inds: return new_node
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            if self.check_collision_segment(near_node, new_node):
                s_diff, d_diff = abs(near_node.s - new_node.s), abs(near_node.d - new_node.d)
                costs.append(near_node.cost + s_diff + d_diff)
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"): return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node

    def search_best_goal_node(self):
        dist_to_goal = [(n.s - self.end.s)**2 + (n.d - self.end.d)**2 for n in self.node_list]
        goal_inds = [i for i, d in enumerate(dist_to_goal) if d <= self.step_size**2]
        if not goal_inds:
            return None
        safe_goal_inds = []
        for i in goal_inds:
            if self.check_collision_segment(self.node_list[i], self.end):
                safe_goal_inds.append(i)
        if not safe_goal_inds:
            return None
        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        return safe_goal_inds[[self.node_list[i].cost for i in safe_goal_inds].index(min_cost)]

    def generate_final_course(self, last_idx):
        path = [self.node_list[last_idx]]
        curr = self.node_list[last_idx]
        while curr.parent != None:
            curr = curr.parent
            path.append(curr)
        path = path[::-1]
        return path
        

    def plan_rrt(self, start, goal, step_size) -> np.ndarray:
        self.start = start
        self.end = goal
        self.step_size = step_size
        self.node_list = [self.start]

        for _ in range(self.max_iter):
            rnd = self.get_random_node(self.start, self.end)
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            nearest_node = self.node_list[nearest_ind]
            
            new_node = self.steer(nearest_node, rnd, self.step_size)
            
            if self.check_collision_segment(nearest_node, new_node):
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds)

        last_idx = self.search_best_goal_node()

        if last_idx is not None:
            nodes_path = self.generate_final_course(last_idx)
            path_frenet = np.array([[a.s, a.d] for a in nodes_path])
            return path_frenet
        else:
            return None
