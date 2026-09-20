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
            (start[0], goal[0]), (-0.8, 0.8)
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
            dist.append(dist)
        return min(dists)
    

    def check_collision_segment(self, nearest_node, new_node):
        num_steps = (new_node.s - nearest_node.s)//self.path_resolution
        for i in range(num_steps):
            clear = self.get_clearance(nearest_node.s, nearest_node.d) >= self.clearance
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
            s_diff = math.abs(new_node.s - near_node.s)
            d_diff = math.abs(new_node.d - near_node.d)
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
                d, _ = self.calc_distance_and_angle(near_node, new_node)
                costs.append(near_node.cost + d)
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"): return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node
    
    def choose_parent(self, new_node, near_inds):
        if not near_inds: return new_node
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            if self.check_collision_segment(near_node, new_node):
                s_diff, d_diff = math.abs(near_node.s - new_node.s), math.abs(near_node.d - new_node.d)
                costs.append(near_node.cost + s_diff + d_diff)
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"): return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node

    def plan_rrt(self, start, goal, step_size) -> np.ndarray:
        self.start = NodeRRT(start[0], start[1])
        self.end = NodeRRT(goal[0], goal[1])
        self.step_size = step_size
        self.node_list = [self.start]

        for _ in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            nearest_node = self.node_list[nearest_ind]
            
            new_node = self.steer(nearest_node, rnd, self.step_size)
            
            if self.check_collision_segment(nearest_node, new_node):
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds)

    
    


class RRTStarPlanner:
    def __init__(self, obstacle_kdtree, search_bounds, expand_dis=0.5, path_resolution=0.1, max_iter=600, clearance=0.45):
        self.obstacle_kdtree = obstacle_kdtree
        self.min_x, self.max_x, self.min_y, self.max_y = search_bounds
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_iter = max_iter
        self.clearance = clearance
        self.node_list = []

    def plan(self, start, goal):
        self.start = NodeRRT(start[0], start[1])
        self.end = NodeRRT(goal[0], goal[1])
        self.node_list = [self.start]

        for _ in range(self.max_iter):
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            nearest_node = self.node_list[nearest_ind]
            
            new_node = self.steer(nearest_node, rnd, self.expand_dis)
            
            if self.check_collision_segment(nearest_node, new_node):
                near_inds = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_inds)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_inds)

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)
        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = NodeRRT(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        extend_length = min(extend_length, d)
        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)
        new_node.parent = from_node
        new_node.cost = from_node.cost + extend_length
        return new_node

    def get_random_node(self):
        if random.random() > 0.15:
            return NodeRRT(random.uniform(self.min_x, self.max_x), random.uniform(self.min_y, self.max_y))
        return NodeRRT(self.end.x, self.end.y)

    def get_nearest_node_index(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        return dlist.index(min(dlist))

    def check_collision_segment(self, node1, node2):
        if self.obstacle_kdtree is None: return True
        d, theta = self.calc_distance_and_angle(node1, node2)
        steps = int(d / self.path_resolution)
        for i in range(steps + 1):
            px = node1.x + i * self.path_resolution * math.cos(theta)
            py = node1.y + i * self.path_resolution * math.sin(theta)
            dist, _ = self.obstacle_kdtree.query([px, py])
            if dist <= self.clearance:
                return False
        return True

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = 1.5 * self.expand_dis * math.sqrt(math.log(nnode) / nnode)
        dlist = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]
        return [i for i, d in enumerate(dlist) if d <= r**2]

    def choose_parent(self, new_node, near_inds):
        if not near_inds: return new_node
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            if self.check_collision_segment(near_node, new_node):
                d, _ = self.calc_distance_and_angle(near_node, new_node)
                costs.append(near_node.cost + d)
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"): return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost
        return new_node

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            d, _ = self.calc_distance_and_angle(near_node, new_node)
            scost = new_node.cost + d
            if near_node.cost > scost and self.check_collision_segment(near_node, new_node):
                near_node.parent = new_node
                near_node.cost = scost

    def search_best_goal_node(self):
        dist_to_goal = [(n.x - self.end.x)**2 + (n.y - self.end.y)**2 for n in self.node_list]
        goal_inds = [i for i, d in enumerate(dist_to_goal) if d <= self.expand_dis**2]
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

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        return path[::-1]

    def simplify_path(self, path):
        if not path or len(path) <= 2:
            return path
        simplified = [path[0]]
        curr = 0
        while curr < len(path) - 1:
            furthest = curr + 1
            for i in range(len(path) - 1, curr, -1):
                node_a = NodeRRT(path[curr][0], path[curr][1])
                node_b = NodeRRT(path[i][0], path[i][1])
                if self.check_collision_segment(node_a, node_b):
                    furthest = i
                    break
            simplified.append(path[furthest])
            curr = furthest
        return simplified

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


class OvertakeInterceptorNode(Node):
    def __init__(self):
        super().__init__('overtake_interceptor')
        
        self.ego_odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.ego_odom_callback, 10)
        self.opp_odom_sub = self.create_subscription(Odometry, '/opp_racecar/odom', self.opp_odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/opp_scan', self.scan_callback, 10)
        
        self.status_sub = self.create_subscription(String, '/overtake_status', self.status_callback, 10)
        
        self.overtake_path_pub = self.create_publisher(Path, '/overtake_spline', 10)
        self.mode_pub = self.create_publisher(String, '/driving_mode', 10)
        self.marker_pub = self.create_publisher(Marker, '/overtake_marker', 10)
        
        self.opp_x = self.opp_y = self.opp_yaw = self.opp_speed = 0.0     
        self.ego_x = self.ego_y = self.ego_yaw = self.ego_speed = 0.0     
        
        self.ego_data_received = False
        self.opp_data_received = False
        
        self.scan_ranges = np.array([])
        self.scan_angle_min = self.scan_angle_inc = 0.0
        
        self.car_width = 0.30
        self.lateral_clearance = 0.85 # FIX: Increased for wider pass
        self.path_resolution = 50
        
        self.max_display_distance = 3.5   
        self.min_ego_distance = 0.5 
        
        self.current_mode = "FTG"
        self.controller_state = "FTG" 
        self.current_side = "left"
        self.locked_spline = None
        self.debug_counter = 0
        
        self.create_timer(0.05, self.continuous_spline_update)
        self.get_logger().info("🎯 PLANNER Started - FIX: LENGTH ENFORCER & PARALLEL TANGENTS ACTIVE!")

    def status_callback(self, msg):
        self.controller_state = msg.data

    def set_mode(self, new_mode):
        if new_mode != self.current_mode:
            self.get_logger().info(f"\n====================================\n🚦 PLANNER TRANSITION: {self.current_mode} ➡️ {new_mode}\n====================================")
            self.current_mode = new_mode
            self.mode_pub.publish(String(data=self.current_mode))

    def opp_odom_callback(self, msg):
        self.opp_x = msg.pose.pose.position.x
        self.opp_y = msg.pose.pose.position.y
        self.opp_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.opp_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.opp_data_received = True

    def ego_odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        self.ego_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.ego_data_received = True

    def scan_callback(self, msg):
        self.scan_ranges = np.array(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def continuous_spline_update(self):
        self.debug_counter += 1
        if not (self.ego_data_received and self.opp_data_received):
            return

        if self.controller_state == "FTG":
            self.locked_spline = None

        opp_pos = np.array([self.opp_x, self.opp_y])
        ego_pos = np.array([self.ego_x, self.ego_y])
        distance = np.linalg.norm(ego_pos - opp_pos)

        if self.locked_spline is not None and distance > (self.max_display_distance + 1.5):
            self.locked_spline = None
            self.set_mode("FTG")
            self.publish_empty_path()

        if not self.is_obstacle_ahead():
            self.publish_empty_path()
            self.locked_spline = None
            self.set_mode("FTG")
            return

        if self.locked_spline is not None:
            self.publish_path(self.locked_spline)
            self.set_mode("OVERTAKE_ACTIVE")
            return
        
        if distance < self.max_display_distance and distance >= self.min_ego_distance:
            self.update_overtake_side()
            new_spline = self.generate_safe_spline()
            
            if new_spline is not None:
                self.locked_spline = new_spline 
                self.publish_path(new_spline)
                self.set_mode("OVERTAKE_ACTIVE")
                self.publish_continuous_marker()
            else:
                self.publish_empty_path()
                self.set_mode("FTG")
        else:
            self.publish_empty_path()
            self.set_mode("FTG")

    def is_obstacle_ahead(self):
        dx = self.ego_x - self.opp_x
        dy = self.ego_y - self.opp_y
        hx = math.cos(self.opp_yaw)
        hy = math.sin(self.opp_yaw)
        # FIX: Drops the spline earlier (-0.5m) to cleanly transition to FTG as we pass
        return (dx * hx + dy * hy) > -0.5 

    def update_overtake_side(self):
        opp_forward = np.array([math.cos(self.opp_yaw), math.sin(self.opp_yaw)])
        to_ego = np.array([self.ego_x - self.opp_x, self.ego_y - self.opp_y])
        cross_product = np.cross(opp_forward, to_ego)
        
        if self.current_side == "left" and cross_product > 0.8:
            self.current_side = "right"
        elif self.current_side == "right" and cross_product < -0.8:
            self.current_side = "left"
        elif self.current_side not in ["left", "right"]:
            self.current_side = "left" if cross_product < 0 else "right"

    def get_obstacle_kdtree(self):
        if len(self.scan_ranges) == 0:
            return None
        obstacles = []
        for i, r in enumerate(self.scan_ranges):
            if math.isfinite(r) and 0.1 < r < 8.0:
                angle = self.opp_yaw + self.scan_angle_min + i * self.scan_angle_inc
                ox = self.opp_x + r * math.cos(angle)
                oy = self.opp_y + r * math.sin(angle)
                obstacles.append([ox, oy])

        for angle in np.linspace(0, 2*math.pi, 8):
            for radius in [0.0, 0.2, 0.4]:
                obstacles.append([self.ego_x + radius*math.cos(angle), self.ego_y + radius*math.sin(angle)])

        if len(obstacles) > 0:
            return KDTree(obstacles)
        return None

    def generate_safe_spline(self):
        spline = self.try_generate_rrt_spline(self.current_side)
        if spline is not None: return spline
        
        other_side = "right" if self.current_side == "left" else "left"
        spline = self.try_generate_rrt_spline(other_side)
        
        if spline is not None:
            self.current_side = other_side
            self.get_logger().info(f"🔄 Switched pass to {self.current_side.upper()} via RRT* fallback!")
            return spline
        return None

    def try_generate_rrt_spline(self, side):
        opp_pos = np.array([self.opp_x, self.opp_y])
        ego_pos = np.array([self.ego_x, self.ego_y]) 
        opp_forward = np.array([math.cos(self.opp_yaw), math.sin(self.opp_yaw)])
        ego_forward = np.array([math.cos(self.ego_yaw), math.sin(self.ego_yaw)])
        
        if side == "left":
            lateral_dir = np.array([-ego_forward[1], ego_forward[0]])
        else:
            lateral_dir = np.array([ego_forward[1], -ego_forward[0]])
            
        kdtree = self.get_obstacle_kdtree()
        if kdtree is None: return None
            
        forward_dist = max(5.0, self.ego_speed * 1.5)
        base_target = ego_pos + forward_dist * ego_forward
        
        # FIX: Ensure target is ALWAYS at least 5.0m ahead of Opponent to prevent tiny backward splines
        to_target = base_target - opp_pos
        dist_ahead_of_opp = np.dot(to_target, opp_forward)
        if dist_ahead_of_opp < 5.0:
            base_target += (5.0 - dist_ahead_of_opp) * opp_forward
            
        end_point = base_target + self.lateral_clearance * lateral_dir
        
        # FIX: Fog of War Safety pulls back strictly along the lane
        dist_from_opp = np.linalg.norm(end_point - opp_pos)
        if dist_from_opp > 7.0:
            end_point -= (dist_from_opp - 7.0) * ego_forward
                
        steps = int(np.linalg.norm(end_point - ego_pos) / 0.1)
        ray_vec = end_point - ego_pos
        ray_norm = np.linalg.norm(ray_vec)
        
        if ray_norm > 0.8:
            ray_dir = ray_vec / ray_norm
            for i in range(8, steps + 1): 
                test_pt = ego_pos + i * 0.1 * ray_dir
                dist, _ = kdtree.query(test_pt)
                if dist < 0.35: 
                    end_point = test_pt - 0.45 * ray_dir
                    break

        dist_total = np.linalg.norm(end_point - ego_pos)
        if dist_total < 0.5:
            return None
        
        margin = 2.0 
        min_x = min(self.opp_x, end_point[0]) - margin
        max_x = max(self.opp_x, end_point[0]) + margin
        min_y = min(self.opp_y, end_point[1]) - margin
        max_y = max(self.opp_y, end_point[1]) + margin
        bounds = [min_x, max_x, min_y, max_y]
        
        planner = RRTStarPlanner(
            obstacle_kdtree=kdtree,
            search_bounds=bounds,
            expand_dis=0.5,           
            path_resolution=0.1,
            max_iter=600,             
            clearance=0.38            
        )
        
        rrt_path = planner.plan([self.opp_x, self.opp_y], [end_point[0], end_point[1]])
        if rrt_path is None or len(rrt_path) < 2:
            return None
            
        pruned_path = planner.simplify_path(rrt_path)
        
        # FIX: End tangency is strictly parallel to ego_forward to prevent hooks
        start_guide = [self.opp_x + 1.2 * opp_forward[0], self.opp_y + 1.2 * opp_forward[1]]
        end_guide = end_point - 1.2 * ego_forward
            
        if len(pruned_path) > 1:
            if planner.check_collision_segment(NodeRRT(self.opp_x, self.opp_y), NodeRRT(start_guide[0], start_guide[1])):
                pruned_path.insert(1, start_guide)
            if planner.check_collision_segment(NodeRRT(end_guide[0], end_guide[1]), NodeRRT(end_point[0], end_point[1])):
                pruned_path.insert(-1, end_guide.tolist())
        
        dense_path = []
        for i in range(len(pruned_path)-1):
            p1 = np.array(pruned_path[i])
            p2 = np.array(pruned_path[i+1])
            dist = np.linalg.norm(p2 - p1)
            steps = max(2, int(dist / 0.3))
            for t in np.linspace(0, 1, steps)[:-1]:
                dense_path.append((1-t)*p1 + t*p2)
        dense_path.append(pruned_path[-1])
            
        try:
            x = [p[0] for p in dense_path]
            y = [p[1] for p in dense_path]
            k_val = min(3, len(dense_path) - 1)
            tck, _ = splprep([x, y], s=0.05, k=k_val) 
            u_new = np.linspace(0, 1, self.path_resolution)
            x_smooth, y_smooth = splev(u_new, tck)
            
            return np.vstack((x_smooth, y_smooth)).T.tolist()
        except Exception as e:
            self.get_logger().error(f"Spline smoothing failed: {e}")
            return None

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, point in enumerate(path_points):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.1
            
            if i < len(path_points) - 1:
                yaw = math.atan2(path_points[i+1][1] - point[1], path_points[i+1][0] - point[0])
            else:
                yaw = 0.0
                
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)
            
        self.overtake_path_pub.publish(path_msg)

    def publish_empty_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        self.overtake_path_pub.publish(path_msg)

    def publish_continuous_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "overtake_origin"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.ego_x)
        marker.pose.position.y = float(self.ego_y)
        marker.pose.position.z = 0.3
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        if self.current_side == "left":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
            
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(marker)

    @staticmethod
    def quat_to_yaw(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def main(args=None):
    rclpy.init(args=args)
    node = OvertakeInterceptorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()