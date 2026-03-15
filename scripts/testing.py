#!/usr/bin/env python3


import signal
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

pred_horizon = 0.225 # dt = 0.15 15 * 0.015 = 0.2225 s (all 15 steps calcauted?-i'm not too sure about this right now)
odom_buff    = 1000    
print_cooldown = 50 

class IMMEvaluator(Node):
    def __init__(self):
        super().__init__('imm_evaluator')

        self._pending: list = []

        # Real car path
        self._odom: list = []

        # Per-step squared-error accumulator: step_index -> list of squared errors
        self._sq_errors: dict = {}

        self._n_evaluated = 0


        self.create_subscription(Path,     '/imm_path',         self.path_cb, 10)
        self.create_subscription(Odometry, '/opp_racecar/odom', self.odom_cb, 10)

        # buffered predictions at ~20 Hz
        self.create_timer(0.05, self.evaluate)

        self.get_logger().info(
            f"IMM Evaluator starting---pred_horizon={pred_horizon} s\n"
            "Play rosbag now-----------------------, then ctrl+c to flush RMSE output"
        )

    #adding the predicted path to the buffer
    def path_cb(self, msg: Path):
            pts = np.array([[p.pose.position.x, p.pose.position.y]
                            for p in msg.poses])
            if pts.shape[0] == 0:
                return
            self._pending.append({
                'time': msg.header.stamp.sec + msg.header.stamp.nanosec/(1e9),
                'pred': pts,
            })

    # adding odom/ real car path ot the bufer
    def odom_cb(self, msg: Odometry):
        self._odom.append({
            'time': msg.header.stamp.sec + msg.header.stamp.nanosec/(1e9),
            'pos':  np.array([msg.pose.pose.position.x,
                                msg.pose.pose.position.y]),
        })
        # Trim oldest samples once buffer is full
        if len(self._odom) > odom_buff:
            self._odom.pop(0)


    #  I'm not too sure this is going to work right now, because it calcuates with every new pose added to odom. 
    #   Instead what we should do is every time a new IMM_path is published, and we only sample ~5 of them 
    #   after doing that, we can find RMSE jsut from that,
    #   Do not have to go through full rosbag 
    def evaluate(self):
            if not self._pending or not self._odom:
                return
            
            odom_times = np.array([o['time'] for o in self._odom])
            latest_t   = odom_times[-1]

            still_pending = []
            for entry in self._pending:
                pred_t   = entry['time']
                pred_pts = entry['pred']       
                n_steps  = len(pred_pts)
                horizon_end = pred_t + pred_horizon

                # Wait until we have odom data that covers the full prediction horizon
                if latest_t < horizon_end:
                    still_pending.append(entry)
                    continue

                # Evenly-spaced target timestamps for each predicted point
                step_dt = pred_horizon / max(n_steps - 1, 1)

                for i, pred_xy in enumerate(pred_pts):
                    target_t = pred_t + i * step_dt
                    closest  = int(np.argmin(np.abs(odom_times - target_t)))
                    gt_xy    = self._odom[closest]['pos']
                    sq_err   = float(np.sum((pred_xy - gt_xy) ** 2))
                    self._sq_errors.setdefault(i, []).append(sq_err)

                self._n_evaluated += 1

                # periodic update to print 
                if self._n_evaluated % print_cooldown == 0:
                    self.print_table(live=True)

            self._pending = still_pending


    def print_table(self, live=False):
        if not self._sq_errors:
            print("[Evaluator] Not enough data yet.")
            return

        step_indices = sorted(self._sq_errors.keys())
        rmses = {i: np.sqrt(np.mean(self._sq_errors[i])) for i in step_indices}
        step_dt = pred_horizon / max(max(step_indices), 1)

        tag = "LIVE UPDATE" if live else "FINAL RESULTS"
        print(f"\nIMM Per-Step RMSE — {tag}")
        print(f"Predictions evaluated: {self._n_evaluated}\n")

        for i in step_indices:
            t_s  = i * step_dt
            rmse = rmses[i]
            print(f"  Step {i+1:02d} ({t_s:.3f}s): {rmse:.4f} m")

        all_sq = [e for lst in self._sq_errors.values() for e in lst]
        overall_rmse = np.sqrt(np.mean(all_sq))
        print(f"\n  Overall RMSE: {overall_rmse:.4f} m")



def main(args=None):
    rclpy.init(args=args)
    node = IMMEvaluator()

    def shutdown(sig, frame):
        print("\n[Evaluator] Bag finished or interrupted — flushing remaining predictions...")
        node.evaluate()
        node.print_table(live=False)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
