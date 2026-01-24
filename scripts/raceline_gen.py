import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        
        # Change the topic name if your robot uses a different one
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
            
        self.file_path = 'recorded_waypoints.csv'
        self.waypoints = []
        
        # Initialize CSV file with headers
        with open(self.file_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])

        self.get_logger().info(f"Logging odom to {self.file_path}. Drive the car now!")

    def odom_callback(self, msg):
        # Extract x and y
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # To avoid massive files, only save if the car has moved a certain distance
        if not self.waypoints or self.has_moved_enough(x, y):
            self.waypoints.append((x, y))
            self.save_to_csv(x, y)
            self.get_logger().info(f"Point saved: {x:.2f}, {y:.2f}")

    def has_moved_enough(self, x, y, threshold=0.1):
        # Only log a new point if the car has moved 10cm from the last point
        last_x, last_y = self.waypoints[-1]
        dist = ((x - last_x)**2 + (y - last_y)**2)**0.5
        return dist > threshold

    def save_to_csv(self, x, y):
        with open(self.file_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([x, y])

def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Logging stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()