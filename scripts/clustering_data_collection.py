#!/usr/bin/env python3
from sklearn.cluster import DBSCAN
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import csv
from sklearn import svm
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from std_msgs.msg import Float64MultiArray
from cv2 import minAreaRect
from nav_msgs.msg import Odometry
import threading


class ClusteringDataCollection(Node):
    def __init__(self):
        super().__init__('clustering_node')
        self.point_cloud_sub = self.create_subscription(PointCloud2, '/pointcloud', self.cluster_callback, 10)

        # sample cluster topics used for training data collection
        self.cluster_pub = self.create_publisher(PointCloud2, '/cluster1', 10)
        self.cluster_pub2 = self.create_publisher(PointCloud2, '/cluster2', 10)
        self.cluster_pub3 = self.create_publisher(PointCloud2, '/cluster3', 10)
        self.cluster_pub4 = self.create_publisher(PointCloud2, '/cluster4', 10)
        self.cluster_pub5 = self.create_publisher(PointCloud2, '/cluster5', 10)

        # publisher for filtered LiDAR cluster (identified as opponent vehicle)
        self.filtered_cluster_pub = self.create_publisher(PointCloud2, '/filtered_cluster', 10)
        self.switching_threshold = 0.2
        self.input_thread = threading.Thread(target=self.get_user_input, daemon=True)
        # self.record_count = np.int64(1)

        self.last_x_pos = 0.0
        self.last_y_pos = 0.0
        
        self.record_count = 0

        self.input_thread.start()

    def get_user_input(self):
        while rclpy.ok():
            if self.record_count == 0:
                user_str = input("Would you like to record new data? ")
                if user_str.lower() == 'y':
                    self.record_count = 1
                else:
                    self.record_count = 0

    def cluster_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True)))
        clustering = DBSCAN(eps=0.15, min_samples=10).fit(points)
        labels = clustering.labels_
        unique_labels = np.unique(labels)

        clusters : np.array = []
    
        for label in unique_labels:
            if label == -1:
                continue 
            cluster_points = points[labels == label]
            clusters.append(cluster_points)

        
        clusters.sort(key=lambda x:len(x), reverse=True)


        publishers = [self.cluster_pub, self.cluster_pub2, self.cluster_pub3, self.cluster_pub4, self.cluster_pub5]

        for i, cluster in enumerate(clusters[:5]):
            # publish to sample clusters (used for training data compilation)
            publishers[i].publish(pc2.create_cloud_xyz32(msg.header, cluster))
            # get center of bounding box and dimensions of bounding box
            x_ctr, y_ctr, z_ctr = np.average(cluster[:, 0], axis=0), np.average(cluster[:, 1], axis=0), np.average(cluster[:, 2], axis=0)
            points_2d = cluster[:,0:2]
            bb_info = minAreaRect(points_2d.astype(np.float32))
            max_dim = max(bb_info[1])
            min_dim = min(bb_info[1])
            num_points = len(cluster)

            # derived characteristics (apsect ratio and density)
            aspect_ratio = max_dim/min_dim
            density = num_points / (max_dim * min_dim)
            new_data = np.array([np.float32(i+1), x_ctr, y_ctr, max_dim, min_dim, num_points, aspect_ratio, density])
            with open("pc_data_revised.csv", "a") as csvfile:
                writer = csv.writer(csvfile)
                if self.record_count > 0:
                    writer.writerow(new_data)
        self.record_count -= 1

def main(args=None):
    rclpy.init(args=args)
    clustering_dc = ClusteringDataCollection()
    rclpy.spin(clustering_dc)
    clustering_dc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
