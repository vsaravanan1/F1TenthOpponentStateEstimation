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
from sklearn.ensemble import RandomForestClassifier
from std_msgs.msg import Float64MultiArray
from cv2 import minAreaRect

class ClusteringNode(Node):
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
        
        # classification model (RF) and standard scaler based on training data
        self.model, self.scaler = self.train_model('/sim_ws/pc_data_revised.csv')
        
    def cluster_callback(self, msg):
        """Uses the DBSCAN algorithm to cluster point-cloud data and a random forest model to identify which cluster belongs to the opponent vehicle"""
        points = np.array(list(pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True)))
        clustering = DBSCAN(eps=0.15, min_samples=10).fit(points)
        labels = clustering.labels_
        unique_labels = np.unique(labels)

        clusters = []
    
        for label in unique_labels:
            if label == -1:
                continue 
            cluster_points = points[labels == label]
            clusters.append(cluster_points)

        
        clusters.sort(key=lambda x:len(x), reverse=True)
        publishers = [self.cluster_pub, self.cluster_pub2, self.cluster_pub3, self.cluster_pub4, self.cluster_pub5]
    
        published_car_cluster = False 
        car_cluster = np.zeros(1)
        max_prob = 0.00
        car_cluster_detected = False

        for i, cluster in enumerate(clusters[:5]):
            # publish to sample clusters (used for training data compilation)
            publishers[i].publish(pc2.create_cloud_xyz32(msg.header, cluster))

            x_ctr, y_ctr, z_ctr = np.average(cluster[:, 0], axis=0), np.average(cluster[:, 1], axis=0), np.average(cluster[:, 2], axis=0)
            points_2d = cluster[:,0:2]
            bb_info = minAreaRect(points_2d.astype(np.float32))
            max_dim = max(bb_info[1])
            min_dim = min(bb_info[1])
            num_points = len(cluster)

            # derived characteristics (apsect ratio and density)
            aspect_ratio = max_dim/min_dim
            density = num_points / (max_dim * min_dim)
            new_data = np.array([x_ctr, y_ctr, max_dim, min_dim, num_points, aspect_ratio, density])

            # standardize data
            new_data_std = self.scaler.transform(new_data.reshape(1, -1))
            is_car_cluster = self.model.predict(new_data_std)
            if int(is_car_cluster) == 1:
                car_cluster_detected = True
                prob = self.model.predict_proba(new_data_std)[0][1]
                # bias toward clusters with more points
                if prob > max_prob + self.switching_threshold:
                    car_cluster = cluster
                    max_prob = prob
        if car_cluster_detected:
            self.filtered_cluster_pub.publish(pc2.create_cloud_xyz32(msg.header, car_cluster))
    
    

    def train_model(self, csv_file : str):
            """Parses training data from a csv file into a pandas DataFrame, performs additional pre-processing, and then
            trains the RBF-kernelized support vector machine to classify whether a cluster is that of the opponent vehicle"""
            df = pd.read_csv(csv_file).iloc[:, 1:].fillna(value=0.0)
            pd.options.mode.use_inf_as_na = True
            median_values = pd.DataFrame({label: [np.median(df[label]) for _ in range(len(df.index))] for label in df.columns})
            # print(df.to_string())
            df = df.fillna(value=median_values)
            print(df.columns)
            X = df[['x_ctr', 'y_ctr', 'max_dim', 'min_dim', 'num_points', 'aspect_ratio', 'density']]
            y = df['car_cluster']
            # print(X)
            # print(y)
            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.2, random_state=42, stratify=y, shuffle=True)
            X_train, X_test, y_train, y_test = X_train.reset_index(drop=True), X_test.reset_index(drop=True), y_train.reset_index(drop=True), y_test.reset_index(drop=True)
            # print(X_train, X_test, y_train, y_test, sep='\n')
            # print(X_train)
            scaler = StandardScaler().fit(X_train)
            X_train_std = scaler.transform(X_train)
            X_test_std = scaler.transform(X_test)
            # print(X_train_std)

            clf = RandomForestClassifier(100, max_depth = 10, max_features='sqrt', min_samples_leaf=2, class_weight='balanced')
            clf.fit(X_train_std, y_train)
            self.get_logger().info(f"Score: {clf.score(X_test_std, y_test)}")
            return clf, scaler
        

def main(args=None):
    rclpy.init(args=args)
    clustering_node = ClusteringNode()
    rclpy.spin(clustering_node)
    clustering_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
