# Opponent State Estimation for F1Tenth / Roboracer Car
**Vignesh Saravanan and Mohana Pamidimukkala**
**Trustworthy Engineered Autonomy Lab**

### Architecture Info
The perception stack we developed for the F1Tenth vehicle operates in multiple stages:
1. LaserScan data from the RoboRacer simulator gets converted to the PointCloud2 message format, which is more suitable for further pre-processing. The node _ScanToCloud_ handles this conversion, using the _laser_geometry_ package in ROS2 to accomplish this.
2. The node _Clustering_ in _clustering.py_ processes this PointCloud2 message in a callback and uses the DBScan algorithm from the _scikit-learn_ library in order to cluster the point cloud data.
3. Each of the clusters is fed into an SVM model that identifies whether or not the cluster belongs to an opponent vehicle. This support vector machine was trained on a variety of features (x_center, y_center, length_bounding_box, width_bounding_box, density, aspect_ratio) ascertained from PointCloud2 messages received on the simulator; the dataset we used is present in _pc_data_new.csv_, which is present in the scripts directory along with the various Python nodes. The cluster with the highest probability of belonging to the opponent vehicle is published to the ROS2 topic _'/filtered_cluster'_.
4. The cluster that does belong to the opponent vehicle is 

