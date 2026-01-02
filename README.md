# Opponent State Estimation for F1Tenth / Roboracer Car
**Vignesh Saravanan and Mohana Pamidimukkala**
**Trustworthy Engineered Autonomy Lab**

### Architecture Details
The perception stack we developed for the F1Tenth vehicle operates in multiple stages:
1. LaserScan data from the RoboRacer simulator gets converted to the PointCloud2 message format, which is more suitable for further pre-processing. The node _ScanToCloud_ handles this conversion, using the _laser_geometry_ package in ROS2 to accomplish this.
2. The node **Clustering** in **clustering.py** processes this PointCloud2 message in a callback and uses the DBScan algorithm from the **scikit-learn** library in order to cluster the point cloud data.
3. Each of the clusters is fed into an SVM model that identifies whether or not the cluster belongs to an opponent vehicle. This support vector machine was trained on a variety of features (x_center, y_center, length_bounding_box, width_bounding_box, density, aspect_ratio) ascertained from PointCloud2 messages received on the simulator; the dataset we used is present in **pc_data_new.csv**, which is present in the scripts directory along with the various Python nodes. The cluster with the highest probability of belonging to the opponent vehicle is published to the ROS2 topic '/filtered_cluster'.
4. The node **BoundingBox** in **bounding_box.cpp** subscribes to '/filtered_cluster' and publishes a state vector periodically, which contains information about the car's current position and velocity with respect to the map frame. In order to get this state vector, **BoundingBox** also performs a coordinate transformation from the reference frame of the ego vehicle's LiDAR to the map's reference frame. The _'tf2'_ ROS package is used to accomplish this coordinate transformation.
5. This state vector is subscribed to by the **IMM** node in **imm_filterpy.py,** which uses an _Interacting Multiple Model_ filter consisting of three _Kalman Filters_ that represent constant velocity, constant acceleration, and constant rotation trajectories respectively. The probability vector in the IMM filter determines the most likely trajectory for the opponent vehicle, and the relevant Kalman Filter model is forward propagated in order to generate the trajectory 

