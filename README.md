# Opponent State Estimation for F1Tenth / Roboracer Car
**Vignesh Saravanan and Mohana Pamidimukkala**
**Trustworthy Engineered Autonomy Lab**

## System Architecture
The perception stack we developed for the F1Tenth vehicle operates in multiple stages:
1. LaserScan data from the RoboRacer simulator gets converted to the PointCloud2 message format, which is more suitable for further pre-processing. The node **ScanToCloud** handles this conversion, using the **laser_geometry** package in ROS2 to accomplish this.
2. The node **Clustering** in **clustering.py** processes this PointCloud2 message in a callback and uses the DBScan algorithm from the **scikit-learn** library in order to cluster the point cloud data.
3. Each of the clusters is fed into an SVM model that identifies whether or not the cluster belongs to an opponent vehicle. This support vector machine was trained on a variety of features (x_center, y_center, length_bounding_box, width_bounding_box, density, aspect_ratio) ascertained from PointCloud2 messages received on the simulator; the dataset we used is present in **pc_data_new.csv**, which is present in the scripts directory along with the various Python nodes. The cluster with the highest probability of belonging to the opponent vehicle is published to the ROS2 topic '/filtered_cluster'.
4. The node **BoundingBox** in **bounding_box.cpp** subscribes to '/filtered_cluster' and publishes a state vector periodically, which contains information about the car's current position and velocity with respect to the map frame. In order to get this state vector, **BoundingBox** also performs a coordinate transformation from the reference frame of the ego vehicle's LiDAR to the map's reference frame. The **tf2** package is used to accomplish this coordinate transformation.
5. This state vector is subscribed to by the **IMM** node in **imm_filterpy.py,** which uses an _Interacting Multiple Model_ filter consisting of three _Kalman Filters_ that assume constant velocity, constant acceleration, and constant rotation trajectories respectively. The probability vector in the IMM filter determines the most likely trajectory for the opponent vehicle, and the relevant Kalman Filter model is forward-propagated in order to generate the opponent trajectory.

## Design Choices
### DBSCAN Clustering
We used **DBSCAN** because it is particularly effective for LiDAR data, given that it doesn't require specification of the number of clusters in advance and allows significant customization using the Epsilon and MinPts parameters, which control, respectively, the minimum separation distance between points in two different clusters and the number of points surrounding a given point in order for it to be considered a "core point" (worthy of being the center of its own cluster).
### Support Vector Machine (SVM) Classification
In the first iteration of this project, we experimented with a rule-based approach to car cluster identification that consisted primarily of simple hueristics. This method worked fine under very specific circumstances, but it proved difficult to generalize and was very susceptible to incorrectly detecting corners of the racetrack as the opponent vehicle. The SVM performed significantly better in distinguishing between the wall clusters and the opponent car cluster, and it was also more generalizable across a variety of maps (both 'levine' and 'spielberg' were tested). 





