# Opponent State Estimation for F1Tenth / Roboracer Car
**Vignesh Saravanan and Mohana Pamidimukkala**\
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
**DBSCAN** was particularly effective for LiDAR point cloud data, due to it not requiring specification of the number of clusters in advance and allowing significant customization using the Epsilon and MinPts parameters, which control, respectively, the minimum separation distance between points in two different clusters and the number of points surrounding a given point in order for it to be considered a "core point" (worthy of being the center of its own cluster).
### Support Vector Machine (SVM) Classification
In the first iteration of this project, we experimented with a rule-based approach to car cluster identification that consisted primarily of simple hueristics. This method worked fine under very specific circumstances, but it proved difficult to generalize and was very susceptible to incorrectly detecting corners of the racetrack as the opponent vehicle. The SVM performed significantly better in distinguishing between the wall clusters and the opponent car cluster, and it was also more generalizable across a variety of maps (both 'levine' and 'spielberg' were tested).
### IMM Filter
Standard Kalman Filters often struggle with the non-linear behavior of racing (sudden braking or sharp turns). Our IMM filter addresses this by running three models in parallel:
1. **Constant Velocity (CV):** Assumes the opponent is traveling at fixed speed on a straight road.
2. **Constant Acceleration (CA):** Captures sudden bursts of speed or heavy braking zones.
3. **Constant Turn Rate (CTR):** Predicts behavior during cornering and hairpins.
The filter dynamically weights these models based on real-time observations, allowing for high-fidelity trajectory prediction during a variety of maneuvers.

## Example Trajectory Generated
![](https://media.discordapp.net/attachments/927036873312972850/1456829543930134679/image.png?ex=6959c9bd&is=6958783d&hm=a0c7755224338af611cafac485fd7cf8e89143a8b760e9b5e3903afe6dc15888&=&format=webp&quality=lossless&width=1157&height=642)

## Usage
### Roboracer Simulator
* Follow the instructions here for installing and running the F1Tenth simulator: [F1Tenth Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros)
* Open VSCode and attach to the running Docker container so that you can view and edit all of the simulation files and those in our perception stack (once it is cloned).
### Perception Stack
* Navigate to the `/sim_ws/src` directory within the simulation's Docker container and clone this repository there. You might need to then rename it to `lidar_processing` for the package to be built appropriately.
* Source your terminal if you haven't done so already and install dependencies listed in the package.xml file with **rosdep** as shown below:
  ``` bash
  source /opt/ros/foxy/setup.bash
  source ../install/setup.bash
  cd /sim_ws
  rosdep install -i --from-path src --rosdistro foxy -y
  ```
* Now, use `pip` to install all necessary python dependencies within the Docker container. This includes `numpy`, `pandas`, and `scikit-learn.`
* Now, build the `lidar_processing` package using the following command: `colcon build --packages-select lidar_processing`. Make sure you build the package from `/sim_ws`.
* The tool `tmux` should already be installed within the Docker container, so use it to create 4 new terminal windows. The simulator can be run in the first window. The second window can be used for running this perception package, which can be launched using the command `ros2 launch lidar_processing launch_lidar_processing.py`. The third and fourth windows can be used to run `gt_reactive_node.py` and `gt_offensive_node.py`, respectively. Both are running the `gt_follow_gap` algorithm, which we did not design but did attempt to optimize by modifying the hyperparameters. The last two nodes are not part of the `lidar_processing` package, so they can be run just as regular Python files. However, because both nodes use the `rclpy` package, make sure to source each `tmux` terminal as shown above before running its respective files.
### Simulator Car Visualization Instructions
This was not immediately obvious for us when we began working with the Roboracer simulator, so here are some instructions for visualizing the opponent vehicle, the filtered point-cloud cluster, and the path generated by the IMM filter.
* **To visualize opponent vehicle:**
  * Navigate to `/sim_ws/src/f1tenth_gym_ros/config` on VSCode and open the `sim.yaml` file. Change the `num_agents` parameter to 2 in order to visualize both the opponent and ego vehicles. Rebuild the simulator package and run it again for the changes to take effect.
  * Now, navigate to the simulator GUI and click on the `Add` button at the bottom. Select `Robot Model` within the `By Display Type` tab. Add the robot model and select `Topic` as the description source and specifically `/opp_robot_description` as the description topic. You should now see both the blue car and the orange car. Go Gators!
* **To visualize /imm_path and /filtered_cluster:**
  * Click on the `Add` button again, but this time, select the `By Topic` tab. Select the relevant topic that you want to visualize. Do this separately for both `/imm_path` and `/filtered_cluster`. I suggest selecting "Arrows" for the pose visualization in the path message, and also changing the size of the squares comprising the point cloud in the PointCloud2 message from 0.01 to 0.1. **Note:** In order to be able to add these visualizations, you must be running the perception stack. There will be no `/imm_path` or `/filterd_cluster` topic if no node is publishing to those respective topics.





