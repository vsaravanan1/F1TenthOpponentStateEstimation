# Running the Dynamic MPPI in the f1tenth_gym_ros Sim

End-to-end run guide for the dynamic-reference MPPI (follow a divert spline, then
smoothly return to the centerline) inside the Docker + WSLg setup.

Package: **lidar_processing**  ·  ROS 2 **Foxy**  ·  Workspace: **/sim_ws**

---

## 0. One-time setup (already done, here for reference)

The display-wired container was created from WSL with WSLg so rviz shows on Windows:

```bash
# in WSL (wsl -d Ubuntu), from a snapshot image of the built container:
docker run -it --name f1tenth_viz \
  -e DISPLAY=$DISPLAY -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR -e PULSE_SERVER=$PULSE_SERVER \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
  --net=host f1tenth_built
```

Start/stop the existing container later:
```bash
docker start -ai f1tenth_viz      # start + attach
docker exec -it f1tenth_viz bash  # extra shells into it
```

---

## 1. Sourcing (EVERY new shell / tmux pane)

A fresh shell has no ROS environment. Run this first, every time:

```bash
cd /sim_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash
```

(equivalently `source ./src_stuff.sh` from /sim_ws)

> After any `colcon build`, re-source in every pane — ROS caches executable
> paths at source time.

---

## 2. Run order (3 tmux panes)

tmux: `Ctrl+b "` splits, `Ctrl+b ↑/↓` moves, `Ctrl+b z` zooms a pane.

### Pane 1 — sim bridge
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
Wait for "Managed nodes are active". rviz window appears (via WSLg).
Confirm topics exist:
```bash
ros2 topic list | grep -E "scan|odom|drive"
```

### Pane 2 — dynamic MPPI controller
```bash
ros2 run lidar_processing dynamic_mppi_node.py --ros-args \
  --params-file /sim_ws/src/lidar_processing/config/params_dynamic.yaml
```
Expect: `dynamic_mppi node initialized (state=RACELINE)`, car starts lapping.

### Pane 3 — divert injector (test tool)
```bash
ros2 run lidar_processing divert_injector_node.py --ros-args \
  -p csv_path:=/sim_ws/src/lidar_processing/config/diverts/divert_mid2.csv
```
Auto-fires ~5 s after launch.

---

## 3. Firing diverts

Re-fire any time (car should be on a straight-ish stretch):
```bash
ros2 topic pub --once /inject_divert std_msgs/msg/Empty "{}"
```

Switch shape live (no relaunch):
```bash
ros2 param set /divert_injector csv_path /sim_ws/src/lidar_processing/config/diverts/divert_right.csv
ros2 topic pub --once /inject_divert std_msgs/msg/Empty "{}"
```

Divert CSVs live in `config/diverts/` : `divert_left.csv`, `divert_right.csv`,
`divert_scurve.csv`. Format = car-relative `x,y` (x forward, y left, metres).

---

## 4. Watching it work

```bash
ros2 topic echo /mppi_state                              # RACELINE / DIVERT / RETURN
ros2 topic echo /drive                                   # commanded speed + steering
ros2 topic echo /ego_racecar/odom | grep -A3 position    # is the car moving
ros2 run tf2_ros tf2_echo map ego_racecar/base_link      # pose / TF
```

In rviz: Add → By topic → `/dynamic_trajectory` (MarkerArray) and `/map` (Map).
Markers: grey = centerline, orange = divert, green = generated return, red = optimal rollout.

Optional full centerline line:
```bash
ros2 run lidar_processing spielberg_centerline_publisher.py
```

---

## 5. Production path (instead of the injector)

Run the converter, then your planner (publishes `/ego_lane_possibilities`):
```bash
ros2 run lidar_processing path_converter_node.py --ros-args \
  --params-file /sim_ws/src/lidar_processing/config/params_dynamic.yaml
ros2 run lidar_processing generate_candidate_trajectories.py
```

---

## 6. Editing & rebuilding

- Edited a `.py` (node or module) → rebuild + re-source every pane:
  ```bash
  cd /sim_ws && colcon build --packages-select lidar_processing && source install/setup.bash
  ```
- Edited `params_dynamic.yaml` or a divert CSV → NO rebuild; just restart the node that reads it.

---

## 7. Gotchas (all hit during setup)

- **`python3\r` / `No such file or directory`** → Windows CRLF line endings on a
  script (from docker cp). Fix every file at once:
  ```bash
  find /sim_ws/src/lidar_processing -name '*.py' -exec sed -i 's/\r$//' {} +
  cd /sim_ws && colcon build --packages-select lidar_processing && source install/setup.bash
  ```
  Also strip the waypoint CSV if suspect:
  ```bash
  sed -i 's/\r$//' /sim_ws/src/lidar_processing/config/Spielberg_mppi_waypoints_sparse.csv
  ```
  Permanent fix: save files as LF (VS Code bottom-right CRLF->LF), or add a
  `.gitattributes` with `*.py text eol=lf` and `core.autocrlf input`.

- **Car won't move but /drive has nonzero speed** → sim config. In
  `install/f1tenth_gym_ros/share/f1tenth_gym_ros/config/sim.yaml`:
  - `kb_teleop: True`  → set **False** (relaunch bridge).
  - `num_agent: 2`     → set **1** for single-car testing (relaunch bridge).
  Edit BOTH the install and src copies.

- **`waypoint_path not found`** → make it absolute in params_dynamic.yaml:
  `/sim_ws/src/lidar_processing/config/Spielberg_mppi_waypoints_sparse.csv`

- **`csv_path not set`** (injector) → pass `-p csv_path:=<absolute path>` after
  `--ros-args`, or set it under `divert_injector:` in the params file.

- **`return infeasible ... offset inside corner radius`** → the guard refusing a
  divert that can't rejoin. Fire on a straighter section, lower the offset CSV,
  or raise `return_len` in params. (See TUNING below — if it rejects EVERYWHERE,
  that's a data/geometry bug, not the guard.)

- **Build fails on `pybind11_vendor`** → either `apt install ros-$ROS_DISTRO-pybind11-vendor`
  or comment the two pybind `find_package` lines in CMakeLists (nothing built uses them).

- **Two controllers / two sims** → only ONE of each; duplicate publishers fight on /drive.

---

## 8. Tuning notes

In `config/params_dynamic.yaml` under `dynamic_mppi`:

- `max_steer: 0.3` (≈17°) gives a min turn radius ~1.07 m. Spielberg's tight
  corners are ~0.55 m radius — the car physically can't make them. Try
  `max_steer: 0.4189` (≈24°) for ~0.72 m. Get a CLEAN centerline lap before testing diverts.
- `return_len: 4.0` → raise (e.g. 6.0) for gentler, more-feasible rejoins.
- `divert_speed: 0.85` → throttle during a divert.
- `min_throttle/max_throttle: 0.75/1.0` → speed band (narrow; car can't slow much for corners).
