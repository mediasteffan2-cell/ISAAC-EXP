# Unitree Go‑2 Robot Simulation in Isaac Sim / Isaac Lab (ROS2)

This repo is tested against **Isaac Sim 5.1** and **Isaac Lab ≥ 2.3** (the Go2 velocity tasks kept the same Gym IDs across 4.5→5.1).

ROS2 note: `src/isaac_go2_ros2.py` first tries a normal `import rclpy`. If that fails (typical for pip installs on Windows/Python 3.11), it auto‑bootstraps the **bundled ROS2 Humble** that ships inside Isaac Sim under `isaacsim.ros2.bridge/humble`. So you don't need a separate ROS2 Python install to get topics out.

## Recommended host setup (pip/binaries)

Prereqs:
- Isaac Sim 5.1 + Isaac Lab 2.3+ (pip or binaries)
- A single conda environment for Isaac Sim/Isaac Lab (we use `env_isaaclab_py311`)
- A separate ROS2 Humble env for Nav2/SLAM (created automatically by `./scripts/after_clone.sh`)

### Environment quickstart
1. Install Isaac Sim/Isaac Lab into `env_isaaclab_py311` (or export `GO2_ISAAC_ENV=<your_env>` before running our scripts).
   ```bash
   # Example if you want to mirror the default name
   conda create -n env_isaaclab_py311 python=3.11
   conda activate env_isaaclab_py311
   pip install --extra-index-url https://pypi.nvidia.com isaaclab isaac-sim==5.1.*
   ```
   (Follow NVIDIA’s official instructions for the full Isaac Lab install.)

2. Create the ROS2 Humble environment (Nav2, slam_toolbox, etc.). The helper script auto-detects conda and installs the needed robostack packages:
   ```bash
   ./scripts/after_clone.sh
   # or override the name:
   GO2_ROS_ENV=my_ros ./scripts/after_clone.sh
   ```
   Afterwards, `./scripts/run_go2.sh` (simulation) and `./scripts/run_nav2_slam.sh` (Nav2) know which envs to activate. If you changed the names, export `GO2_ISAAC_ENV` / `GO2_ROS_ENV_PATH` (for scripts/ros2_env.sh) accordingly.

1. Create/activate a Python env for Isaac Lab (conda or venv).
2. Install Isaac Sim 5.1 and Isaac Lab following the official docs.
   Make sure the extra packages are present:
   - `isaaclab_tasks` (registers the Go2 Gym envs)
   - `isaaclab_assets` (robot asset configs)
   - `isaaclab_rl` (RSL‑RL wrapper)
3. Launch the simulation (script handles conda activation + ROS bridge setup):
```bash
./scripts/run_go2.sh                   # GUI + keyboard teleop
GO2_HEADLESS=1 ./scripts/run_go2.sh    # headless/livestream (default)

# Environment selection: set GO2_ENV_NAME or pass a Hydra override, e.g.
#   GO2_HEADLESS=0 ./scripts/run_go2.sh env_name=warehouse
#   ./scripts/run_go2.sh env_name=warehouse-forklifts
# Defaults to the obstacle-dense terrain so Nav2 sees obstacles immediately.
```

Policy checkpoints:
- The locomotion controller looks for RSL‑RL checkpoints in `ckpts/` at repo root.  
  If that folder is missing, it falls back to `src/ckpts/` (this repo currently ships weights there).
  Expected layout (from `src/go2/go2_ctrl_cfg.py`):
  - `src/ckpts/unitree_go2/rough_model_7850.pt`
  - `src/ckpts/unitree_go2/flat_model_6800.pt`
- You can also point to your own training logs via `export GO2_CKPTS_DIR=/path/to/ckpts`.
- If no checkpoint is found, the sim falls back to a zero‑action policy (robot stands still) so you can still test ROS/Nav2 wiring.

## Nav2 + SLAM (ROS2 Humble)

Nav2 and SLAM run **outside** Isaac Sim in a normal ROS2 Humble environment. The sim publishes:
- `/unitree_go2/odom` + TF `odom -> unitree_go2/base_link`
- `/unitree_go2/lidar/point_cloud` (PointCloud2)
- `/clock` (sim time)

LiDAR note:
- The default RTX LiDAR config is `Example_Rotary_2D` (full 360°), because limited‑FOV sensors will only map “one side” unless you turn the robot.
- To emulate a solid‑state / limited coverage sensor, set `sensor.lidar_config: HESAI_XT32_SD10` in `src/cfg/sim.yaml`.

Nav2 expects a 2D `/scan` and a TF `map -> odom`. The helper script in this repo does both:
1. Converts the Go2 point cloud to `/scan` using `pointcloud_to_laserscan`.
2. Starts `slam_toolbox` + `nav2_bringup` in SLAM mode.

Prereqs:
- ROS2 Humble with `nav2_bringup`, `slam_toolbox`, `pointcloud_to_laserscan`.
  Example via robostack (no sudo):
  ```bash
  conda create -n ros2_humble -c conda-forge -c robostack-staging \
    ros-humble-desktop \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan
  ```

Run:

Terminal 1 (Isaac Sim / Isaac Lab env):
```bash
./scripts/run_go2.sh
# Optional (useful for CI): limit runtime by steps
# GO2_MAX_STEPS=2000 GO2_HEADLESS=1 ./scripts/run_go2.sh

# Hydra overrides are supported, e.g.:
# GO2_HEADLESS=0 ./scripts/run_go2.sh env_name=office sensor.enable_camera=false
```

Terminal 2 (ROS2 Humble env):
```bash
./scripts/run_nav2_slam.sh   # auto-activates ros2_humble if needed, then runs pointcloud->scan + Nav2
```
Notes:
- `scripts/run_nav2_slam.sh` defaults to FastDDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) and `ROS_DOMAIN_ID=0` to match Isaac Sim's ROS2 bridge.
- By default it runs Nav2/SLAM with **wall time** (no sim time) to match the Go2 sim bridge timestamps. Opt-in to sim time with `GO2_USE_SIM_TIME=1`.

Quick sanity checks (before blaming Nav2):
```bash
# In the ROS2 Humble env, while the sim is running:
python3 scripts/pointcloud_stats.py   # should print points_read > 0
python3 scripts/scan_stats.py         # should print finite > 0
```

Automated Nav2 goal test (picks a “hard” goal from the current /map):
```bash
python3 scripts/test_nav2_goal.py --auto --repeat 5 --wait 120
```

If RViz/TF "jumps" or the map looks unstable, you almost certainly have **multiple Nav2/SLAM stacks running**.
Stop them and restart:
```bash
./scripts/stop_nav2_slam.sh
./scripts/run_nav2_slam.sh
```

RViz:
```bash
rviz2 -d nav2/go2_nav2.rviz
```
- Start this **after** `run_nav2_slam.sh` is running, otherwise the `map` frame doesn't exist yet.
- Fixed frame: `map` (comes from `slam_toolbox`).
- `nav2/go2_nav2.rviz` shows `/map`, `/scan` and the raw lidar PointCloud2.
- For an "everything in one window" view (Nav2 + SLAM + camera images):
  ```bash
  rviz2 -d nav2/go2_nav2_full.rviz
  ```
- Or when using the helper script:
  ```bash
  GO2_RVIZ_PROFILE=full ./scripts/run_nav2_slam.sh
  ```
- Use **“Nav2 Goal”** → Nav2 typically publishes `/cmd_vel` (and sometimes `/cmd_vel_smoothed` / `/cmd_vel_nav`).
  The bridge subscribes to all of these plus `/unitree_go2/cmd_vel`.
- To restrict accepted velocity topics: `GO2_CMDVEL_TOPICS="cmd_vel_smoothed"` (comma-separated list).

Sensors‑only RViz (no Nav2/SLAM):
```bash
rviz2 -d src/rviz/go2.rviz
```
- Fixed frame is `odom`, so it works with the sim alone.

Notes:
- To save a map: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: my_map}"`.
- For localization-only later (AMCL), switch `slam:=False` and provide a real map; we can add a helper script for that.

## Troubleshooting

- **Reference logs:** `debug/go2_run.log` (Isaac Sim bring-up) and `debug/nav2_run.log` (Nav2/SLAM bring-up) contain captured runs from this repo. Diff your output against them—e.g., if Nav2 keeps printing `Invalid frame ID "odom"` then the sim/bridge isn’t publishing `/tf` yet.
- **Keep the sim running:** Always start `./scripts/run_go2.sh` first and ensure it stays alive before launching `./scripts/run_nav2_slam.sh`. Nav2 will never activate if the sim is stopped or if `/tf` isn’t flowing.
- **Topic sanity checks:** In the ROS2 terminal run `python3 scripts/pointcloud_stats.py` and `python3 scripts/scan_stats.py`. Both should report non-zero counts; otherwise the bridge isn’t producing data (check `logs/debug_go2.log`).
- **Reproduce logs for debugging:** You can capture the exact commands we used via
  ```bash
  GO2_HEADLESS=1 GO2_MAX_STEPS=400 ./scripts/run_go2.sh > logs/debug_go2.log 2>&1 &
  GO2_NO_RVIZ=1 ./scripts/run_nav2_slam.sh > logs/debug_nav2.log 2>&1
  ```
  then copy the tails into `debug/go2_run.log` / `debug/nav2_run.log` before filing an issue.
- **RViz on ThinLinc:** set `GO2_RVIZ_SOFTWARE=1 ./scripts/run_nav2_slam.sh` to avoid GPU/GL crashes.
- **Env confusion:** Run `./scripts/after_clone.sh` once. Afterwards `./scripts/run_go2.sh` and `./scripts/run_nav2_slam.sh` activate their respective conda envs automatically. If you rename the envs, export `GO2_ISAAC_ENV` / `GO2_ROS_ENV_PATH` accordingly.
