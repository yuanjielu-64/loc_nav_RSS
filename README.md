# nav_loc_ws — Navigation + Localization workspace (your own)

ROS2 humble colcon workspace where you write your own navigation/localization
glue on top of `go2_robot_sdk` (in `../go2_ros2_ws/`).

## Layout

```
nav_loc_ws/
├── src/
│   ├── nav_loc_bringup/        top-level launch + rviz preset
│   │   ├── launch/bringup.launch.py
│   │   ├── rviz/nav_loc.rviz
│   │   ├── package.xml
│   │   └── setup.py
│   ├── nav_loc_localization/   EKF / AMCL goes here
│   │   ├── nav_loc_localization/   <- python modules go in here
│   │   ├── launch/localization.launch.py
│   │   ├── config/ekf.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   └── nav_loc_navigation/     Nav2 goes here
│       ├── nav_loc_navigation/
│       ├── launch/navigation.launch.py
│       ├── config/nav2_params.yaml
│       ├── package.xml
│       └── setup.py
├── build/   (colcon)
├── install/ (colcon)
└── log/     (colcon)
```

## Build

```bash
cd ~/Desktop/navigation/nav_loc_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` lets you edit launch/yaml without rebuilding.

## Run

After `start_go2_sdk.sh` (or `ros2 launch go2_robot_sdk robot.launch.py ...`)
is up so we have `/joint_states`, `/tf`, `/scan`, `/utlidar/*`:

```bash
ros2 launch nav_loc_bringup bringup.launch.py
```

Args:
- `localization:=false` — skip localization
- `navigation:=false`   — skip navigation
- `rviz:=false`         — no rviz window

## Where to look in go2_robot_sdk for inspiration

| Topic | File |
|---|---|
| nav2 + slam launch composition | `go2_ros2_ws/src/go2_robot_sdk/launch/robot.launch.py` |
| nav2 params | `go2_ros2_ws/src/go2_robot_sdk/config/nav2_params.yaml` |
| slam_toolbox params | `go2_ros2_ws/src/go2_robot_sdk/config/mapper_params_online_async.yaml` |
| how driver bridges /lowstate -> joint_states | `go2_ros2_ws/src/go2_robot_sdk/go2_robot_sdk/infrastructure/ros2/ros2_publisher.py` |
| URDF including realsense | `go2_ros2_ws/src/go2_robot_sdk/urdf/go2_with_realsense.urdf` |

Copy the bits you need into our own packages and tweak.
