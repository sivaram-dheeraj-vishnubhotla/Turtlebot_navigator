# TurtleBot Navigator

A* path planning for a differential-drive robot: visualize navigation on a 2D
map, then run the same planner with a TurtleBot3 Waffle in Gazebo. The simulation
uses saved obstacle geometry; it does not perform SLAM or live obstacle mapping.

## Files

| File or directory | Purpose |
| --- | --- |
| [visualize_astar.py](visualize_astar.py) | Creates the offline map, visualizes A* exploration, and saves a route animation. |
| [astar_search.py](turtlebot_navigator/scripts/astar_search.py) | Shared A* search, differential-drive motions, and collision checks. |
| [navigate_robot.py](turtlebot_navigator/scripts/navigate_robot.py) | Plans a Gazebo route and runs the ROS controller. |
| [motion_controller.py](turtlebot_navigator/scripts/motion_controller.py) | Tracks planned motions with odometry feedback and stop checks. |
| [launch/](turtlebot_navigator/launch/) | Starts Gazebo, spawns the robot, and launches the bridge. |
| [worlds/](turtlebot_navigator/worlds/) | Competition and empty test arenas. |
| [models/](turtlebot_navigator/models/), [urdf/](turtlebot_navigator/urdf/), [config/](turtlebot_navigator/config/) | Robot assets, transforms, and ROS–Gazebo topic configuration. |

## Requirements

Offline: Python 3, NumPy, and OpenCV. Simulation: Ubuntu 24.04, ROS 2 Jazzy,
Gazebo Harmonic, and colcon. With ROS Jazzy installed and its apt repository configured:

```bash
sudo apt install python3-numpy python3-opencv python3-colcon-common-extensions \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-robot-state-publisher
```

## Run the offline visualization

Run all commands from the project directory:

```bash
python3 visualize_astar.py --clearance 0 --rpm 25 50 --start 300 300 0 --goal 5700 1700 --show
```

Positions and clearance are in millimetres; heading is in degrees. The script
saves `astar_navigation.mp4`. Omit `--show` for video output only. Planning may
take about a minute, depending on the machine.

## Build and run the simulation

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtlebot_navigator
source install/setup.bash
ros2 launch turtlebot_navigator arena.launch.py
```

Terminal 2, after Gazebo has spawned the robot:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtlebot_navigator navigate_robot.py --clearance 20 --rpm 25 50 --goal 3400 900 --show-path
```

Press a key in the preview window to begin driving. Keep Gazebo unpaused and
restart the simulation before each run. The example goal is Gazebo position
`(2.9, -0.099)` m with the default spawn. Use `--goal-heading DEGREES` to set the
final orientation. See the [package guide](turtlebot_navigator/PACKAGE_GUIDE.md) for details.

