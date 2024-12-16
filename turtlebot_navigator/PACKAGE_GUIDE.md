# TurtleBot Navigator package guide

See the [run instructions](../README.md) for the offline demo and Gazebo simulation.

## Simulation flow

1. A world launch starts Gazebo, spawns the Waffle, and starts the ROS bridge and
   robot state publisher. Gazebo calculates physics and wheel odometry.
2. The bridge forwards Gazebo odometry to ROS as `/odom`. The state publisher uses
   the URDF and `/joint_states` to publish the robot's local transforms. Gazebo
   supplies the transform from `odom` to `base_footprint`.
3. The planner reads the bundled world's box and cylinder collisions, inflates
   them for the robot's radius and requested clearance, and searches over
   position and heading. The follower waits for odometry and sends linear and
   angular velocity as `/cmd_vel`. The bridge forwards these to Gazebo's
   differential-drive system.

The assignment has two entry points: the offline A* visualization and the ROS
A* planner/follower. Both import the same shared planner. Planning uses saved
world geometry, not SLAM or live lidar mapping.

## Files

Paths are relative to this package unless they start with `../`.

| File | Role |
| --- | --- |
| [../visualize_astar.py](../visualize_astar.py) | Offline command-line entry point. Reads start, goal, clearance and RPM, then draws the A* exploration and route into an MP4 using OpenCV. |
| [scripts/astar_search.py](scripts/astar_search.py) | Shared planner. Rasterizes the offline map or Gazebo obstacle footprints, checks collisions along sampled differential-drive motions, and uses A* to recover a path from parent states. A straight-line distance lower bound guides search. Rounded continuous states still prevent a global shortest-path guarantee. |
| [scripts/navigate_robot.py](scripts/navigate_robot.py) | ROS planning and follower entry point. Maps user coordinates into Gazebo metres, obtains checked motion arcs from the shared planner, subscribes to odometry and publishes feedback-corrected velocity. |
| [scripts/motion_controller.py](scripts/motion_controller.py) | ROS-independent tracking of the planned differential-drive arcs, command collision checks, progress monitoring and an odometry watchdog. Imported by the ROS entry point. |
| [launch/arena.launch.py](launch/arena.launch.py) | Selects the competition arena and includes the shared simulation launch. |
| [launch/test_arena.launch.py](launch/test_arena.launch.py) | Selects the sparse test arena and includes the shared simulation launch. |
| [launch/gazebo.launch.py](launch/gazebo.launch.py) | Common orchestration for Gazebo, the model search path, bridge, state publisher and robot spawn. |
| [launch/spawn_robot.launch.py](launch/spawn_robot.launch.py) | Uses `ros_gz_sim create` to spawn the bundled Waffle SDF at the requested X/Y position. |
| [launch/robot_frames.launch.py](launch/robot_frames.launch.py) | Loads the local Waffle URDF and starts the ROS transform publisher. |
| [config/ros_gazebo_topics.yaml](config/ros_gazebo_topics.yaml) | Defines ROS/Gazebo topic conversions. Commands travel to Gazebo. Clock, odometry, TF, joint states, lidar, IMU and camera data travel to ROS. |
| [CMakeLists.txt](CMakeLists.txt) | Installs the Python planner/follower, launch files and simulation assets for ROS to find. |
| [package.xml](package.xml) | Declares the ROS package name and its build and runtime dependencies. |

### Simulation assets and documentation

| File | Role |
| --- | --- |
| [worlds/arena.world](worlds/arena.world) | Main arena. Describes lighting, ground, boxes, a cylinder and Gazebo world systems. |
| [worlds/test_arena.world](worlds/test_arena.world) | Sparse test arena. Contains the ground plane and one distant cylinder. |
| [models/navigator_waffle/waffle_robot.sdf](models/navigator_waffle/waffle_robot.sdf) | Active simulated robot. Defines mass, collisions, wheels, caster links, lidar, IMU, camera and the Gazebo drive and joint-state systems. |
| [models/navigator_waffle/model.config](models/navigator_waffle/model.config) | Waffle model metadata pointing to the active Gazebo SDF. |
| [models/navigator_meshes/model.config](models/navigator_meshes/model.config) | Metadata for the shared mesh collection. |
| [models/navigator_meshes/meshes/waffle_chassis.dae](models/navigator_meshes/meshes/waffle_chassis.dae) | Waffle chassis mesh. |
| [models/navigator_meshes/meshes/drive_wheel.dae](models/navigator_meshes/meshes/drive_wheel.dae) | Shared wheel mesh used by the Waffle. |
| [models/navigator_meshes/meshes/lidar_housing.dae](models/navigator_meshes/meshes/lidar_housing.dae) | Laser scanner housing mesh used by the Waffle. |
| [models/navigator_meshes/meshes/camera_housing.dae](models/navigator_meshes/meshes/camera_housing.dae) | RealSense camera housing mesh used by the Waffle. |
| [urdf/navigator_waffle.urdf](urdf/navigator_waffle.urdf) | ROS link and joint tree for transforms and RViz visualization, with references to local meshes. |
| [rviz/navigation.rviz](rviz/navigation.rviz) | Optional RViz layout with robot model, laser scan, odometry and transforms in the `odom` frame. |
| [../README.md](../README.md) | Project overview and run instructions. |
| [../search_preview.gif](../search_preview.gif) | Existing animated preview used by the repository README. |
| [PACKAGE_GUIDE.md](PACKAGE_GUIDE.md) | This package guide. |

## Planning and tracking checks

A* searches differential-drive motion primitives. Its heuristic
is `max(0, straight_line_distance - goal_tolerance)`, a lower bound on remaining
translation cost. This estimate can produce large searches around obstacles.
State merging at 20 mm / 10 degrees remains an approximation: the implementation
does not guarantee a globally shortest path or completeness in continuous space.

Obstacle inflation uses a 240 mm radius covering the bundled Waffle chassis,
plus user clearance and a 2 mm raster margin. Each integration segment is checked
against the distance field; near obstacles it is sampled at intervals no greater
than 1 mm. The saved path includes endpoint poses, motion velocities/durations,
and the sampled arcs. Both previews draw those arcs.

The follower slows each arc uniformly to preserve its curvature, then uses
odometry feedback to correct position and heading. Reference limits are 0.12 m/s
and 0.9 rad/s, with correction limits of 0.18 m/s and 1.5 rad/s. It checks the
corrected command over the next 0.15 seconds against the same static map.
These checks do not model braking distance or detect newly moved obstacles.

The ROS process publishes a stop and returns failure if no odometry arrives
within 5 seconds, odometry is more than 1 second old, tracking position error
exceeds 60 mm, tracking heading error exceeds 0.8 rad, no measurable movement
occurs for 10 seconds, or the corrected command enters an exclusion zone.
Timeouts and trajectory timing use monotonic wall time: pausing or substantially
slowing Gazebo may trigger a stop. Reset the robot to the configured spawn before
starting a fresh run. The controller aligns the final heading after positional
planning.
