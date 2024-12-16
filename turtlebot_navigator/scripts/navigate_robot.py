#!/usr/bin/env python3

"""Plan through the selected Gazebo world and drive the Waffle via ROS."""

import argparse
from math import atan2, cos, isfinite, radians, sin
from pathlib import Path
import sys
from time import monotonic

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from astar_search import PlanningMap, plan_path, wrap_angle
from motion_controller import OdometryWatchdog, TrackingError, TrajectoryFollower


class MoveRobot(Node):
    def __init__(self, path, goal_yaw, spawn, planning_map):
        super().__init__('move_robot')
        self.follower = TrajectoryFollower(path, goal_yaw, planning_map)
        self.spawn = spawn
        self.odom_origin = None
        self.pose = None
        self.finished = False
        self.failed = False
        self.odom_watchdog = OdometryWatchdog(monotonic())
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.timer = self.create_timer(0.05, self.follow_path)
        self.get_logger().info(f'Waiting for /odom to follow {len(path.motions)} planned motions')

    def update_odometry(self, message):
        position = message.pose.pose.position
        q = message.pose.pose.orientation
        values = (position.x, position.y, q.w, q.x, q.y, q.z)
        if not all(isfinite(value) for value in values) or sum(v * v for v in values[2:]) < 1e-8:
            self.stop_with_error('Invalid odometry received.')
            return
        self.odom_watchdog.received(monotonic())
        yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.odom_origin is None:
            self.odom_origin = position.x, position.y, yaw
        ox, oy, heading = self.odom_origin
        dx, dy = position.x - ox, position.y - oy
        self.pose = (
            self.spawn[0] + dx * cos(heading) + dy * sin(heading),
            self.spawn[1] - dx * sin(heading) + dy * cos(heading),
            wrap_angle(yaw - heading),
        )

    def stop_with_error(self, reason):
        self.failed = self.finished = True
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().error(reason)

    def follow_path(self):
        if self.finished:
            self.cmd_vel_pub.publish(Twist())
            return
        now = monotonic()
        try:
            self.odom_watchdog.check(now)
            if self.pose is None:
                return
            linear, angular = self.follower.command(self.pose, now)
        except TrackingError as error:
            self.stop_with_error(str(error))
            return
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self.cmd_vel_pub.publish(velocity)
        if self.follower.finished:
            self.finished = True
            self.get_logger().info(f'Goal reached at ({self.pose[0]:.2f}, {self.pose[1]:.2f}) m')


def parse_args(args):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', type=int, nargs=2, metavar=('X_MM', 'Y_MM'))
    parser.add_argument('--goal-heading', type=float, metavar='DEGREES')
    parser.add_argument('--clearance', type=int, metavar='MM')
    parser.add_argument('--rpm', type=int, nargs=2)
    parser.add_argument('--world', choices=['arena', 'test_arena'], default='arena')
    parser.add_argument('--spawn', type=float, nargs=2, default=(0.0, 0.0), metavar=('X_M', 'Y_M'))
    parser.add_argument('--show-path', action='store_true')
    options = parser.parse_args(args)
    if options.clearance is None:
        options.clearance = int(input('Extra obstacle clearance (mm): '))
    if options.rpm is None:
        options.rpm = int(input('First RPM: ')), int(input('Second RPM: '))
    if options.goal is None:
        options.goal = int(input('Goal X (mm): ')), int(input('Goal Y (mm): '))
        if options.goal_heading is None:
            options.goal_heading = float(input('Goal heading (degrees): '))
    if options.goal_heading is None:
        options.goal_heading = 0.0
    if not all(isfinite(value) for value in (*options.spawn, options.goal_heading)):
        parser.error('Spawn and goal heading must be finite.')
    if options.clearance < 0:
        parser.error('--clearance must be nonnegative.')
    return options


def main(args=None):
    args = sys.argv if args is None else [sys.argv[0], *args]
    try:
        options = parse_args(remove_ros_args(args)[1:])
        package = Path(get_package_share_directory('turtlebot_navigator'))
        world = package / 'worlds' / f'{options.world}.world'
        spawn = tuple(options.spawn)
        origin = spawn[0] - 0.500, spawn[1] - 0.999
        goal = tuple(offset + coordinate / 1000 for offset, coordinate in zip(origin, options.goal))
        planning_map = PlanningMap(origin, options.clearance / 1000, world)
        print('Planning A* route; the distance heuristic may take a while...', flush=True)
        path = plan_path(planning_map, (*spawn, 0.0), goal, *options.rpm)
        if options.show_path:
            image = planning_map.image()
            for previous, current in zip(path.samples, path.samples[1:]):
                cv2.line(image, planning_map.pixel(*previous[:2]), planning_map.pixel(*current[:2]), (0, 0, 255), 8)
            cv2.imshow('Planned path', cv2.resize(image, (1500, 500)))
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    except (ValueError, EOFError) as error:
        print(f'Could not plan a path: {error}', file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = MoveRobot(path, radians(options.goal_heading), spawn, planning_map)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if node.failed else 0


if __name__ == '__main__':
    sys.exit(main())
