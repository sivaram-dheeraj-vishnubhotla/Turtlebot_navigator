"""ROS-independent tracking of the motion arcs returned by A*."""

from bisect import bisect_right
from math import atan2, cos, dist, isfinite, sin

from astar_search import sample_motion, wrap_angle


class TrackingError(RuntimeError):
    """Motion must stop; a fresh plan or operator intervention is needed."""


class OdometryWatchdog:
    """Use receipt time so a frozen simulator clock cannot hide stale data."""

    def __init__(self, now):
        self.created = now
        self.last_received = None

    def received(self, now):
        self.last_received = now

    def check(self, now):
        if self.last_received is None:
            if now - self.created > 5.0:
                raise TrackingError('No odometry received within 5 seconds.')
        elif now - self.last_received > 1.0:
            raise TrackingError('Odometry is stale; stopping the robot.')


class TrajectoryFollower:
    MAX_LINEAR = 0.18
    MAX_ANGULAR = 1.5
    MAX_POSITION_ERROR = 0.06
    MAX_HEADING_ERROR = 0.8
    PROGRESS_TIMEOUT = 10.0

    def __init__(self, path, goal_yaw, planning_map):
        self.path = path
        self.goal_yaw = goal_yaw
        self.planning_map = planning_map
        self.started = None
        self.last_progress = None
        self.progress_pose = None
        self.finished = False
        self.ends = []
        self.scales = []
        elapsed = 0.0
        for motion in path.motions:
            # Slow both velocities equally, preserving curvature and leaving
            # headroom for odometry feedback corrections.
            scale = max(1.0, abs(motion.linear) / 0.12, abs(motion.angular) / 0.9)
            elapsed += motion.duration * scale
            self.scales.append(scale)
            self.ends.append(elapsed)
        self.duration = elapsed

    def reference(self, elapsed):
        index = bisect_right(self.ends, elapsed)
        if index == len(self.ends):
            return self.path[-1], 0.0, 0.0
        motion = self.path.motions[index]
        previous_end = self.ends[index - 1] if index else 0.0
        scale = self.scales[index]
        linear, angular = motion.linear / scale, motion.angular / scale
        pose = sample_motion(motion.samples[0], linear, angular,
                             max(0.0, elapsed - previous_end), steps=1)[-1]
        return pose, linear, angular

    def command(self, pose, now):
        if not all(isfinite(value) for value in (*pose, now)):
            raise TrackingError('Non-finite odometry or clock value.')
        if self.finished:
            return 0.0, 0.0
        if self.started is None:
            self.started = self.last_progress = now
            self.progress_pose = pose
        if (dist(pose[:2], self.progress_pose[:2]) > 0.005 or
                abs(wrap_angle(pose[2] - self.progress_pose[2])) > 0.04):
            self.progress_pose = pose
            self.last_progress = now
        if now - self.last_progress > self.PROGRESS_TIMEOUT:
            raise TrackingError('No motion progress for 10 seconds.')

        elapsed = now - self.started
        target, reference_linear, reference_angular = self.reference(elapsed)
        dx, dy = target[0] - pose[0], target[1] - pose[1]
        error = dist(pose[:2], target[:2])
        heading_error = wrap_angle(target[2] - pose[2])
        if error > self.MAX_POSITION_ERROR:
            raise TrackingError('Robot deviated more than 60 mm from the timed trajectory.')
        if elapsed < self.duration and abs(heading_error) > self.MAX_HEADING_ERROR:
            raise TrackingError('Robot heading deviated too far from the planned motion.')

        if elapsed >= self.duration:
            # Correct endpoint position before aligning to the final heading.
            if error > 0.015:
                angle = wrap_angle(atan2(dy, dx) - pose[2])
                linear = min(0.06, error) if abs(angle) < 0.35 else 0.0
                angular = 3.0 * angle
            else:
                angle = wrap_angle(self.goal_yaw - pose[2])
                if abs(angle) <= 0.08:
                    self.finished = True
                    return 0.0, 0.0
                linear, angular = 0.0, 3.0 * angle
        else:
            # Position error expressed in the robot's frame. The reference
            # velocities come from A*, rather than straight waypoint chords.
            forward = cos(pose[2]) * dx + sin(pose[2]) * dy
            lateral = -sin(pose[2]) * dx + cos(pose[2]) * dy
            linear = reference_linear * cos(heading_error) + 2.0 * forward
            angular = reference_angular + 8.0 * reference_linear * lateral + 3.0 * sin(heading_error)
        linear = max(0.0, min(self.MAX_LINEAR, linear))
        angular = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular))

        # Validate the actual corrected command, not just the original plan.
        samples = sample_motion(pose, linear, angular, duration=0.15)
        for before, after in zip(samples, samples[1:]):
            if not self.planning_map.segment_is_free(before[:2], after[:2]):
                raise TrackingError('Corrected motion would enter an obstacle exclusion zone.')
        return linear, angular
