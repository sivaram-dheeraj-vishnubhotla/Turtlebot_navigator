"""Differential-drive A* for the offline demo and TurtleBot simulation."""

import heapq
from dataclasses import dataclass
from itertools import count
from math import ceil, cos, dist, isfinite, pi, sin, tau
import xml.etree.ElementTree as ET

import cv2
import numpy as np

WIDTH_MM = 6000
HEIGHT_MM = 2000
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.287
# The chassis reaches hypot(0.064 + 0.265/2, 0.265/2) = 0.237 m
# from the drive origin in the bundled Waffle SDF. Round up to cover it.
ROBOT_RADIUS = 0.240
RASTER_MARGIN = 0.002
GOAL_TOLERANCE = 0.020
MOTION_DURATION = 0.6


def wrap_angle(angle):
    return (angle + pi) % tau - pi


class PlanningMap:
    """One millimetre per pixel, with world Y increasing upwards."""

    def __init__(self, origin, clearance, world=None):
        if not isfinite(clearance) or clearance < 0:
            raise ValueError('Clearance must be finite and nonnegative.')
        self.origin = origin
        self.obstacles = np.zeros((HEIGHT_MM, WIDTH_MM), dtype=np.uint8)
        self.obstacles[[0, -1], :] = 1
        self.obstacles[:, [0, -1]] = 1
        if world is None:
            cv2.rectangle(self.obstacles, (1500, 0), (1750, 999), 1, -1)
            cv2.rectangle(self.obstacles, (2500, 1000), (2750, 1999), 1, -1)
            cv2.circle(self.obstacles, (4200, 800), 600, 1, -1)
        else:
            self.add_world_obstacles(world)
        distance_mm = cv2.distanceTransform(1 - self.obstacles, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        self.required_distance = ROBOT_RADIUS + clearance + RASTER_MARGIN
        self.distance = distance_mm / 1000
        self.free = self.distance > self.required_distance

    def pixel(self, x, y):
        return round((x - self.origin[0]) * 1000), HEIGHT_MM - 1 - round((y - self.origin[1]) * 1000)

    def add_world_obstacles(self, world):
        # The two bundled worlds use one link and a box or cylinder per obstacle.
        for model in ET.parse(world).findall('./world/model'):
            if model.get('name') == 'ground_plane':
                continue
            x, y, _, _, _, yaw = map(float, model.findtext('pose').split())
            geometry = model.find('./link/collision/geometry')
            if geometry.find('cylinder') is not None:
                radius = round(float(geometry.findtext('cylinder/radius')) * 1000)
                cv2.circle(self.obstacles, self.pixel(x, y), radius, 1, -1)
            else:
                width, height, _ = map(float, geometry.findtext('box/size').split())
                corners = [
                    self.pixel(x + dx * cos(yaw) - dy * sin(yaw), y + dx * sin(yaw) + dy * cos(yaw))
                    for dx, dy in (
                        (-width / 2, -height / 2), (width / 2, -height / 2),
                        (width / 2, height / 2), (-width / 2, height / 2),
                    )
                ]
                cv2.fillConvexPoly(self.obstacles, np.array(corners, dtype=np.int32), 1)

    def is_free(self, x, y):
        column, row = self.pixel(x, y)
        return 0 <= column < WIDTH_MM and 0 <= row < HEIGHT_MM and self.free[row, column]

    def segment_is_free(self, before, after):
        """Check the swept centre segment, conservatively including pixel rounding.

        A distance-field bound certifies segments far from obstacles. Near an
        obstacle, sample at <= 1 mm; the 2 mm raster margin also covers the tiny
        arc/chord error for the <= 10 mm, <= 0.1 rad motion integration steps.
        """
        if not self.is_free(*before) or not self.is_free(*after):
            return False
        length = dist(before, after)
        column, row = self.pixel(*before)
        if self.distance[row, column] > self.required_distance + length + 0.001:
            return True
        steps = max(1, ceil(length / 0.001))
        return all(self.is_free(
            before[0] + (after[0] - before[0]) * i / steps,
            before[1] + (after[1] - before[1]) * i / steps,
        ) for i in range(1, steps))

    def image(self):
        image = np.full((HEIGHT_MM, WIDTH_MM, 3), 235, dtype=np.uint8)
        image[self.free] = (30, 30, 30)
        image[self.obstacles != 0] = (0, 150, 0)
        return image


def motion_actions(rpm1, rpm2):
    pairs = dict.fromkeys([
        (0, rpm1), (rpm1, 0), (rpm1, rpm1), (0, rpm2), (rpm2, 0),
        (rpm2, rpm2), (rpm1, rpm2), (rpm2, rpm1), (-rpm1, rpm1), (rpm1, -rpm1),
    ])
    for left, right in pairs:
        linear = WHEEL_RADIUS * pi * (left + right) / 60
        angular = WHEEL_RADIUS * tau * (right - left) / (60 * WHEEL_SEPARATION)
        steps = max(6, ceil(linear * MOTION_DURATION / 0.010), ceil(abs(angular) * MOTION_DURATION / 0.1))
        yield linear, angular, steps


def sample_motion(pose, linear, angular, duration=MOTION_DURATION, steps=None):
    """Integrate an arc exactly; return samples including its starting pose."""
    if steps is None:
        steps = max(6, ceil(abs(linear) * duration / 0.010),
                    ceil(abs(angular) * duration / 0.1))
    x, y, heading = pose
    samples = [pose]
    dt = duration / steps
    turn = angular * dt
    step = linear * dt if turn == 0 else 2 * linear * sin(turn / 2) / angular
    for _ in range(steps):
        x += step * cos(heading + turn / 2)
        y += step * sin(heading + turn / 2)
        heading += turn
        samples.append((x, y, wrap_angle(heading)))
    return samples


def advance(pose, action, planning_map):
    linear, angular, steps = action
    samples = sample_motion(pose, linear, angular, steps=steps)
    for before, after in zip(samples, samples[1:]):
        if not planning_map.segment_is_free(before[:2], after[:2]):
            return None
    return samples[-1]


@dataclass(frozen=True)
class PlannedMotion:
    linear: float
    angular: float
    duration: float
    samples: tuple


class PlannedPath(list):
    """Endpoint poses, with the checked arcs retained for display and control."""

    def __init__(self, poses, actions):
        super().__init__(poses)
        self.motions = [PlannedMotion(action[0], action[1], MOTION_DURATION,
                       tuple(sample_motion(pose, *action[:2], steps=action[2])))
                        for pose, action in zip(poses, actions)]
        self.samples = [poses[0]]
        for motion in self.motions:
            self.samples.extend(motion.samples[1:])


def goal_distance(pose, goal):
    # A lower bound: every remaining route must cover this much translation.
    return max(0.0, dist(pose[:2], goal) - GOAL_TOLERANCE)


def state_key(pose):
    x, y, heading = pose
    return round(x / 0.020), round(y / 0.020), round(heading / (tau / 36)) % 36


def plan_path(planning_map, start, goal, rpm1, rpm2, on_expand=None):
    if not all(isfinite(value) for value in (*start, *goal, rpm1, rpm2)):
        raise ValueError('Start, goal and RPM values must be finite.')
    if not planning_map.is_free(*start[:2]):
        raise ValueError('Start is outside the free space for this clearance.')
    if not planning_map.is_free(*goal):
        raise ValueError('Goal is outside the free space for this clearance.')
    if not (0 < rpm1 and 0 < rpm2):
        raise ValueError('Both RPM values must be positive.')

    def estimate(pose):
        return goal_distance(pose, goal)

    actions = list(motion_actions(rpm1, rpm2))
    sequence = count()
    frontier = [(estimate(start), next(sequence), 0.0, start, (start, None, None))]
    costs = {state_key(start): 0.0}
    while frontier:
        _, _, cost, pose, history = heapq.heappop(frontier)
        if cost != costs[state_key(pose)]:
            continue
        if dist(pose[:2], goal) <= GOAL_TOLERANCE:
            path, chosen_actions = [], []
            while history is not None:
                point, action, history = history
                path.append(point)
                if action is not None:
                    chosen_actions.append(action)
            return PlannedPath(list(reversed(path)), list(reversed(chosen_actions)))

        if on_expand is not None:
            on_expand(pose)
        for action in actions:
            neighbor = advance(pose, action, planning_map)
            if neighbor is None:
                continue
            linear, angular, _ = action
            new_cost = cost + (linear + abs(angular) * 0.05) * MOTION_DURATION
            key = state_key(neighbor)
            if new_cost < costs.get(key, float('inf')):
                costs[key] = new_cost
                heapq.heappush(frontier, (
                    new_cost + estimate(neighbor), next(sequence), new_cost, neighbor, (neighbor, action, history),
                ))
    raise ValueError('No route found for these inputs.')
