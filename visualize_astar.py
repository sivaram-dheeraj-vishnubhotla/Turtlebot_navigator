#!/usr/bin/env python3

"""Run A* on the original 2D demo map and save an exploration video."""

import argparse
from math import radians
import sys
from time import monotonic

import cv2

from turtlebot_navigator.scripts.astar_search import PlanningMap, plan_path


def read_point(label):
    return int(input(f'{label} X (mm): ')), int(input(f'{label} Y (mm): '))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--clearance', type=int, metavar='MM')
    parser.add_argument('--rpm', nargs=2, type=int)
    parser.add_argument('--start', nargs=3, type=int, metavar=('X_MM', 'Y_MM', 'HEADING_DEG'))
    parser.add_argument('--goal', nargs=2, type=int, metavar=('X_MM', 'Y_MM'))
    parser.add_argument('--output', default='astar_navigation.mp4')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()

    try:
        clearance = args.clearance if args.clearance is not None else int(input('Extra obstacle clearance (mm): '))
        rpm = args.rpm or (int(input('First RPM: ')), int(input('Second RPM: ')))
        start = args.start or (*read_point('Start'), int(input('Start heading (degrees): ')))
        goal = args.goal or read_point('Goal')
        if clearance < 0:
            raise ValueError('Clearance must be nonnegative.')

        planning_map = PlanningMap((0.0, 0.0), clearance / 1000)
        image = planning_map.image()
        size = (1500, 500)
        video = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*'mp4v'), 30, size)
        if not video.isOpened():
            raise ValueError(f'Could not open video output {args.output}.')
        expanded = 0

        def save_frame():
            frame = cv2.resize(image, size)
            video.write(frame)
            if args.show:
                cv2.imshow('A* path planning', frame)
                cv2.waitKey(1)
            return frame

        def draw_expansion(pose):
            nonlocal expanded
            cv2.circle(image, planning_map.pixel(*pose[:2]), 4, (230, 100, 60), -1)
            expanded += 1
            if expanded % 500 == 0:
                save_frame()

        try:
            started = monotonic()
            start_pose = start[0] / 1000, start[1] / 1000, radians(start[2])
            target = goal[0] / 1000, goal[1] / 1000
            path = plan_path(planning_map, start_pose, target, *rpm, on_expand=draw_expansion)
            cv2.circle(image, planning_map.pixel(*path[0][:2]), 20, (180, 60, 220), -1)
            cv2.circle(image, planning_map.pixel(*target), 20, (0, 165, 255), -1)
            for index, (before, after) in enumerate(zip(path.samples, path.samples[1:])):
                cv2.line(image, planning_map.pixel(*before[:2]), planning_map.pixel(*after[:2]), (0, 0, 255), 8)
                if index % 10 == 0:
                    save_frame()
            last_frame = save_frame()
            for _ in range(60):
                video.write(last_frame)
            print(f'Found {len(path)} poses in {monotonic() - started:.2f}s. Saved {args.output}')
            if args.show:
                cv2.waitKey(0)
                cv2.destroyAllWindows()
        finally:
            video.release()
    except (ValueError, EOFError) as error:
        print(f'Could not plan a path: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
